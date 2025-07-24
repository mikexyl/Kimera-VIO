#pragma once

#include <faiss/gpu/GpuResources.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include <opencv2/calib3d.hpp>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {

struct CuGemmContext {
  float *d_A = nullptr, *d_B = nullptr, *d_C = nullptr;
  float *h_A = nullptr, *h_B = nullptr;
  int max_N1 = 0, max_N2 = 0, max_D = 0;
  cublasHandle_t handle;
  cudaStream_t stream;

  void init(int N1, int N2, int D) {
    max_N1 = N1;
    max_N2 = N2;
    max_D = D;

    cudaHostAlloc(&h_A, N1 * D * sizeof(float), cudaHostAllocDefault);
    cudaHostAlloc(&h_B, N2 * D * sizeof(float), cudaHostAllocDefault);

    cudaMalloc(&d_A, N1 * D * sizeof(float));
    cudaMalloc(&d_B, N2 * D * sizeof(float));
    cudaMalloc(&d_C, N1 * N2 * sizeof(float));

    cudaStreamCreate(&stream);
    cublasCreate(&handle);
    cublasSetStream(handle, stream);
  }

  void destroy() {
    if (h_A) cudaFreeHost(h_A);
    if (h_B) cudaFreeHost(h_B);
    if (d_A) cudaFree(d_A);
    if (d_B) cudaFree(d_B);
    if (d_C) cudaFree(d_C);
    cublasDestroy(handle);
    cudaStreamDestroy(stream);
  }

  cv::Mat compute(const cv::Mat& desc1, const cv::Mat& desc2) {
    int N1 = desc1.rows;
    int N2 = desc2.rows;
    int D = desc1.cols;

    CV_Assert(desc1.type() == CV_32F && desc2.type() == CV_32F);
    CV_Assert(desc2.cols == D);

    // Copy data into pinned host memory
    memcpy(h_A, desc1.ptr<float>(), N1 * D * sizeof(float));
    memcpy(h_B, desc2.ptr<float>(), N2 * D * sizeof(float));

    cudaMemcpyAsync(
        d_A, h_A, N1 * D * sizeof(float), cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(
        d_B, h_B, N2 * D * sizeof(float), cudaMemcpyHostToDevice, stream);

    const float alpha = 1.0f;
    const float beta = 0.0f;

    cublasSgemm(handle,
                CUBLAS_OP_T,
                CUBLAS_OP_N,
                N1,
                N2,
                D,
                &alpha,
                d_A,
                D,
                d_B,
                D,
                &beta,
                d_C,
                N1);

    cv::Mat result(N2, N1, CV_32F);

    cudaMemcpy(result.ptr<float>(),
               d_C,
               N1 * N2 * sizeof(float),
               cudaMemcpyDeviceToHost);
    return result;
  }
};

inline std::vector<cv::DMatch> matchWithInitialFlow(
    const cv::Mat& descriptors1,
    const std::vector<cv::Point2f>& keypoints1,
    const cv::Mat& descriptors2,
    const std::vector<cv::Point2f>& keypoints2,
    const std::vector<cv::Point2f>& predictedPts,
    float searchRadius,
    int normType = cv::NORM_L2) {
  CV_Assert(descriptors1.rows == (int)keypoints1.size());
  CV_Assert(descriptors2.rows == (int)keypoints2.size());
  CV_Assert(descriptors1.rows == (int)predictedPts.size());
  CV_Assert(descriptors1.type() == descriptors2.type());

  const int N = descriptors1.rows;
  const int M = descriptors2.rows;

  tbb::concurrent_vector<cv::DMatch> concurrent_matches;

  tbb::parallel_for(0, N, [&](int i) {
    const cv::Point2f& pred = predictedPts[i];
    float bestDist = std::numeric_limits<float>::max();
    int bestJ = -1;

    for (int j = 0; j < M; ++j) {
      float dx = std::abs(pred.x - keypoints2[j].x);
      float dy = std::abs(pred.y - keypoints2[j].y);
      if (dx > searchRadius || dy > searchRadius) continue;

      float d = cv::norm(descriptors1.row(i), descriptors2.row(j), normType);
      if (d < bestDist && d < 0.5f) {
        bestDist = d;
        bestJ = j;
      }
    }

    if (bestJ >= 0) {
      concurrent_matches.emplace_back(cv::DMatch(i, bestJ, bestDist));
    }
  });

  // Convert concurrent_vector to std::vector
  return std::vector<cv::DMatch>(concurrent_matches.begin(),
                                 concurrent_matches.end());
}

inline std::vector<cv::DMatch> matchWithInitialFlowCuBLAS(
    CuGemmContext* ctx,
    const cv::Mat& descriptors1,
    const std::vector<cv::Point2f>& keypoints1,
    const cv::Mat& descriptors2,
    const std::vector<cv::Point2f>& keypoints2,
    const std::vector<cv::Point2f>& predictedPts,
    float searchRadius) {
  if (descriptors1.empty() || descriptors2.empty()) {
    std::cerr << "Empty descriptors passed to ONNX model." << std::endl;
    return {};
  }

  CV_Assert(descriptors1.rows == (int)keypoints1.size());
  CV_Assert(descriptors2.rows == (int)keypoints2.size());
  CV_Assert(descriptors1.rows == (int)predictedPts.size());
  CV_Assert(descriptors1.type() == CV_32F && descriptors2.type() == CV_32F);
  CV_Assert(descriptors1.cols == descriptors2.cols);

  int N1 = descriptors1.rows;
  int N2 = descriptors2.rows;

  // Step 1: compute inner product matrix [N1 x N2] with timing
  cv::Mat scoreMat = ctx->compute(descriptors2, descriptors1);
  tbb::concurrent_vector<cv::DMatch> concurrent_matches;

  tbb::parallel_for(0, N1, [&](int i) {
    const cv::Point2f& pred = predictedPts[i];
    float bestScore = 0.5;
    int bestJ = -1;

    const float* score_row = scoreMat.ptr<float>(i);
    for (int j = 0; j < N2; ++j) {
      float dx = std::abs(pred.x - keypoints2[j].x);
      float dy = std::abs(pred.y - keypoints2[j].y);
      if (dx > searchRadius * 2 || dy > searchRadius * 2)
        continue;  // Skip points outside the search radius

      // float weight = std::exp(-dist2 / (2 * sigma * sigma));
      float weight = 1;
      float adjusted_score = score_row[j] * weight;
      if (adjusted_score > bestScore) {
        bestScore = adjusted_score;
        bestJ = j;
      }
    }

    if (bestJ >= 0)
      concurrent_matches.emplace_back(cv::DMatch(i, bestJ, -bestScore));
  });

  // Convert to std::vector
  std::vector<cv::DMatch> matches(concurrent_matches.begin(),
                                  concurrent_matches.end());
  return matches;
}

class FlannTracker : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(FlannTracker);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FlannTracker);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FlannTracker(int nkpts) {
    ctx_ = std::make_unique<CuGemmContext>();
    ctx_->init(nkpts, nkpts, 64);  // Assuming 128 is the descriptor size
  }
  virtual ~FlannTracker() = default;

  void track(Frame* ref_frame,
             Frame* cur_frame,
             cv::InputArray prevPts,
             cv::InputOutputArray nextPts,
             cv::OutputArray status,
             cv::OutputArray err,
             cv::Size winSize = cv::Size(21, 21),
             int maxLevel = 3,
             cv::TermCriteria criteria = cv::TermCriteria(
                 cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                 30,
                 0.01),
             int flags = 0,
             double minEigThreshold = 1e-4) override {
    throw std::runtime_error(
        "FlannMatcher does not support optical flow tracking. "
        "Use LighterGlueCV or OpticalFlowCV instead.");
  }

  void trackDesc(Frame* ref_frame,
                 Frame* cur_frame,
                 cv::Mat homography,
                 int search_radius,
                 const std::vector<cv::Point2f>& predictedPts,
                 DMatchVec* matches) override {
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    CHECK_NOTNULL(matches);

    matches->clear();

    *matches = matchWithInitialFlowCuBLAS(ctx_.get(),
                                          ref_frame->descriptors_,
                                          ref_frame->keypoints_,
                                          cur_frame->descriptors_,
                                          cur_frame->keypoints_,
                                          predictedPts,
                                          15.0f);
  }

  std::unique_ptr<CuGemmContext> ctx_;
};
}  // namespace VIO