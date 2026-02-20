# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Standard CMake build (Release by default)
mkdir -p build && cd build
cmake ..
make -j4

# Build with tests disabled
cmake -DKIMERA_BUILD_TESTS=OFF ..

# Run all tests
cd build && ./testKimeraVIO

# Run a specific test (gtest regex filter)
./testKimeraVIO --gtest_filter=*TestName*
```

The build auto-downloads the ORB vocabulary file (`vocabulary/ORBvoc.yml`) from Dropbox if missing. C++17 is required. Default build type is Release.

## Running the Pipeline

```bash
# EuRoC dataset with loop closure detection enabled
bash ./scripts/stereoVIOEuroc.bash -p "PATH_TO_DATASET/V1_01_easy" -lcd

# With logging
bash ./scripts/stereoVIOEuroc.bash -p "PATH_TO_DATASET" -log

# Single-threaded sequential mode (for debugging/reproducibility)
bash ./scripts/stereoVIOEuroc.bash -p "PATH_TO_DATASET" -s

# Monocular mode: edit PARAMS_PATH in the script to ../params/EurocMono
```

See `docs/gflags_parameters.md` for all command-line flags.

## Code Architecture

The pipeline runs as a linear chain of modules, each processing data asynchronously:

```
DataProvider → IMU Frontend → Visual Frontend → Backend → Loop Closure Detector → Mesh/Visualizer
```

- **`pipeline/`** — Orchestrates the above chain; `Pipeline.h` is the top-level entry point.
- **`frontend/`** — Stereo/mono visual tracking. Supports optical flow (ViLib or OpenCV), LighterGlue deep-learning matcher, and ANMS-based feature detection. `StereoVisionImuFrontend` is the main class.
- **`imu-frontend/`** — IMU preintegration (Forster et al. 2016) feeding into both the frontend and backend.
- **`backend/`** — GTSAM factor graph optimization. Handles VIO state estimation with IMU preintegration factors and optional structural regularity factors.
- **`loopclosure/`** — DBoW2 bag-of-words place recognition + Kimera-RPGO graph optimization. Disabled by default; enabled via `-lcd` flag.
- **`dataprovider/`** — Loads and time-synchronizes stereo images and IMU data from datasets (EuRoC format primarily).
- **`mesh/`** — 3D mesh generation from stereo depth.
- **`factors/`** — Custom GTSAM factors (structural regularities, etc.).
- **`utils/`** — Math helpers, numerical utilities, OpenCV wrappers.

Key design pattern: heavy use of **PIMPL** (`std::unique_ptr`) between modules to minimize header dependencies.

## Code Style

Follows Google C++ Style Guide with ASL modifications (see `docs/developer_guide.md`):

- **Line limit:** 80 characters
- **Parameter passing:** All inputs (including primitives) by `const&`; outputs and in/out params by pointer
- **No default parameter values** — use overloads instead
- **Null checks:** Use `CHECK_NOTNULL(ptr)` (glog) for raw pointers, `CHECK(shared_ptr)` for shared pointers
- **Assertions:** Use `CHECK(x)`, `CHECK_EQ`, `CHECK_LT`, etc. (glog macros, not `assert`) — these run regardless of NDEBUG
- **Naming:** Variables as `snake_case`, frames as `A_B_quantity` (e.g., `W_T_B` for pose of Body in World)
- **Logging:** Use `glog` macros (`LOG(INFO)`, `VLOG(...)`)
- **Flags:** Use `gflags` (`DEFINE_*` / `DECLARE_*`) — never hardcode values
- **Comments:** Start with capital letter, end with period. Only comment non-obvious logic.
- **Formatting:** Run `clang-format` (Google style, configured in `.clang-format`)

## Testing

Test files are in `tests/`. Each major module has a corresponding `test*.cpp`. Key test files:
- `testStereoImuPipeline.cpp` — full pipeline integration test
- `testTracker.cpp`, `testLoopClosureDetector.cpp` — largest/most complex tests
- `testEdgeSelection.cpp` — new, untracked (current branch work)

Test data is in `tests/data/` (MicroEurocDataset subset).

## Active Development Context

Current branch `dev/code-slam` has active work on:
- Loop closure sequence scoring (`feature/sequence_scoring` merged in)
- Edge-based feature selection (`include/kimera-vio/frontend/edge-selection/` — untracked)
- New dataset params (`params/V4RLD455_1/`)
- Modified: `include/kimera-vio/loopclosure/LoopClosureDetector-inl.h`
