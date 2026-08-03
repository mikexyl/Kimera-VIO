# VLAD Sequence Frame Selection Logic

This note summarizes the current sequence frame admission, finalization, and
fixed-length selection logic in `src/loopclosure/VLADLoopClosureDetector.cpp`.

## 1. Frame Admission Into the Active Sequence

Each incoming LCD frame reaches `computeSequenceGlobalDesc(target_frame_id,
add_to_sequence)`.

A frame is only considered for the active VPR sequence when:

- The frontend tracking status marks the frame as valid.
- `add_to_sequence == true`.
- The frame is far enough in frame-id space from the last admitted sequence
  frame.

The interval rule is:

```cpp
vpr_seq_interval = max(1, lcd_params_.vpr_seq_interval_);

if (!new_seq_frames_.empty() &&
    target_frame_id < new_seq_frames_.back()->id_ + vpr_seq_interval) {
  return;
}
```

So the active sequence is already downsampled before final VPR model selection:
it keeps at most one frame per `vpr_seq_interval` frame-id interval.

This admission and boundary logic is used for every VPR model, including
single-image models such as MixVPR. A single-image model no longer interprets
`vpr_seq_interval` as an immediate output stride.

## 2. Sequence Boundary / Finalization Logic

After a frame is admitted into `new_seq_frames_`, the code may finalize the
current sequence.

There are two boundary mechanisms.

### Max Travel Distance Boundary

If `lcd_params_.vpr_max_sequence_distance_m_ > 0.0`, the code computes the
cumulative traveled distance through `new_seq_frames_`.

If the sequence travel distance exceeds `vpr_max_sequence_distance_m_`:

- The frames before the overflow frame are finalized.
- The overflow frame and later frames roll over into the next sequence.
- The rollover frames are assigned the next `seq_id_`.

Conceptually:

```cpp
distance_overflow_idx =
    findSequenceDistanceOverflowIndex(new_seq_frames_,
                                      vpr_max_sequence_distance_m);

if (distance_overflow_idx < new_seq_frames_.size()) {
  finalize(new_seq_frames_[0 : distance_overflow_idx]);
  new_seq_frames_ = new_seq_frames_[distance_overflow_idx : end];
}
```

### Covisibility Boundary

If the max-distance boundary does not trigger, the code checks covisibility
against the current anchor frame.

The boundary condition is:

```cpp
reached_sequence_boundary_now =
    new_seq_frames_.size() > 1 &&
    landmark_manager_->computeCovisibilityScore(anchor_frame_id,
                                                target_frame_id) <
        lcd_params_.max_covisibility_score_;
```

Once this condition becomes true, `active_seq_boundary_reached_` stays true
until the sequence is finalized or reset.

If the sequence boundary is reached, the code calls:

```cpp
finalizeSequenceFrames(new_seq_frames_, false);
```

If finalization succeeds, `new_seq_frames_` is cleared and the next sequence
starts fresh.

## 3. Short Sequence Policy

Sequence finalization first checks the model-independent configured minimum:

```cpp
min_sequence_frames = lcd_params_.vpr_min_sequence_frames_;
```

If the current sequence has fewer frames than `min_sequence_frames`, behavior
depends on `lcd_params_.vpr_short_sequence_policy_`. The default value is five,
which preserves the existing JIST sequence boundary behavior while allowing a
single-image model to use exactly the same finalized sequences.

If the policy is `wait` and finalization was not forced:

```cpp
if (sequence_frames.size() < min_sequence_frames &&
    !force_finalize_short_sequence &&
    vpr_short_sequence_policy == wait) {
  return false;
}
```

So the detector waits for the configured model-independent sequence minimum.

After extraction, the VPR model's required input length is queried. Candidate
sequences are reduced to that length, or short inputs are expanded with
`duplicateSequenceFramesInOrder()`.

That function samples evenly spaced positions over the available frames, using
rounded indices, so some frames may be repeated when the sequence is shorter
than the VPR model input length.

## 4. Fixed-Length Frame Selection for the VPR Model

When the sequence has at least `target_seq_length` frames, the code calls:

```cpp
selectDiverseSequenceFrames(sequence_frames, target_seq_length);
```

This function chooses exactly `target_seq_length` frames from the candidate
sequence.

For a single-image model, the sequence's final boundary frame is selected. It
therefore produces one descriptor at the same sequence endpoint where JIST's
multi-frame descriptor is attached.

The selection process is:

1. Compute cumulative traveled distance over the candidate frames using pose
   translation distance.
2. Compute an ideal motion spacing:

```cpp
min_motion_spacing_m =
    total_distance_m / static_cast<double>(target_size - 1);
```

3. For each desired output slot, compute an ideal temporal/index position
   across the sequence.
4. Search the valid remaining index range so selections stay ordered and leave
   enough frames for later slots.
5. Score each candidate using:

```cpp
score = repeated_frame_penalty + position_deviation;
```

The candidate with the lowest score is selected.

## 5. `repeated_frame_penalty`

`repeated_frame_penalty` is not based on covisibility.

It is based only on traveled translation distance since the last selected
frame.

The logic is:

```cpp
motion_since_last_pick_m =
    cumulative_distance_m[candidate_idx] - cumulative_distance_m[last_idx];

normalized_motion_shortfall =
    max(0.0, min_motion_spacing_m - motion_since_last_pick_m) /
    min_motion_spacing_m;

repeated_frame_penalty = 10.0 * normalized_motion_shortfall;
```

This penalizes selecting a frame that is too close in traveled distance to the
previous selected frame. The intent is to avoid near-stationary repeated views
before refining the choice using temporal coverage.

If the candidate has moved at least `min_motion_spacing_m` from the last
selected frame, the penalty is zero.

If it has moved less than that, the penalty grows in proportion to the motion
shortfall.

## 6. `position_deviation`

After the motion-based repeat penalty, the selector also tries to keep selected
frames spread across the full sequence.

The temporal/index coverage term is:

```cpp
position_deviation =
    abs(candidate_idx - ideal_position) / candidate_frames.size();
```

This encourages selected frames to cover the beginning, middle, and end of the
sequence rather than clustering in one part.

## 7. Summary

The current logic is:

```text
valid LCD frames
  -> admitted only every vpr_seq_interval frame ids
  -> accumulated in new_seq_frames_
  -> finalized by max travel distance or low covisibility to anchor
  -> if too short, either wait or duplicate frames
  -> if long enough, select fixed-length frames using:
       motion-based repeated_frame_penalty
       + temporal/index position_deviation
  -> send selected frames to the VPR model
```

Important distinction:

- Covisibility is used to decide sequence boundaries.
- `repeated_frame_penalty` is used during intra-sequence frame selection.
- `repeated_frame_penalty` is based on pose translation travel distance, not
  covisibility.
