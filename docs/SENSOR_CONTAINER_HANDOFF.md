# Sensor Container Handoff

This document is for the next Codex / engineer who will implement the **real-sensor-side container** that feeds the Alpamayo planner.

## Critical Assumption

The `sensor container` does **not** contain this TensorRT repo and should **not** depend on importing code from it.

That means this document must be treated as a **standalone interface contract**:

- the sensor container is a separate component
- it is allowed to know the planner input schema
- it should not rely on TensorRT-Edge-LLM source files being present at runtime
- repo file links in this document are **background references only**, not implementation dependencies

The real goal is to define **what the sensor container must produce** so that the planner container can consume it.

## Goal

Build a `sensor container` that:

- ingests live camera + state sensors
- downsizes / preprocesses camera frames before planner inference
- synchronizes timestamps across sensors
- constructs planner-ready samples at `10 Hz`
- publishes those samples to the planner/inference container

This container should **not** own:

- the planner model runtime
- TensorRT engine loading
- request JSON generation for production
- the `50 Hz` control UDP publisher

Its job is to prepare synchronized sensor inputs for the planner.

## What The Sensor Container Must Deliver

The planner team needs a container that can deliver a `10 Hz` synchronized sample with:

- resized multi-camera image frames
- ego-history pose tensors
- timestamps
- stable camera ordering / camera IDs

The sensor container should be judged on whether it can reliably produce those samples, not whether it knows anything about TensorRT internals.

## Fixed Decisions

The following are fixed for the current integration unless the planner team explicitly changes them later:

- camera source baseline on Thor/GMSL: `nvsiplsrc`
- planner camera order: `left`, `front`, `right`, `front_tele`
- planner camera IDs: `0`, `1`, `2`, `6`
- image format: `RGB uint8`
- image tensor shape: `[Cam, T, 3, 320, 576]`
- resize policy: simple resize + bicubic
- external time basis: `UTC us`
- `T=4` frame offsets: `t0-300 ms`, `t0-200 ms`, `t0-100 ms`, `t0`
- frame selection: nearest frame to each target timestamp
- `ego_history_xyz` source: GNSS/INS fused pose
- `ego_history_rot` source: INS/Xsens orientation converted to rotation matrix
- invalid/stale defaults:
  - camera target mismatch `> 50 ms`
  - localization stale `> 100 ms`
  - camera-state skew at `t0` `> 50 ms`
- `/latest`: return the last valid sample; if no valid sample exists yet, return `503`
- `clip_id`: stable run/segment identifier

## Contract-First View

Think of the system boundary like this:

```text
sensor container
  -> produces planner sample contract
planner container
  -> consumes planner sample contract
```

The contract is the important part.

## Planner Input Contract

For Phase 1 bring-up, the contract should match the already working live sample format used by the planner side.

### Required Sample Keys

Each produced sample must contain:

- `image_frames`
- `camera_indices`
- `ego_history_xyz`
- `ego_history_rot`
- `relative_timestamps`
- `absolute_timestamps`
- `t0_us`
- `fixed_delta_seconds`
- `clip_id`
- `camera_order`

### Required Shapes

Current working planner expectation:

- `image_frames`: `[Cam, T, C, H, W]`
- `ego_history_xyz`: `[1, 1, 16, 3]`
- `ego_history_rot`: `[1, 1, 16, 3, 3]`
- `camera_indices`: `[Cam]`
- `relative_timestamps`: `[Cam, T]`
- `absolute_timestamps`: `[Cam, T]`
- `t0_us`: `[1]`
- `fixed_delta_seconds`: `[1]`
- `clip_id`: `[1]`
- `camera_order`: `[Cam]`

Current working live setup typically assumes:

- `Cam = 4`
- `T = 4`
- `C = 3`
- `H = 320`
- `W = 576`

### Important NPZ Encoding Notes

For Phase 1 compatibility, do **not** store metadata as bare Python scalars or plain strings in the NPZ.

Use NumPy arrays so the planner-side live consumer can read them consistently:

- `t0_us`: `int64[1]`
- `fixed_delta_seconds`: `float32[1]`
- `clip_id`: string array with shape `[1]`
- `camera_order`: string array with shape `[Cam]`
- `relative_timestamps`: `float32[Cam, T]`, seconds relative to `t0`
- `absolute_timestamps`: `int64[Cam, T]`, absolute UTC timestamps for the selected frames

This matters because the current planner consumer reads:

- `t0_us` as `int(npz["t0_us"][0])`
- `fixed_delta_seconds` as `float(npz["fixed_delta_seconds"][0])`
- `clip_id` as `str(npz["clip_id"][0])`
- `camera_order` as `npz["camera_order"].tolist()`

The timestamp arrays should describe the actual selected frames, not only the nominal requested offsets.

### Recommended Dtypes

Use these unless planner team explicitly changes the contract:

- `image_frames`: `uint8`
- `camera_indices`: `int32`
- `ego_history_xyz`: `float32`
- `ego_history_rot`: `float32`
- `relative_timestamps`: `float32`
- `absolute_timestamps`: `int64`
- `t0_us`: `int64`
- `fixed_delta_seconds`: `float32`
- `clip_id`: string
- `camera_order`: list of strings

### Camera ID Mapping

Current working camera ID mapping should be treated as part of the contract:

- `0` = left
- `1` = front
- `2` = right
- `6` = front_tele

The sensor container must preserve this mapping unless planner team explicitly changes it.

## Pose / History Coordinate Definition

This is fixed for Phase 1:

- `ego_history_xyz` is in the **t0-relative ego-local frame**
- the last history step corresponds to the current reference pose at `t0`
- `ego_history_xyz[..., -1, :]` should therefore be approximately `[0, 0, 0]`
- local axis convention:
  - `+x` = forward
  - `+y` = left
  - `+z` = up

For rotation:

- `ego_history_rot` is also expressed in the **same t0-relative ego-local frame**
- the last history rotation at `t0` should therefore be approximately identity
- if INS/Xsens provides quaternion or Euler attitude, convert it to a `3x3` rotation matrix before packaging

This is the preferred contract over ENU / map-frame history for the planner-facing sample.

### Recommended Camera Order

Current working semantic order:

1. `left`
2. `front`
3. `right`
4. `front_tele`

If a camera is unavailable, do not silently reshuffle IDs. Missing-camera behavior must be explicit.

## Phase 1 Transport Contract

For first integration, the sensor container should expose:

- HTTP endpoint: `/latest`
- response payload: NPZ

Semantics:

- `/latest` returns the newest **complete valid sample**
- planner is allowed to poll
- sensor container can overwrite old samples
- this is a latest-value feed, not a queue

If no valid sample is available yet, return:

- HTTP `503 Service Unavailable`

Do **not** use `404`, `204`, or an empty body for the "no valid sample yet" case. The current planner consumer explicitly treats `503` as the retry / wait condition.

### Suggested Metadata Headers

These are optional but strongly recommended:

- `X-Sample-Sequence`
- `X-T0-US`
- `X-Clip-ID`
- `X-Sample-Valid`

## What The Sensor Container Team Needs From Planner Team

Before implementation starts, these values should be confirmed and written down:

1. final output image size for planner-facing frames
2. exact camera ID mapping
3. exact `T` frame count per camera
4. timestamp basis: UTC or monotonic
5. stale threshold for invalid samples
6. orientation source preference:
   - INS heading / quaternion
   - GNSS/INS fused yaw
   - fallback policy

If those are not confirmed, the default assumptions in this document should be used.

## High-Level System Split

Recommended split:

1. `sensor container`
   - camera ingest
   - IMU / GNSS / INS ingest
   - timestamp alignment
   - GPU downscale / image preprocessing
   - `10 Hz` planner sample building
   - publish sample to planner container

2. `planner container`
   - persistent `llm_inference`
   - VLM + FM inference
   - produce full predicted trajectory

3. `control bridge container` or process
   - resample planner trajectory to `50 Hz`
   - send UDP to control stack

## Why This Split

We do **not** want to pass raw `4K x 4 cameras` into the inference path if we can avoid it.

Background facts from the current planner repo:

- Qwen image resize is currently done in C++ on CPU via `stb_image_resize2`: [imageUtils.cpp](/root/TensorRT-Edge-LLM-v060/cpp/runtime/imageUtils.cpp:110)
- Qwen preprocess always goes through resize in the current path: [qwenViTRunner.cpp](/root/TensorRT-Edge-LLM-v060/cpp/multimodal/qwenViTRunner.cpp:1039)
- Current live path expects an NPZ sample server at `/latest`: [README.md](/root/TensorRT-Edge-LLM-v060/README.md:118)
- Current live sample schema is already defined: [README.md](/root/TensorRT-Edge-LLM-v060/README.md:182)

That means the safest architecture is:

- do camera decode + GPU resize on the sensor side
- keep planner container focused on model execution
- reuse the current live sample schema first

## Planner-Side Constraints To Respect

These matter only because they tell us what the planner side can already consume today.

### Current Planner Input Boundary

Today the working planner-side live path is:

- fetch NPZ sample from HTTP
- write PNGs / NPY files
- build request JSON
- call `llm_inference`

Background references:

- live consumer: [jetson_live_infer_alpamayo15.py](/root/TensorRT-Edge-LLM-v060/jetson_live_infer_alpamayo15.py)
- persistent mode command interface: [llm_inference.cpp](/root/TensorRT-Edge-LLM-v060/examples/llm/llm_inference.cpp:1216)

### Current Runtime Is Only Partially File-Based

Internally the runtime already uses a structured request object:

- request struct: [llmRuntimeUtils.h](/root/TensorRT-Edge-LLM-v060/cpp/runtime/llmRuntimeUtils.h:53)

However:

- FM still expects `ego_history_xyz_npy` / `ego_history_rot_npy` file paths: [alpamayoPostVlmRuntime.cpp](/root/TensorRT-Edge-LLM-v060/cpp/runtime/alpamayoPostVlmRuntime.cpp:209)

So near-term integration should assume:

- image / state sample generation can be modernized
- full in-memory planner API is possible later
- but first bring-up should reuse the current live sample schema

## Recommended Implementation Phases

Do this in phases. Do **not** jump directly to the final shared-memory / zero-copy design.

### Phase 1: Bring-Up Path

Implement a sensor container that:

- ingests real sensors
- builds samples matching the existing NPZ schema
- exposes them via HTTP `/latest`

This lets us reuse the existing planner live consumer with minimal planner-side changes.

Advantages:

- faster bring-up
- easier debugging
- can compare live samples with existing offline extracted samples
- planner side stays stable

### Phase 2: Shared-Memory Planner Feed

After Phase 1 is stable:

- replace HTTP `/latest` with shared-memory sample transport
- keep the sample schema logically the same
- send only slot IDs / metadata over a small notify channel

### Phase 3: Full In-Memory Planner Request

After the above is stable:

- remove request JSON boundary
- remove PNG image staging
- remove NPY ego-history staging
- pass image buffers + ego history directly into planner runtime

This phase requires planner runtime changes and is **not** the first target.

## Sensor Container Responsibilities

The sensor container should own the following.

### 1. Camera Ingest

Support the camera set used by the planner. Current working assumption:

- `camera_left`
- `camera_front`
- `camera_right`
- `camera_front_tele`

Recommended responsibilities:

- open streams
- decode frames
- track per-camera frame timestamps
- detect dropped / stale streams
- expose frame health metrics

### 2. GPU Resize / Image Preprocess

This is a key responsibility of the sensor container.

Required behavior:

- receive raw 4K frames
- downscale on GPU / hardware path before planner handoff
- output planner-facing frames at a smaller inference-friendly resolution

Recommended output target for first bring-up:

- match the current working planner-facing convention of:
  - per-frame image tensor shape `[C, 320, 576]`
  - equivalently `H = 320`, `W = 576`
  - `RGB uint8`
  - simple resize + bicubic

Notes:

- final exact resize policy may be updated later
- preserve aspect ratio as much as possible
- keep camera order stable

### 3. State Sensor Ingest

Ingest:

- IMU
- GNSS / INS / localization
- optional wheel speed / odometry if available

Important:

- do **not** rely on position-difference yaw as the long-term solution
- prefer INS / fused orientation if available

### 4. Timestamp Alignment

All sensor streams must be aligned to a single time base.

Use one canonical timestamp basis:

- monotonic microseconds, or
- UTC microseconds

Requirements:

- camera timestamps must be comparable to localization timestamps
- every sample must carry an exact `t0`
- frame selection must be deterministic

### 5. Ring Buffers

Maintain recent history for:

- camera frames
- IMU
- GNSS / INS

Ring buffers should retain enough history to build:

- `ego_history_xyz`
- `ego_history_rot`
- recent image frames required by the planner input

### 6. 10 Hz Sample Builder

At every planner tick:

- choose `t0`
- pick the appropriate synchronized camera frames
- build ego-history tensors
- package a planner sample

The sample should match the planner input contract defined at the top of this document.

- `image_frames`
- `camera_indices`
- `ego_history_xyz`
- `ego_history_rot`
- `relative_timestamps`
- `absolute_timestamps`
- `t0_us`
- `fixed_delta_seconds`
- `clip_id`
- `camera_order`

Do not require planner repo code to build these tensors. The sensor container should construct them independently using the agreed schema.

### 7. Publish to Planner Container

Phase 1 transport:

- HTTP `/latest`
- NPZ payload

Phase 2 transport:

- shared memory ring buffer
- notify channel for metadata

### 8. Health / Metrics

Expose:

- camera fps
- dropped frames
- stale sensor flags
- per-camera last timestamp
- timestamp skew between camera and localization
- sample build latency
- publish latency

## Recommended Transport Design

### Phase 1

Use:

- HTTP endpoint `/latest`
- exactly the same sample semantics already consumed by the planner live consumer

### Phase 2

Use:

- shared-memory ring buffer for sample payloads
- small IPC notify channel for:
  - `sample_seq`
  - `slot_id`
  - `t0_us`
  - health flags

Recommended small control channels:

- Unix domain socket
- ZeroMQ
- localhost TCP

Recommended large-data channel:

- shared memory

## What This Container Should Not Do

Do **not** put these into the sensor container at first:

- LLM / ViT / FM inference
- request JSON generation for production
- control UDP `50 Hz` publish
- final trajectory selection logic

Keep the sensor side focused on clean input generation.

## Standalone Deliverables Expected From Sensor Container Team

The sensor container team should be able to deliver the following artifacts without needing this TRT repo inside their image:

1. a sensor container image
2. a config file describing camera and localization sources
3. a `/latest` HTTP sample server
4. a sample schema validator script
5. a short README describing:
   - how to launch the sensor container
   - what `/latest` returns
   - what timestamp basis is used
   - what camera ID mapping is used
6. at least one captured NPZ sample that planner team can use for validation

## Suggested Directory / File Layout

This is a proposed structure, not a hard requirement.

```text
sensor_container/
  README.md
  config/
    sensor_container.example.yaml
  python/
    sensor_hub.py
    camera_ingest.py
    state_ingest.py
    sample_builder.py
    sample_server.py
    health_server.py
  tests/
    test_timestamp_alignment.py
    test_sample_schema.py
    test_camera_order.py
    test_stale_policy.py
```

If implemented in C++ instead, keep the same logical component split.

## Implementation Tasks

### A. Camera Ingest

- define camera source configuration
- implement source startup / reconnect behavior
- verify stable decode for all required cameras
- store latest frame timestamps and frame counters

### B. GPU Downscale

- choose downscale path
  - preferred: DeepStream / hardware pipeline
  - acceptable: CUDA or NPP path
- benchmark `4 cameras` per batch
- ensure output resolution and color layout are correct
- validate camera ordering is stable

### C. State Ingest

- parse IMU stream
- parse GNSS / INS / localization stream
- identify orientation source
- expose fused ego pose history

### D. Timestamp Synchronization

- define the canonical timestamp basis
- implement nearest-match / alignment policy
- define allowed skew thresholds
- mark samples invalid when skew exceeds threshold

### E. Sample Builder

- build `10 Hz` planner samples
- enforce exact sample schema and shapes
- populate `camera_indices` consistently
- populate `camera_order` consistently

### F. HTTP Sample Server

Phase 1 only:

- implement `/latest`
- return the most recent valid sample
- include metadata headers if helpful:
  - sequence
  - `t0`
  - clip / route ID

### G. Health Endpoints

- expose a minimal status endpoint
- include stream stale flags and sample sequence
- include last successful sample build timestamp

This is strongly recommended for Phase 1, but `/latest` remains the only mandatory data endpoint for the first bring-up.

## Test Plan

The following tests should be implemented before calling the sensor container "usable".

### 1. Camera Ingest Tests

- each configured camera opens correctly
- fps is within expected range
- frame timestamps are monotonic
- dropped-frame behavior is detected and reported

### 2. Resize / Preprocess Tests

- output size matches requested inference size
- output color channel order is correct
- batch of `4 x 4K -> target size` completes within measured budget
- resize latency is reported

### 3. Timestamp Alignment Tests

- camera and localization timestamps align correctly at `t0`
- misaligned inputs trigger invalid / stale flags
- deterministic frame selection for repeated runs

### 4. Ego History Tests

- `ego_history_xyz` shape is correct
- `ego_history_rot` shape is correct
- orientation source is documented and validated
- no silent fallback to low-quality yaw estimation unless explicitly configured

### 5. Sample Schema Tests

- generated NPZ contains all required keys
- shapes match planner expectations
- planner live consumer can parse the sample without changes
- sample can be validated without importing TRT repo code on the sensor side

### 6. End-to-End Bring-Up Tests

- sensor container publishes `/latest`
- planner container consumes it successfully
- one full inference completes end-to-end
- output trajectory is written successfully

### 7. Long-Run Stability Tests

- run for at least `30 min`
- no memory growth issues
- no camera stream stalls
- no sample sequence regressions
- stale / invalid sample counters remain visible

## Acceptance Criteria

The Phase 1 sensor container is accepted when:

1. It can continuously ingest all required sensors.
2. It can generate valid `10 Hz` planner samples.
3. The current live planner consumer can read those samples without planner code changes.
4. Camera downscale happens before planner ingest.
5. Health and stale conditions are visible.
6. The system can run for at least `30 min` without manual intervention.

## Risks / Things To Watch

### 1. Timestamp Drift

This is likely to be the first real-world failure mode.

### 2. Orientation Quality

Do not assume velocity-difference yaw is good enough for production.

### 3. Camera Order Mismatch

The planner is sensitive to consistent camera ordering.

### 4. Hidden CPU Copies

If the sensor container claims "GPU downscale" but still copies excessively through CPU memory, the benefit may be lost.

### 5. Overbuilding Too Early

Do not start with shared-memory zero-copy unless Phase 1 is already working.

## Recommended First Deliverables

The next Codex should aim to produce these in order:

1. `sensor container README`
2. explicit sample contract document
3. sample schema validator
4. `10 Hz` sample builder
5. `/latest` NPZ sample server
6. health endpoint
7. latency / health logging
8. one successful live planner run using the existing planner container

## Immediate Next Step

The best next move is:

1. implement the sensor container in `Phase 1` form
2. output the planner sample contract described in this document
3. expose `/latest`
4. verify one full end-to-end live inference path against the planner container

Once that works, the team can safely decide whether to move to shared-memory transport and a fully in-memory planner interface.
