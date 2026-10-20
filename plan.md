# Plan: Perspective off-axis camera + radius Kalman filtering

## Context
- Camera currently orthographic top-down (`combined.xml`), pixel->world via `(pixel-center)/pixelToMeterRatio` (only valid for ortho top-down).
- User correction: camera position & facing are CONSTANT — no filtering. Only the measured pixel radius gets Kalman filtering. True radius is a runtime-modifiable pipeline parameter.
- Decisions: camera = position + forward + up vectors; remove pixelToMeterRatio; radius filter lives inside the tracker; change sim camera to perspective off-axis.

## Math (perspective unprojection)
- Camera basis from forward F, up U: F=norm(F), R=norm(cross(U,F)), U'=cross(F,R).
- Ray dir (world) = norm((u-cx)*R + (v-cy)*U' + f*F).
- Depth d = f * R_true / r_px_filtered  (small-angle sphere projection).
- World pos = C + d * dir. Solve IK on (x,y,z).

## Files & changes
### C++ core
- `src/object_tracking.hpp/.cpp`: add `ScalarKalmanFilter` (1D: state, variance, meas/process noise). Add radius filter member to `TrackedObject`; use in `update()` (both base and KalmanTrackedObject) replacing EMA `radius = 0.7r+0.3r`.
- `src/pipeline.hpp/.cpp`: add `CameraParameters{position[3],forward[3],up[3],focalLength,principalX,principalY}`. `PipelineParameters{lookaheadTime,trueRadius,camera}` (drop pixelToMeterRatio). Add `setTrueRadius(double)`. In `update()`: run tracker on pixel detections, take first confirmed track, predict pixel pos at lookahead, unproject using filtered `track->radius` + trueRadius + camera, solve IK. Guard r_px<=0.
### C interop
- `src/main.hpp`: add `CCameraParameters`; update `CPipelineParameters` (lookaheadTime, trueRadius, camera); add `pipeline_set_true_radius(handle, radius)`.
- `src/main.cpp`: update handle construction; implement setter.
### Python
- `py/wrapper.py`: add `_CCameraParameters` ctypes struct + `CameraParameters` dataclass; update `_CPipelineParameters`/`PipelineParameters`; add `set_true_radius()`.
- `py/main.py`: build camera params (read from model), pass to pipeline.
- `py/vision/object_tracking.py`: add scalar radius Kalman filter to TrackedObject/KalmanTrackedObject.
- `py/main_simulated.py`: read cam_pos/cam_forward/cam_up from model, compute focal length from fovy+height, add unproject helper, use filtered radius for depth, replace hardcoded 415.8.
- `py/sim/models/combined.xml`: change camera to perspective off-axis, e.g. `pos="0.6 0.4 1.2" targetpos="0 0 0" fovy="60"`.

## Verification
1. `cmake --build build` (Release) succeeds.
2. Run `py/main.py` — ball tracked, IK drives arm, no NaN/errors.
3. Run `py/main_simulated.py` — world position matches ball's true sim position.
4. Call `set_true_radius` at runtime and confirm depth estimate changes.
