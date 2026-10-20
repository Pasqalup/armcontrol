"""Python ctypes wrapper for the modeled-c native pipeline.

Build the C++ shared library first, then use:

    from modeled_c import (
        RobotPipeline, IKParameters, TrackingParameters,
        PipelineParameters, CameraParameters,
    )

    with RobotPipeline(
        IKParameters(l1=0.3, l2=0.25),
        TrackingParameters(
            max_distance=50.0,
            max_missed_frames=5,
            confirmation_hits=2,
            velocity_alpha=0.5,
            radius_weight=1.0,
            max_speed=1000.0,
            use_kalman=False,
        ),
        PipelineParameters(
            lookahead_time=0.1,
            true_radius=0.0762,
            arm_base_height=0.15,
            camera=CameraParameters(
                position=(0.0, 0.0, 1.0),
                forward=(0.0, 0.0, -1.0),
                up=(0.0, 1.0, 0.0),
                focal_length=415.8,
                principal_x=320.0,
                principal_y=240.0,
            ),
        ),
    ) as pipeline:
        result = pipeline.update([320.0, 240.0, 15.0], timestamp=1.0)
        print(result)

Detection input is a flat sequence of x, y, radius triples.
"""

from __future__ import annotations

import ctypes as ct
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence


# Match these structures exactly to the C-compatible definitions in main.hpp.
class _CIKParameters(ct.Structure):
    _fields_ = [
        ("l1", ct.c_double),
        ("l2", ct.c_double),
    ]


class _CTrackingParameters(ct.Structure):
    _fields_ = [
        ("maxDistance", ct.c_double),
        ("maxMissedFrames", ct.c_int32),
        ("confirmationHits", ct.c_int32),
        ("velocityAlpha", ct.c_double),
        ("radiusWeight", ct.c_double),
        ("maxSpeed", ct.c_double),
        # main.hpp uses int32_t for the public C ABI boolean.
        ("useKalman", ct.c_int32),
    ]


class _CCameraParameters(ct.Structure):
    _fields_ = [
        ("position", ct.c_double * 3),
        ("forward", ct.c_double * 3),
        ("up", ct.c_double * 3),
        ("focalLength", ct.c_double),
        ("principalX", ct.c_double),
        ("principalY", ct.c_double),
    ]


class _CPipelineParameters(ct.Structure):
    _fields_ = [
        ("lookaheadTime", ct.c_double),
        ("trueRadius", ct.c_double),
        ("armBaseHeight", ct.c_double),
        ("camera", _CCameraParameters),
    ]


class _CIKResult(ct.Structure):
    _fields_ = [
        ("errorCode", ct.c_int32),
        ("theta1", ct.c_double),
        ("theta2", ct.c_double),
        ("theta3", ct.c_double),
    ]


@dataclass(frozen=True)
class IKParameters:
    l1: float
    l2: float


@dataclass(frozen=True)
class TrackingParameters:
    max_distance: float
    max_missed_frames: int
    confirmation_hits: int
    velocity_alpha: float
    radius_weight: float
    max_speed: float
    use_kalman: bool = False


@dataclass(frozen=True)
class CameraParameters:
    position: tuple[float, float, float]
    forward: tuple[float, float, float]
    up: tuple[float, float, float]
    focal_length: float
    principal_x: float
    principal_y: float


@dataclass(frozen=True)
class PipelineParameters:
    lookahead_time: float
    true_radius: float
    arm_base_height: float
    camera: CameraParameters


@dataclass(frozen=True)
class IKOutput:
    # This is the value written from IKResult.success in the current C++ wrapper.
    # Rename to status/error_code if that field represents an error code instead.
    error_code: int
    theta1: float
    theta2: float
    theta3: float


def _default_library_path() -> Path:
    """Find the shared library beside this Python file, unless overridden."""
    override = os.environ.get("MODELED_C_LIBRARY")
    if override:
        return Path(override).expanduser().resolve()

    if sys.platform == "darwin":
        filename = "libmodeled_c.dylib"
    elif os.name == "nt":
        filename = "modeled_c.dll"
    else:
        filename = "libmodeled_c.so"

    return Path(__file__).resolve().parent / filename


class RobotPipeline:
    """Owns one native PipelineHandle and exposes its update() function."""

    def __init__(
        self,
        ik: IKParameters,
        tracking: TrackingParameters,
        pipeline: PipelineParameters,
        library_path: Optional[os.PathLike | str] = None,
    ) -> None:
        path = Path(library_path).expanduser().resolve() if library_path else _default_library_path()
        if not path.is_file():
            raise FileNotFoundError(
                f"Native library not found: {path}\n"
                "Build the CMake shared library or pass library_path= explicitly. "
                "You can also set MODELED_C_LIBRARY."
            )

        self._lib = ct.CDLL(str(path))
        self._configure_functions()

        c_ik = _CIKParameters(ik.l1, ik.l2)
        c_tracking = _CTrackingParameters(
            tracking.max_distance,
            tracking.max_missed_frames,
            tracking.confirmation_hits,
            tracking.velocity_alpha,
            tracking.radius_weight,
            tracking.max_speed,
            int(tracking.use_kalman),
        )
        c_camera = _CCameraParameters(
            pipeline.camera.position,
            pipeline.camera.forward,
            pipeline.camera.up,
            pipeline.camera.focal_length,
            pipeline.camera.principal_x,
            pipeline.camera.principal_y,
        )
        c_pipeline = _CPipelineParameters(
            pipeline.lookahead_time,
            pipeline.true_radius,
            pipeline.arm_base_height,
            c_camera,
        )

        self._handle = self._lib.pipeline_create(c_ik, c_tracking, c_pipeline)
        if not self._handle:
            raise RuntimeError("pipeline_create() failed and returned a null handle")

    def _configure_functions(self) -> None:
        self._lib.pipeline_create.argtypes = [
            _CIKParameters,
            _CTrackingParameters,
            _CPipelineParameters,
        ]
        self._lib.pipeline_create.restype = ct.c_void_p

        self._lib.pipeline_update.argtypes = [
            ct.c_void_p,
            ct.POINTER(ct.c_double),
            ct.c_int32,
            ct.c_double,
            ct.POINTER(_CIKResult),
        ]
        self._lib.pipeline_update.restype = ct.c_int32

        self._lib.pipeline_destroy.argtypes = [ct.c_void_p]
        self._lib.pipeline_destroy.restype = None

        self._lib.pipeline_set_true_radius.argtypes = [ct.c_void_p, ct.c_double]
        self._lib.pipeline_set_true_radius.restype = ct.c_int32

    def update(self, detections: Sequence[float] | Iterable[float], timestamp: float) -> IKOutput:
        """Process flat x,y,radius triples and return the native IK output."""
        if not self._handle:
            raise RuntimeError("Pipeline has been closed")

        values = [float(value) for value in detections]
        if len(values) % 3 != 0:
            raise ValueError(
                f"detections must contain x,y,radius triples; got {len(values)} values"
            )

        if values:
            native_detections = (ct.c_double * len(values))(*values)
            detections_ptr = ct.cast(native_detections, ct.POINTER(ct.c_double))
        else:
            native_detections = None
            detections_ptr = None

        output = _CIKResult()
        wrapper_status = self._lib.pipeline_update(
            self._handle,
            detections_ptr,
            len(values),
            float(timestamp),
            ct.byref(output),
        )

        if wrapper_status != 0:
            raise RuntimeError(f"pipeline_update() wrapper failed with status {wrapper_status}")

        return IKOutput(
            error_code=int(output.errorCode),
            theta1=float(output.theta1),
            theta2=float(output.theta2),
            theta3=float(output.theta3),
        )

    def set_true_radius(self, radius: float) -> None:
        """Update the physical object radius used for perspective depth."""
        if not self._handle:
            raise RuntimeError("Pipeline has been closed")
        status = self._lib.pipeline_set_true_radius(self._handle, float(radius))
        if status != 0:
            raise ValueError(
                f"pipeline_set_true_radius() failed with status {status}"
            )

    def close(self) -> None:
        """Destroy the native handle. Safe to call more than once."""
        if self._handle:
            self._lib.pipeline_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> "RobotPipeline":
        if not self._handle:
            raise RuntimeError("Pipeline has been closed")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def __del__(self) -> None:
        # Best-effort cleanup if the caller forgets close()/the context manager.
        try:
            self.close()
        except Exception:
            pass