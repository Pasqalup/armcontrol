"""MuJoCo simulation wrapper: model load, step, and rendering.

Two rendering paths are supported, and they can run at the same time:

* **Viewer** (``render=True``): a passive ``mujoco.viewer`` window. A passive
  viewer does not redraw by itself, so ``step()`` (or an explicit ``sync()``)
  is what keeps the window current.
* **Offscreen** (``render_frame()``): ``mujoco.Renderer`` RGB array.

To use both, pass ``offscreen_size=(width, height)``. That creates the
offscreen renderer *before* the viewer window, which matters because closing a
``Renderer`` terminates the shared GLFW session process-wide, so a later
close/recreate would take the viewer's window down with it. Passing the same
size you later use for ``render_frame()`` keeps the renderer on its reuse path
and never hits that close/recreate at all.

On macOS GLFW windows must live on the main thread, so ``threaded`` is
downgraded to ``False`` there with a warning rather than being honoured.
"""

from __future__ import annotations

import atexit
import logging
import sys
import threading
from pathlib import Path

import mujoco

LOGGER = logging.getLogger(__name__)

try:
    import mujoco.viewer

    _HAS_VIEWER = True
except (ImportError, AttributeError):
    # Older MuJoCo builds require ``mujoco.viewer.launch_passive`` via a
    # separate install; keep the wrapper importable regardless.
    _HAS_VIEWER = False


class MuJoCoSim:
    """Thin wrapper around a MuJoCo model with an optional viewer."""

    def __init__(self, model_path, render=True, threaded=True, offscreen_size=None):
        """``offscreen_size`` is an optional ``(width, height)`` pair; supplying it
        pre-creates the offscreen renderer so the viewer and ``render_frame()``
        can be used together (see the module docstring)."""
        self.model_path = Path(model_path)

        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)

        self.render = render
        self.viewer = None
        self._viewer_thread = None
        self._renderer = None
        self._render_width = None
        self._render_height = None

        if offscreen_size is not None:
            self._ensure_renderer(*offscreen_size)

        if render:
            if not _HAS_VIEWER:
                raise RuntimeError(
                    "mujoco.viewer is unavailable; install 'mujoco' >= 3.2 "
                    "or the separate viewer package"
                )
            if threaded and sys.platform == "darwin":
                LOGGER.warning(
                    "threaded=True is unsupported on macOS (GLFW windows must be "
                    "created on the main thread); launching the viewer inline."
                )
                threaded = False
            if threaded:
                self._start_viewer_thread()
            else:
                self._launch_viewer()

        atexit.register(self.close)

    # ------------------------------------------------------------------
    # Viewer lifecycle
    # ------------------------------------------------------------------

    def _launch_viewer(self):
        if self._renderer is not None:
            # The offscreen renderer was created first, so its GL context is
            # already live; the viewer joins it instead of racing for it.
            LOGGER.debug(
                "Offscreen renderer is active; opening the viewer window on the "
                "existing GL context."
            )
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data
        )

    def _start_viewer_thread(self):
        self._viewer_thread = threading.Thread(
            target=self._launch_viewer,
            name="mujoco-viewer",
            daemon=True,
        )
        self._viewer_thread.start()

    def close(self):
        """Close the viewer and renderer if running."""
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None

        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None

    # ------------------------------------------------------------------
    # Simulation control
    # ------------------------------------------------------------------

    def step(self, n=1):
        """Advance the simulation ``n`` steps (default 1) and refresh the viewer."""
        for _ in range(max(1, int(n))):
            mujoco.mj_step(self.model, self.data)
        self.sync()

    def sync(self):
        """Push the current ``data`` into the viewer window.

        A passive viewer only redraws when asked, so state changes made outside
        ``step()`` - writing ``data.ctrl`` or ``data.xfrc_applied`` - need a
        ``sync()`` before they show up on screen.
        """
        if self.viewer is None or not self.viewer.is_running():
            return
        try:
            self.viewer.sync()
        except Exception:
            LOGGER.debug("viewer.sync() failed", exc_info=True)

    @property
    def viewer_running(self):
        """True while the viewer window is open; False when headless/closed."""
        return self.viewer is not None and self.viewer.is_running()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.close()

    # ------------------------------------------------------------------
    # Offscreen camera rendering
    # ------------------------------------------------------------------

    def render_frame(self, camera=0, width=640, height=480):
        """Render one offscreen frame from a model camera.

        ``camera`` may be a camera id or a camera name from the MJCF.
        Returns a uint8 array shaped ``(height, width, 3)`` in RGB order;
        convert with ``cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)`` before
        passing it to the vision pipeline.
        """
        if isinstance(camera, str):
            camera_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera
            )
            if camera_id < 0:
                raise ValueError(f"camera not found: {camera!r}")
        else:
            camera_id = camera

        if self._renderer is not None and self.viewer_running:
            LOGGER.debug(
                "Viewer window is open; rendering offscreen on the same GL context."
            )

        # Size changes recreate the renderer, which terminates the shared GLFW
        # session; pass offscreen_size to the constructor to avoid that while a
        # viewer is open.
        self._ensure_renderer(width, height)

        self._renderer.update_scene(self.data, camera_id)
        return self._renderer.render()

    def _ensure_renderer(self, width, height):
        """Create or resize the offscreen renderer, reusing it when possible.

        Some MuJoCo versions have no ``Renderer.update()``, so a size change
        means close + recreate. Recreating while the viewer is open is what
        breaks the GLFW session, hence the warning.
        """
        if (
            self._renderer is not None
            and (self._render_width, self._render_height) == (width, height)
        ):
            return

        if self._renderer is not None:
            if self.viewer_running:
                LOGGER.warning(
                    "Resizing the offscreen renderer while the viewer is open "
                    "closes the shared GLFW session and may take the viewer "
                    "window down; fix the size via offscreen_size=(w, h)."
                )
            try:
                self._renderer.close()
            except Exception:
                LOGGER.debug("renderer.close() failed", exc_info=True)
            self._renderer = None

        self._renderer = mujoco.Renderer(self.model, height, width)
        self._render_width, self._render_height = width, height

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass