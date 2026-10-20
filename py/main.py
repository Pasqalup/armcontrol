import math
import threading
import time

import cv2
import mujoco
from pynput import keyboard

from cvPip import runPipeline
from sim import MuJoCoSim

from wrapper import (
    RobotPipeline,
    IKParameters,
    TrackingParameters,
    PipelineParameters,
    CameraParameters,
)
TARGET_W, TARGET_H = 640, 480
# Horizontal force applied to the ball per held key (Newtons).
FORCE = 140.0

# MuJoCo simulation. render=True opens the interactive viewer window
# (drag to orbit, scroll to zoom, and the sliders/actuators are live);
# offscreen_size pre-creates the offscreen renderer so the in-scene "tracking"
# camera can also be rendered for the vision pipeline without tearing down the
# GLFW session the viewer window lives in.
# threaded=False: GLFW windows must be created on the main thread on macOS.
sim = MuJoCoSim(
    "sim/models/combined.xml",
    render=True,
    threaded=False,
    offscreen_size=(TARGET_W, TARGET_H),
)

# Precompute the ball's body id so we can index its row in xfrc_applied.
_BALL_BODY_ID = mujoco.mj_name2id(
    sim.model, mujoco.mjtObj.mjOBJ_BODY, "ball"
)
if _BALL_BODY_ID < 0:
    raise RuntimeError("no body named 'ball' in the model")

_CAMERA_ID = mujoco.mj_name2id(
    sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "tracking"
)
if _CAMERA_ID < 0:
    raise RuntimeError("no camera named 'tracking' in the model")
if int(sim.model.cam_projection[_CAMERA_ID]) != 0:
    raise RuntimeError("tracking camera must use perspective projection")
_camera_rotation = sim.model.cam_mat0[_CAMERA_ID].reshape(3, 3)
_camera_fovy = math.radians(float(sim.model.cam_fovy[_CAMERA_ID]))
if not math.isfinite(_camera_fovy) or _camera_fovy <= 0.0:
    raise RuntimeError("tracking camera has invalid perspective intrinsics")
_ball_geom_id = mujoco.mj_name2id(
    sim.model, mujoco.mjtObj.mjOBJ_GEOM, "ball_geom"
)
if _ball_geom_id < 0:
    raise RuntimeError("no geometry named 'ball_geom' in the model")
_shoulder_joint_id = mujoco.mj_name2id(
    sim.model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder"
)
if _shoulder_joint_id < 0:
    raise RuntimeError("no joint named 'shoulder' in the model")
_base_body_id = mujoco.mj_name2id(
    sim.model, mujoco.mjtObj.mjOBJ_BODY, "base"
)
if _base_body_id < 0:
    raise RuntimeError("no body named 'base' in the model")
_arm_base_height = (
    float(sim.model.body_pos[_base_body_id][2])
    + float(sim.model.jnt_pos[_shoulder_joint_id][2])
)

ik = IKParameters(
    l1=0.40,
    l2=0.40,
)
tracking = TrackingParameters(
    max_distance=50.0,
    max_missed_frames=5,
    confirmation_hits=2,
    velocity_alpha=0.5,
    radius_weight=1.0,
    max_speed=1000.0,
    use_kalman=False,
)

pipeline_params = PipelineParameters(
    lookahead_time=0.5,
    true_radius=float(sim.model.geom_size[_ball_geom_id][0]),
    arm_base_height=_arm_base_height,
    camera=CameraParameters(
        position=tuple(float(value) for value in sim.model.cam_pos[_CAMERA_ID]),
        forward=tuple(float(value) for value in -_camera_rotation[:, 2]),
        up=tuple(float(value) for value in _camera_rotation[:, 1]),
        focal_length=TARGET_H / (2.0 * math.tan(_camera_fovy / 2.0)),
        principal_x=TARGET_W / 2.0,
        principal_y=TARGET_H / 2.0,
    ),
)


def start_keyboard_listener():
    """Run a background thread tracking which WASD/quit keys are held.

    Returns (pressed_set, stop_event). pressed_set is mutated by the listener
    thread; reads are the current set of held keys, so multiple keys work.
    """
    pressed = set()
    stop = threading.Event()
    letter_to_char = {
        keyboard.KeyCode.from_char('w'): 'w',
        keyboard.KeyCode.from_char('a'): 'a',
        keyboard.KeyCode.from_char('s'): 's',
        keyboard.KeyCode.from_char('d'): 'd',
        keyboard.KeyCode.from_char('q'): 'q',
    }

    def on_press(key):
        char = letter_to_char.get(key)
        if char:
            pressed.add(char)

    def on_release(key):
        char = letter_to_char.get(key)
        if char:
            pressed.discard(char)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.daemon = True
    listener.start()
    return pressed, stop


def ctrl_ball(pressed, sim=sim):
    """Read the held WASD keys (a set) and apply a horizontal force."""
    fx = 0.0
    fy = 0.0
    if 'w' in pressed:
        fy += FORCE
    if 's' in pressed:
        fy -= FORCE
    if 'a' in pressed:
        fx -= FORCE
    if 'd' in pressed:
        fx += FORCE

    # Index the ball's row in xfrc_applied, then write the x/y/z forces.
    ball_force = sim.data.xfrc_applied[_BALL_BODY_ID]
    ball_force[0] = fx
    ball_force[1] = fy
    ball_force[2] = 0.0  # no vertical push
def control(base_angle, joint1_angle, joint2_angle):
    """Set the robot arm's joint angles."""
    sim.data.ctrl[0] = base_angle
    sim.data.ctrl[1] = -joint1_angle
    sim.data.ctrl[2] = joint2_angle
with RobotPipeline(ik, tracking, pipeline_params, library_path="../build/Release/modeled_c.dll") as pipeline:
    pressed, _stop = start_keyboard_listener()

    while True:
        # Step the simulation forward by one time step.
        sim.step()
        ctrl_ball(pressed)

        # Render the scene from the tracking camera's perspective.
        rgb_frame = sim.render_frame(camera="tracking", width=TARGET_W, height=TARGET_H)
        frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        contours, image, centers = runPipeline(frame,[1])


        # Run the vision pipeline on the rendered image.
        result = pipeline.update(centers, timestamp=time.monotonic())
        if(result.error_code != 0):
            print(f"Vision pipeline error: {result.error_code}")
        else:
            control(result.theta1, result.theta2, result.theta3)
        
        

        # Print the result of the vision pipeline.
        print(result)
        cv2.imshow("tracking", image)
        time.sleep(0.01)  # slow down the loop so the simulation doesn't run too fast
        if cv2.waitKey(1) == ord('q'):
            break
