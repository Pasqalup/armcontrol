import math
import threading
import time

import cv2
import mujoco
from pynput import keyboard

from sim import MuJoCoSim
from vision import ObjectTracker, parse_centers, runPipeline


TARGET_W, TARGET_H = 640, 480
# Horizontal force applied to the ball per held key (Newtons).
FORCE = 80.0

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


# intialize object tracker
tracker = ObjectTracker(
    max_distance=90.0,
    max_missed_frames=5,
    confirmation_hits=2,
    velocity_alpha=0.4,
    use_kalman=True
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

def get_capsule_length(geom_name):
    geom_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_GEOM,
        geom_name,
    )

    if geom_id < 0:
        raise RuntimeError(f"No geometry named {geom_name!r}")

    if sim.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_CAPSULE:
        raise ValueError(f"{geom_name!r} is not a capsule")

    radius, half_length = sim.model.geom_size[geom_id][:2]
    return 2.0 * half_length

def control(base_angle, joint1_angle, joint2_angle):
    """Set the robot arm's joint angles."""
    sim.data.ctrl[0] = -base_angle
    sim.data.ctrl[1] = -joint1_angle
    sim.data.ctrl[2] = joint2_angle
def vizpoint(x, y, z):
    """Set the position of the visualization site."""
    site_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, "ball_site")
    if site_id < 0:
        raise RuntimeError("no site named 'ball_site' in the model")
    sim.data.site_xpos[site_id] = [x, y, z]
# initialize inverse kinematics solver for the arm
# get the lengths of the two arm segments from the MuJoCo model

from kinematics.ik import IK
l1len = get_capsule_length("link1_geom")
l2len = get_capsule_length("link2_geom")
ik_solver = IK(l1len, l2len)
# get offsets for the arm base position from the MuJoCo model
# get "pos" attribute of the "shoulder" joint, 3rd element it z position
#                 <joint name="shoulder" type="hinge" axis="0 1 0" pos="0 0 0.1" range="-300 0" damping="1"/>
base_offset = sim.model.jnt_pos[mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder")][2] + 0.05
print(f"Arm base offset: {base_offset}")
print(f"Arm segment lengths: l1={l1len}, l2={l2len}")

coords = (0.4, 0.3, 0.1)  # x, y, z coordinates of the target point
coords = (coords[0], coords[1], coords[2] - base_offset)  # adjust for base offset
try:
    base_angle, joint1_angle, joint2_angle = ik_solver.solve(*coords)
    print(f"IK solution for {coords}: base={math.degrees(base_angle):.1f}, "
          f"joint1={math.degrees(joint1_angle):.1f}, "
          f"joint2={math.degrees(joint2_angle):.1f}")
    control(base_angle, joint1_angle, joint2_angle)
except ValueError as e:
    print(f"IK error for {coords}: {e}")

i = 0

# Watch physical keyboard state so multiple keys (e.g. w+d) can be held at once.
# cv2.waitKey only reports one key per frame, so it can't do diagonal movement.
pressed, _stop = start_keyboard_listener()

while True:
    i += 1
    # Closing the interactive viewer window ends the run.
    if not sim.viewer_running:
        print("Viewer window closed - exiting.")
        break

    # 'q' to quit is now tracked via the listener; still drain pending
    # waitKey events so the cv2 window stays responsive to normal keys.
    cv2.waitKey(1)
    if 'q' in pressed:
        break
    ctrl_ball(pressed)

    # Advance the physics simulation. step() also syncs the viewer window, so
    # the force written above and the new pose both appear immediately.
    sim.step()

    # Render the scene from the camera inside the sim, into a BGR frame
    rgb_frame = sim.render_frame(camera="tracking", width=TARGET_W, height=TARGET_H)
    frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

    # The rendered frame is already 640x480, so no cropping is needed.
    final_frame = frame

    ## START MODIFIABLE CODE
    contours, image, centers = runPipeline(final_frame, [0])

    # parse the centers into DetectedObject instances
    detections = parse_centers(centers)
    timestamp = time.monotonic()
    tracked_objects = tracker.update(detections, timestamp)

    for obj in tracked_objects:
        if not obj.confirmed:
            continue

        # print(
        #     "ID:", obj.id,
        #     "Position:", round(obj.x, 1), round(obj.y, 1),
        #     "Velocity:", round(obj.vx, 1), round(obj.vy, 1),
        #     "Radius:", round(obj.radius, 1),
        #     "Missing frames:", obj.missed_frames
        # )
        # compute the position of the object in world coordinates (meters)
        # must scale pixels to meters, and center the origin at the center of the image
        predicted_x, predicted_y = obj.predict(0.5)
        if i % 10 == 0:  # only compute IK every 10 frames to reduce computation
            position_world = ((predicted_x - TARGET_W / 2) / 415.8, (predicted_y - TARGET_H / 2) / 415.8)
            print("World Position:", round(position_world[0], 3), round(position_world[1], 3))

            vizpoint(position_world[0], position_world[1], 0.2)
            coords = (position_world[0], position_world[1], 0.2 - base_offset)  # x, y, z coordinates of the target point
            try:
                base_angle, joint1_angle, joint2_angle = ik_solver.solve(*coords)
                print(f"IK solution for {coords}: base={math.degrees(base_angle):.1f}, "
                    f"joint1={math.degrees(joint1_angle):.1f}, "
                    f"joint2={math.degrees(joint2_angle):.1f}")
                control(base_angle, joint1_angle, joint2_angle)
            except ValueError as e:
                print(f"IK error for {coords}: {e}")
        x = int(round(obj.x))
        y = int(round(obj.y))
        radius = max(1, int(round(obj.radius)))

        cv2.circle(image, (x, y), radius, (0, 255, 0), 2)

        

        cv2.circle(image, (int(round(predicted_x)), int(round(predicted_y))), radius, (255, 0, 0), 2)

        # Object ID
        # cv2.putText(
        #     image,
        #     f"ID {obj.id}",
        #     (x + radius + 4, y - 4),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.5,
        #     (0, 255, 0),
        #     1,
        #     cv2.LINE_AA
        # )

        # Velocity arrow (scaled for visibility)
        velocity_scale = 0.2
        end_x = int(round(x + obj.vx * velocity_scale))
        end_y = int(round(y + obj.vy * velocity_scale))

        if obj.vx != 0 or obj.vy != 0:
            cv2.arrowedLine(
                image,
                (x, y),
                (end_x, end_y),
                (0, 0, 255),
                2,
                cv2.LINE_AA,
                tipLength=0.3
            )



    ## END MODIFIABLE CODE
    cv2.imshow("Frame", image)

sim.close()
cv2.destroyAllWindows()