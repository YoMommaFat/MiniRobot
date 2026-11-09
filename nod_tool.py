"""nod_towards: MiniRobot turns towards a visible object and drives a few short steps to it.

The AI only says what to approach. The robot's own camera and distance sensors decide
how far it turns and when it stops, and the tool sets the face itself (OBSERVING while it
works, then SUCCESS, SORRY or ALERT) and tells the model which tag to start its answer with.

Needs: a camera node publishing sensor_msgs/Image (camera_ros: /camera/image_raw) and
arduino_bridge.py (/cmd_vel in, the distance sensors on /arduino_sensors).
"""
import base64
import io
import json
import math
import re
import threading
import time
from typing import Annotated

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from langchain_core.tools import tool
from PIL import Image as PILImage
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# --- Settings: measure these on the robot ------------------------------------------------
CAMERA_TOPIC = "/camera/image_raw"
TURN_SPEED = 1.5        # rad/s while turning on the spot (wheels about ±0.07 m/s, PWM ±20)
TURN_SCALE = 1.0        # >1 if the robot turns less than asked (timed turns, no odometry yet)
TURN_STEP_DEG = 20.0    # first turn when the object is in the left or right third
MIN_TURN_DEG = 6.0      # smaller turns may not move the wheels at all
MAX_TURNS = 4
DRIVE_SPEED = 0.10      # m/s forward (PWM about 29)
DRIVE_SCALE = 1.0       # >1 if the robot drives shorter than asked
STEPS = 3               # the "nod": a few short steps with a pause between them
STEP_M = 0.05
STEP_PAUSE_S = 0.2      # plus the wait for a fresh distance reading
SETTLE_S = 0.3          # wait after a move, so the next picture is taken standing still
# Front distance sensors [DF_L, DF_M, DF_R] are raw ADC values (0..1023), sent every 0.5 s.
# Measure: hold a box 10 cm in front of each sensor and read `ros2 topic echo /arduino_sensors`.
DIST_CLOSE_RAW = 450    # at or past this value something is too close in front
DIST_HIGHER_IS_CLOSER = True   # true for Sharp-style analog IR sensors
DIST_MAX_AGE_S = 1.5    # older distance data counts as no data: the robot does not move
# -----------------------------------------------------------------------------------------

VALID_TAGS = ("READY", "CONFUSED", "SUCCESS", "SORRY", "CURIOSITY", "OBSERVING",
              "AFFECTION", "WINK", "ALERT")

QUESTION = ("This is the robot's camera picture. Is there a {target} in it? Answer with one "
            "word: left, middle or right if you see it (where it is in the picture), or no.")

WORDS = {"no": {"no", "not", "none", "nincs", "nem"},
         "left": {"left", "bal", "balra", "balon"},
         "right": {"right", "jobb", "jobbra", "jobbon"},
         "middle": {"middle", "center", "centre", "centered", "közép", "középen"}}


def parse_where(answer):
    """The vision model's answer as left, middle, right or no."""
    words = re.findall(r"[a-záéíóöőúüű]+", answer.lower())
    if not words or words[0] in WORDS["no"]:
        return "no"
    for w in words:
        for where in ("left", "right", "middle"):
            if w in WORDS[where]:
                return where
    return "no"


def _article(word):
    return "az" if word[:1].lower() in "aáeéiíoóöőuúüű" else "a"


def nod(target, look, turn, step, front, face):
    """The nod without ROS, so it can be tested on its own.

    look(target) -> left/middle/right/no/nocamera; turn(deg), positive = left;
    step(m) -> False if the distance guard stopped it; front() -> True too close,
    False clear, None no data; face(tag).
    """
    def done(tag, text):
        face(tag)
        return f"{text} A válaszod ezzel kezdődjön: [{tag}]"

    face("OBSERVING")
    if front() is None:                           # no distance data: do not move at all
        return done("ALERT", "Nem mozdultam: a távolságmérők nem küldenek adatot.")
    where = look(target)
    turned, size, last = 0.0, TURN_STEP_DEG, None
    for _ in range(MAX_TURNS):
        if where not in ("left", "right"):
            break
        if last and where != last:                # went past it: turn back half as much
            size = max(size / 2, MIN_TURN_DEG)
        angle = size if where == "left" else -size
        turn(angle)
        turned += angle
        last = where
        where = look(target)
        if where == "no":                         # lost it: undo the last turn, look again
            turn(-angle)
            turned -= angle
            where = look(target)
            break

    if where == "nocamera":
        return done("ALERT", "Nem mozdultam: nem jön kép a kamerából.")
    if where == "no":
        return done("SORRY", f"Nem látom magam előtt ezt: {target}. Nem mentem közelebb.")
    if where != "middle":
        return done("SORRY", f"{abs(turned):.0f} fokot fordultam, de nem sikerült magam elé "
                             f"fordítani ezt: {target}. Nem mentem közelebb.")

    moved, close = 0, False
    for i in range(STEPS):
        state = front()
        if state is None:
            return done("ALERT", "Nem indultam el: a távolságmérők nem küldenek adatot.")
        if state or not step(STEP_M):
            close = True
            break
        moved += 1
        if i < STEPS - 1:
            time.sleep(STEP_PAUSE_S)

    turn_text = ("nem kellett fordulnom" if abs(turned) < 1 else
                 f"{abs(turned):.0f} fokot fordultam {'balra' if turned > 0 else 'jobbra'}")
    art = _article(target)
    if moved == 0:
        return done("SUCCESS", f"Kész: {turn_text}. Már közvetlenül {art} {target} előtt "
                               f"vagyok, ezért nem mentem közelebb.")
    text = f"Kész: {turn_text}, és {moved} lépésben {moved * STEP_M * 100:.0f} cm-t mentem {art} {target} felé."
    if close:
        text += " Utána megálltam, mert már közel vagyok hozzá."
    return done("SUCCESS", text)


def picture_to_png_b64(msg):
    """sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8 or mono8) as a base64 PNG."""
    enc = msg.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if channels is None:
        raise ValueError(f"camera encoding {msg.encoding} is not supported")
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    a = a[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc.startswith("bgr"):
        a = a[..., [2, 1, 0]]
    elif channels == 4:
        a = a[..., :3]
    elif channels == 1:
        a = np.repeat(a, 3, axis=2)
    buf = io.BytesIO()
    PILImage.fromarray(np.ascontiguousarray(a)).save(buf, "PNG")
    return base64.b64encode(buf.getvalue()).decode()


class RobotIO:
    """The ROS side: camera pictures, distance sensors, /cmd_vel and the face."""

    def __init__(self):
        self.node = rclpy.create_node("minirobot_nod")
        self.cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.face_pub = self.node.create_publisher(String, "/cmd_face", 10)
        self.lock = threading.Lock()
        self.picture, self.picture_t = None, 0.0
        self.dist, self.dist_t = None, 0.0
        self.moved_t = 0.0          # when the last move ended
        self.node.create_subscription(Image, CAMERA_TOPIC, self._on_picture, qos_profile_sensor_data)
        self.node.create_subscription(String, "/arduino_sensors", self._on_sensors, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _on_picture(self, msg):
        with self.lock:
            self.picture, self.picture_t = msg, time.time()

    def _on_sensors(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if data.get("type") == "dist" and len(data.get("val") or []) >= 3:
            with self.lock:
                self.dist, self.dist_t = data["val"], time.time()

    def picture_after(self, t, wait_s=3.0):
        """A picture received after time t, as base64 PNG; None if none comes."""
        end = time.time() + wait_s
        while time.time() < end:
            with self.lock:
                if self.picture is not None and self.picture_t > t:
                    return picture_to_png_b64(self.picture)
            time.sleep(0.05)
        return None

    def front(self, fresh=True):
        """True: something too close in front; False: clear; None: no data.

        fresh: wait (up to 1 s) for a reading taken after the last move, because the
        sensors only report every 0.5 s and an older reading is from before the step.
        """
        end = time.time() + (1.0 if fresh else 0.0)
        while True:
            with self.lock:
                usable = (self.dist is not None and time.time() - self.dist_t <= DIST_MAX_AGE_S
                          and (not fresh or self.dist_t > self.moved_t))
                values = self.dist[:3] if usable else None
            if values is not None:
                break
            if time.time() >= end:
                return None
            time.sleep(0.05)
        if DIST_HIGHER_IS_CLOSER:
            return max(values) >= DIST_CLOSE_RAW
        return min(values) <= DIST_CLOSE_RAW

    def _drive(self, linear, angular, seconds, guard=False):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        end = time.time() + seconds
        try:
            while time.time() < end:
                if guard and self.front(fresh=False) is not False:
                    return False
                self.cmd.publish(msg)
                time.sleep(0.05)
            return True
        finally:
            self.stop()
            self.moved_t = time.time()

    def turn(self, deg):
        seconds = math.radians(abs(deg)) / TURN_SPEED * TURN_SCALE
        self._drive(0.0, math.copysign(TURN_SPEED, deg), max(seconds, 0.05))

    def step(self, metres):
        return self._drive(DRIVE_SPEED, 0.0, metres / DRIVE_SPEED * DRIVE_SCALE, guard=True)

    def stop(self):
        for _ in range(3):        # arduino_bridge has no timeout: make sure a stop arrives
            self.cmd.publish(Twist())
            time.sleep(0.02)

    def face(self, tag):
        msg = String()
        msg.data = tag
        self.face_pub.publish(msg)


def make_nod_tool(vision_llm, robot=None):
    """The LangChain tool for the agent. vision_llm: a chat model that can see pictures."""
    from rai.messages import HumanMultimodalMessage
    robot = robot or RobotIO()

    def look(target):
        picture = robot.picture_after(time.time() + SETTLE_S)
        if picture is None:
            return "nocamera"
        answer = vision_llm.invoke([HumanMultimodalMessage(
            content=QUESTION.format(target=target), images=[picture])]).content
        if isinstance(answer, list):               # Gemini sometimes answers in parts
            answer = " ".join(p.get("text", "") for p in answer if isinstance(p, dict))
        return parse_where(str(answer))

    @tool
    def nod_towards(target: Annotated[str, "the object's short English name, e.g. 'red ball'"]) -> str:
        """Turn towards an object the robot can see and drive a few short steps (about 15 cm)
        to it. The result says which state code the answer must start with."""
        try:
            return nod(target, look, robot.turn, robot.step, robot.front, robot.face)
        except Exception as e:     # noqa: BLE001 - the robot must stop and say what happened
            robot.face("ALERT")
            return f"Hiba történt, megálltam: {e}. A válaszod ezzel kezdődjön: [ALERT]"
        finally:
            robot.stop()

    return nod_towards
