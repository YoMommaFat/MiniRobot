"""MiniRobot camera node: the robot's camera pictures as ROS topics.

  /camera/image_raw             sensor_msgs/Image, bgr8 (nod_tool.py reads this)
  /camera/image_raw/compressed  sensor_msgs/CompressedImage, JPEG (for viewing over Wi-Fi:
                                the web UI through rosbridge, rqt_image_view)

Each topic is only produced while something subscribes to it, so the node costs almost
nothing when nobody is looking. Settings: the "camera" section of robot_config.json.

Two picture sources ("backend"):
  "rpicam"  a Raspberry Pi camera module on the ribbon cable. Runs rpicam-vid, which must
            work first: `rpicam-hello --list-cameras` has to list the camera.
  "opencv"  a USB webcam (V4L2), or a video file for testing.

Run:  source /opt/ros/jazzy/setup.bash && python3 camera_node.py
"""
import array
import collections
import json
import os
import re
import shutil
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# Remove ROS 2 UNIX timestamps from terminal output globally
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{name}] [{severity}]: {message}"

FIRST_PICTURE_S = 15.0   # rpicam-vid may take a few seconds to start the camera
STALL_S = 5.0            # no picture for this long while running: restart the camera


class JpegStream:
    """Cuts a stream of back-to-back JPEG pictures (MJPEG) into single pictures.

    It walks the JPEG structure instead of searching for the end marker, so an FF D9
    inside a header segment (an EXIF thumbnail, for example) cannot cut a picture in two.
    """

    MAX_BUFFER = 8 * 1024 * 1024
    MORE, SKIPPED = object(), object()

    def __init__(self):
        self.buf = bytearray()
        self.dropped = 0          # broken pictures thrown away
        self._reset()

    def _reset(self):
        self.started = False      # buf[0:2] is the start marker of the current picture
        self.pos = 0              # where parsing continues
        self.in_scan = False      # inside the compressed picture data after an SOS segment

    def feed(self, data):
        """Add bytes from the stream; returns the pictures they completed."""
        self.buf += data
        pictures = []
        while True:
            r = self._step()
            if r is self.MORE:
                break
            if r is not self.SKIPPED:
                pictures.append(r)
        if len(self.buf) > self.MAX_BUFFER:     # no sense in this data: start over
            self.buf.clear()
            self._reset()
            self.dropped += 1
        return pictures

    def _resync(self, start=1):
        """Throw away the broken picture at the start of buf; continue at the next one."""
        i = self.buf.find(b"\xff\xd8", start)
        del self.buf[:i if i >= 0 else max(len(self.buf) - 1, 0)]
        self._reset()
        self.dropped += 1
        return self.SKIPPED

    def _step(self):
        buf = self.buf
        if not self.started:
            i = buf.find(b"\xff\xd8")
            if i < 0:
                del buf[:max(len(buf) - 1, 0)]   # keep a last FF: it may start a marker
                return self.MORE
            del buf[:i]
            self.started, self.pos, self.in_scan = True, 2, False
        while True:
            if self.in_scan:
                i = buf.find(b"\xff", self.pos)
                if i < 0:
                    self.pos = len(buf)
                    return self.MORE
                if i + 1 >= len(buf):
                    self.pos = i
                    return self.MORE
                b = buf[i + 1]
                if b == 0x00 or 0xD0 <= b <= 0xD7:   # a stuffed FF or a restart marker
                    self.pos = i + 2
                    continue
                if b == 0xFF:                          # fill byte
                    self.pos = i + 1
                    continue
                self.in_scan, self.pos = False, i      # a real marker: back to segments
            p = self.pos
            if p + 1 >= len(buf):
                return self.MORE
            if buf[p] != 0xFF:
                return self._resync()
            m = buf[p + 1]
            if m == 0xFF:                              # fill byte before a marker
                self.pos = p + 1
                continue
            if m == 0xD9:                              # end of picture
                picture = bytes(buf[:p + 2])
                del buf[:p + 2]
                self._reset()
                return picture
            if m == 0xD8:                              # a new picture began: this one is cut
                return self._resync(p)
            if 0xD0 <= m <= 0xD7 or m == 0x01:         # markers without a length
                self.pos = p + 2
                continue
            if p + 3 >= len(buf):
                return self.MORE
            length = (buf[p + 2] << 8) | buf[p + 3]
            if length < 2:
                return self._resync()
            if p + 2 + length > len(buf):
                return self.MORE
            self.pos = p + 2 + length
            if m == 0xDA:                              # start of scan: compressed data follows
                self.in_scan = True


class RpicamSource:
    """Pictures from a Raspberry Pi camera module: rpicam-vid writing MJPEG to stdout."""

    def __init__(self, cfg, on_jpeg, log):
        self.cfg, self.on_jpeg, self.log = cfg, on_jpeg, log
        self.proc = None
        self.last_picture = 0.0
        self.stopping = threading.Event()

    def command(self, exe):
        c = self.cfg
        cmd = [exe, "-t", "0", "-n", "--codec", "mjpeg", "-q", str(c["jpeg_quality"]),
               "--width", str(c["width"]), "--height", str(c["height"]),
               "--framerate", str(c["framerate"]), "--camera", str(c["rpicam_camera"]),
               "--flush", "-o", "-"]
        if c["rotate_180"]:
            cmd += ["--rotation", "180"]
        return cmd + [str(a) for a in c["rpicam_extra_args"]]

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.stopping.set()
        proc = self.proc
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _run(self):
        wait = 3.0
        while not self.stopping.is_set():
            exe = shutil.which("rpicam-vid") or shutil.which("libcamera-vid")
            if exe is None:
                self.log.error("rpicam-vid was not found. Install rpicam-apps (Raspberry Pi "
                               "camera software), or use \"backend\": \"opencv\" for a USB camera.")
                self.stopping.wait(30)
                continue
            got_pictures = self._run_once(exe)
            if self.stopping.is_set():
                break
            wait = 3.0 if got_pictures else min(wait * 2, 30.0)
            self.log.info(f"Starting the camera again in {wait:.0f} s.")
            self.stopping.wait(wait)

    def _run_once(self, exe):
        cmd = self.command(exe)
        self.log.info("Starting: " + " ".join(cmd))
        errors = collections.deque(maxlen=15)
        try:
            self.proc = proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                                stderr=subprocess.PIPE, bufsize=0)
        except OSError as e:
            self.log.error(f"Could not start {exe}: {e}")
            return False

        def keep_errors():
            for line in proc.stderr:
                errors.append(line.decode("utf-8", "replace").rstrip())
        threading.Thread(target=keep_errors, daemon=True).start()

        started = time.time()
        self.last_picture = 0.0
        stalled = threading.Event()

        def watchdog():
            while proc.poll() is None and not self.stopping.is_set():
                quiet = time.time() - (self.last_picture or started)
                if quiet > (STALL_S if self.last_picture else FIRST_PICTURE_S):
                    stalled.set()
                    proc.kill()
                    return
                time.sleep(0.5)
        threading.Thread(target=watchdog, daemon=True).start()

        stream, count = JpegStream(), 0
        while True:
            data = proc.stdout.read(65536)
            if not data:
                break
            for jpeg in stream.feed(data):
                self.last_picture = time.time()
                count += 1
                if count == 1:
                    self.log.info(f"Camera running: {self.cfg['width']}x{self.cfg['height']}, "
                                  f"{self.cfg['framerate']} pictures/s.")
                self.on_jpeg(jpeg)
        code = proc.wait()
        if self.stopping.is_set():
            return count > 0
        time.sleep(0.2)                          # let the last error lines arrive
        reason = ("no picture for a while, restarted it" if stalled.is_set()
                  else f"rpicam-vid stopped (exit code {code})")
        self.log.error(f"Camera: {reason} after {count} pictures. Its last messages:\n  "
                       + "\n  ".join(errors or ["(none)"]))
        return count > 0


class OpenCvSource:
    """Pictures from a USB webcam (V4L2) or, for testing, from a video file."""

    def __init__(self, cfg, on_frame, log):
        self.cfg, self.on_frame, self.log = cfg, on_frame, log
        self.stopping = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.stopping.set()

    def device(self):
        """The camera as OpenCV wants it: a number for a camera, a path for a video file."""
        device = self.cfg["device"]
        if isinstance(device, int):
            return device, False
        if device.isdigit():
            return int(device), False
        if device.startswith("/dev/"):
            # /dev/video2, or a stable link like /dev/v4l/by-id/usb-...-video-index0;
            # some OpenCV builds open V4L2 cameras only by number.
            m = re.fullmatch(r"/dev/video(\d+)", os.path.realpath(device))
            return (int(m.group(1)) if m else device), False
        return device, True

    def _open(self):
        device, is_file = self.device()
        if not is_file:
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg["width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["height"])
            cap.set(cv2.CAP_PROP_FPS, self.cfg["framerate"])
            return cap, False
        return cv2.VideoCapture(device), True          # a video file: loop it

    def _run(self):
        c = self.cfg
        period = 1.0 / c["framerate"]
        size = (c["width"], c["height"])
        while not self.stopping.is_set():
            cap, is_file = self._open()
            if not cap.isOpened():
                self.log.error(f"Could not open the camera {c['device']!r}. List the cameras "
                               "with: v4l2-ctl --list-devices")
                self.stopping.wait(5)
                continue
            self.log.info(f"Camera running: {c['device']!r}")
            fails, next_t, last_sent = 0, 0.0, 0.0
            while not self.stopping.is_set():
                ok, frame = cap.read()
                if not ok:
                    if is_file:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    fails += 1
                    if fails >= 10:
                        self.log.error("The camera stopped sending pictures; opening it again.")
                        break
                    time.sleep(0.1)
                    continue
                fails = 0
                now = time.monotonic()
                if is_file:                      # a file reads instantly: keep the pace
                    time.sleep(max(next_t - now, 0))
                    next_t = max(next_t + period, time.monotonic())
                elif now - last_sent < 0.7 * period:   # a camera faster than asked: skip
                    continue
                last_sent = now
                if (frame.shape[1], frame.shape[0]) != size:
                    frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
                if c["rotate_180"]:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                self.on_frame(frame)
            cap.release()
            self.stopping.wait(2)


DEFAULTS = {"backend": "rpicam", "topic": "/camera/image_raw", "frame_id": "camera_link",
            "width": 640, "height": 480, "framerate": 10, "jpeg_quality": 80,
            "rotate_180": False, "rpicam_camera": 0, "rpicam_extra_args": [], "device": 0}


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")
        with open(config_path, "r", encoding="utf-8") as f:
            given = json.load(f).get("camera", {})
        self.cfg = cfg = {k: given.get(k, v) for k, v in DEFAULTS.items()}

        # Reliable, last 2 pictures: works for both best-effort and reliable subscribers.
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.raw_pub = self.create_publisher(Image, cfg["topic"], qos)
        self.jpeg_pub = self.create_publisher(CompressedImage, cfg["topic"] + "/compressed", qos)
        self.bad_pictures = 0

        if cfg["backend"] == "rpicam":
            self.source = RpicamSource(cfg, self.publish_jpeg, self.get_logger())
        elif cfg["backend"] == "opencv":
            self.source = OpenCvSource(cfg, self.publish_frame, self.get_logger())
        else:
            raise ValueError(f"Unknown camera backend {cfg['backend']!r}: use 'rpicam' or 'opencv'")
        self.source.start()
        self.get_logger().info(f"Camera node started ({cfg['backend']}). Publishing on "
                               f"{cfg['topic']} and {cfg['topic']}/compressed.")

    def _header(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg["frame_id"]

    def _send_jpeg(self, jpeg):
        msg = CompressedImage()
        self._header(msg)
        msg.format = "bgr8; jpeg compressed bgr8"
        msg.data = array.array("B", jpeg)
        self.jpeg_pub.publish(msg)

    def _send_raw(self, frame):
        frame = np.ascontiguousarray(frame)
        msg = Image()
        self._header(msg)
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = array.array("B", frame.tobytes())
        self.raw_pub.publish(msg)

    def publish_jpeg(self, jpeg):
        """rpicam: the picture arrives as JPEG; decode it only if someone wants it raw."""
        if self.jpeg_pub.get_subscription_count() > 0:
            self._send_jpeg(jpeg)
        if self.raw_pub.get_subscription_count() > 0:
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self.bad_pictures += 1
                if self.bad_pictures in (1, 10, 100):
                    self.get_logger().warning(f"{self.bad_pictures} pictures could not be decoded.")
                return
            self._send_raw(frame)

    def publish_frame(self, frame):
        """opencv: the picture arrives decoded (BGR); encode JPEG only if someone wants it."""
        if self.jpeg_pub.get_subscription_count() > 0:
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY,
                                                    int(self.cfg["jpeg_quality"])])
            if ok:
                self._send_jpeg(jpeg.tobytes())
        if self.raw_pub.get_subscription_count() > 0:
            self._send_raw(frame)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.source.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
