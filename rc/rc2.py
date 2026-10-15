#!/usr/bin/env python3
"""
RC Controller - manual drive console for the Raspberry Pi RC car, with live camera view,
live sensor telemetry (wheel encoders + ultrasonic), YOLO object detection, "go to object"
visual-feedback driving, photo capture and recording.

Windows, Python 3.8+.
Drive controls use only the standard library (Tkinter). Optional packages:

    pip install av pillow            # live video, photos, recording
    pip install ultralytics          # object detection (pulls in numpy / torch)

Run:  python rc_controller.py [--model path\\to\\best_self2.pt]

Control protocol (TCP, port 6000), one text command per line:
    S                    immediate stop
    D <throttle> <steer> proportional drive, both -100..100 (steer < 0 = left)
The car answers "OK" to every line. If the connection drops, the server stops the motors.

Telemetry protocol (TCP, port 6001), server -> client only, one JSON object per line:
    {"type":"hello","proto":1,"ppr":5,"enc_ms":250,"us_ms":200}
    {"type":"enc","seq":42,"t_ms":123456,
     "left":{"pulses":120,"rev":24.0,"rpm":61.5}, "right":{"pulses":118,"rev":23.6,"rpm":60.2}}
    {"type":"us","seq":17,"t_ms":123500,"cm":42.3}          ("cm" is null = no valid echo)
Unknown message types / fields are ignored, so the protocol can grow.

Camera (TCP, port 5000): rpicam-vid ... --listen -o tcp://0.0.0.0:5000  (raw H.264,
decoded here with PyAV, no ffmpeg process needed).

Driving: HOLD a control to drive, RELEASE to stop. STOP is a single press.
Keys: W/A/S/D or arrows, Q/E = sharp turns, Space = STOP, P = photo, R = record on/off,
G = go to selected object / stop, F11 = full screen.

Go-to-object (visual feedback loop), only starts while the car is stopped:
  1. YOLO finds the selected object, its horizontal centre is compared with the image centre
  2. clearly left  -> short fixed steer-left pulse, clearly right -> steer-right pulse,
     near the centre -> short forward pulse
  3. stop, wait for the picture to settle, take the NEWEST frame, detect again, repeat
  ... until the object fills enough of the frame (arrived), is lost, or you press anything.
"""

import ctypes
import itertools
import json
import math
import os
import queue
import socket
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from collections import deque
from fractions import Fraction

try:
    import av
    from PIL import Image, ImageDraw, ImageTk
    _RS = getattr(Image, "Resampling", Image)
    RESAMPLE, RESAMPLE_HQ = _RS.BILINEAR, _RS.LANCZOS
    try:
        av.logging.set_level(av.logging.PANIC)       # keep the console quiet on damaged frames
    except Exception:
        pass
    VIDEO_OK = True
except Exception:                                    # missing packages: drive-only mode
    VIDEO_OK = False

# --------------------------------------------------------------------------- #
#  Protocol settings
# --------------------------------------------------------------------------- #
PORT = 6000
TERMINATOR = "\n"          # change to "\r\n" if your server expects CRLF
CONNECT_TIMEOUT = 4.0      # seconds
ACK_WARN_AFTER = 2.0       # warn if the car hasn't replied "OK" within this long
CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".rc_controller.json")
OUT_DIR = os.path.join(os.path.expanduser("~"), "RC_Controller")   # photos + videos

CAM_PORT = 5000            # rpicam-vid --listen port (same Pi as the control server)
CAM_CONNECT_TIMEOUT = 4.0
CAM_READ_TIMEOUT = 4.0     # no data for this long = stream considered dead
CAM_RETRY_MS = 2000        # how often to retry while the car link is up
CAM_ROTATE = 0             # 0, 90, 180 or 270 if the camera is mounted rotated

TELE_PORT = 6001           # sensor telemetry (JSON lines) from the same Pi
TELE_CONNECT_TIMEOUT = 4.0
TELE_READ_TIMEOUT = 3.0    # the car sends ~9 messages/s; silence this long = link is dead
TELE_RETRY_MS = 2000
TELE_STALE_S = 1.5         # a reading older than this is shown as "no data"
DIST_NEAR_CM = 30.0        # distance colours: red below this ...
DIST_FAR_CM = 80.0         # ... amber below this, green above

# --------------------------------------------------------------------------- #
#  Vision / follow settings  (tune these for your car)
# --------------------------------------------------------------------------- #
MODEL_PATH = "best_self2.pt"   # or set env RC_MODEL, or pass --model
CONFIDENCE = 0.5
IMG_SIZE = 320                 # YOLO inference size

AUTO_THROTTLE = 90             # power (0-100) for forward pulses
AUTO_TURN_THROTTLE = 100       # power (0-100) for turn pulses (skid steer needs strong power)
AUTO_TURN_STEER = 100          # 100 = spin in place at full power on both sides
TURN_PULSE_S = 0.15            # length of a fixed steer pulse (shorter since power is high)
FWD_PULSE_S = 0.35             # length of a fixed forward pulse
SETTLE_S = 0.30                # wait after each pulse so the picture is not blurred / stale
DEADZONE = 0.12                # |error| below this (fraction of half image width) = "centred"
ARRIVE_FRAC = 0.90             # stop when the object covers this fraction of the frame
EDGE_MARGIN = 0.06             # stop when the object's box gets this close (fraction of the
                               # frame) to the top/bottom edge, i.e. just before it leaves view
FOLLOW_MAX_MISSES = 8          # consecutive fresh frames without the target before giving up
FOLLOW_TIMEOUT_S = 120.0       # absolute safety limit for one run
FOLLOW_STALL_S = 4.0           # no fresh detection for this long -> stop
MATCH_MAX = 0.40               # max jump (fraction of the image) when several same-class objects

REC_FPS = 30

# --------------------------------------------------------------------------- #
#  Look and feel
# --------------------------------------------------------------------------- #
BG = "#0D1017"
SURFACE = "#151A24"
SURFACE_2 = "#1C2331"
BORDER = "#283044"
TEXT = "#E9EDF5"
MUTED = "#7E89A0"
VIDEO_BG = "#05070B"

BLUE = "#6C9CFF"     # forward / reverse
TEAL = "#2FD3BE"     # normal turns
AMBER = "#FFB02E"    # sharp turns
RED = "#FF4D5E"      # stop / errors
GREEN = "#38D996"    # connected / auto

FONT = "Segoe UI"
FONT_SB = "Segoe UI Semibold"

# code: (label, colour, wire command)
COMMANDS = {
    "F": ("Forward", BLUE, "D 100 0"),
    "B": ("Reverse", BLUE, "D -100 0"),
    "L": ("Left", TEAL, "D 100 -50"),
    "R": ("Right", TEAL, "D 100 50"),
    "SL": ("Sharp left", AMBER, "D 100 -100"),
    "SR": ("Sharp right", AMBER, "D 100 100"),
    "S": ("STOP", RED, "S"),
    # automatic pulses used by go-to-object
    "AF": ("Auto: forward", GREEN, "D %d 0" % AUTO_THROTTLE),
    "AL": ("Auto: turn left", GREEN, "D %d -%d" % (AUTO_TURN_THROTTLE, AUTO_TURN_STEER)),
    "AR": ("Auto: turn right", GREEN, "D %d %d" % (AUTO_TURN_THROTTLE, AUTO_TURN_STEER)),
}
KEYMAP = {
    "w": "F", "up": "F", "s": "B", "down": "B",
    "a": "L", "left": "L", "d": "R", "right": "R",
    "q": "SL", "e": "SR", "space": "S", "escape": "S",
}
HOTKEYS = {"p": "photo", "r": "record", "g": "follow"}
KEYCAP = {"F": "W", "B": "S", "L": "A", "R": "D", "SL": "Q", "SR": "E", "S": "Space"}

# On Windows, Tk sends no KeyRelease between auto-repeat presses. Elsewhere it does.
KEY_DEBOUNCE_MS = 0 if sys.platform == "win32" else 30

S = 1.0  # UI scale (DPI); set once the Tk root exists


def px(n):
    return int(round(n * S))


# --------------------------------------------------------------------------- #
#  Small helpers
# --------------------------------------------------------------------------- #
def _rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def mix(a, b, t):
    t = max(0.0, min(1.0, t))
    ra, rb = _rgb(a), _rgb(b)
    return "#%02x%02x%02x" % tuple(round(x + (y - x) * t) for x, y in zip(ra, rb))


def round_rect(cv, x1, y1, x2, y2, r, **kw):
    r = max(1, min(r, (x2 - x1) / 2, (y2 - y1) / 2))
    pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
           x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
           x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
    return cv.create_polygon(pts, smooth=True, splinesteps=14, **kw)


def chevron(cv, cx, cy, s, direction, color, width, offset=0.0):
    th = math.radians({"right": 0, "down": 90, "left": 180, "up": 270}[direction])
    c, sn = math.cos(th), math.sin(th)
    ox, oy = offset * c, offset * sn
    pts = []
    for x, y in ((-0.45 * s, -s), (0.45 * s, 0), (-0.45 * s, s)):
        pts += [cx + ox + x * c - y * sn, cy + oy + x * sn + y * c]
    cv.create_line(*pts, fill=color, width=width, capstyle="round", joinstyle="round")


def distance_color(cm):
    if cm < DIST_NEAR_CM:
        return RED
    if cm < DIST_FAR_CM:
        return AMBER
    return GREEN


def describe(exc):
    if isinstance(exc, socket.gaierror):
        return "That address can't be found. Check the IP address."
    if isinstance(exc, (socket.timeout, TimeoutError)):
        return "No answer. Check the IP address and that you're on the car's network."
    if isinstance(exc, ConnectionRefusedError):
        return "Connection refused. Is the control server running on the Pi?"
    if isinstance(exc, (ConnectionResetError, ConnectionAbortedError, BrokenPipeError)):
        return "The car closed the connection."
    return getattr(exc, "strerror", None) or str(exc)


def describe_cam(exc):
    if isinstance(exc, ConnectionRefusedError):
        return "Nothing is listening on port %d. Start rpicam-vid on the Pi." % CAM_PORT
    if isinstance(exc, (socket.timeout, TimeoutError)):
        return "The camera didn't answer on port %d." % CAM_PORT
    if isinstance(exc, (ConnectionResetError, ConnectionAbortedError, BrokenPipeError)):
        return "The camera stream was closed."
    return describe(exc)


def describe_tele(exc):
    if isinstance(exc, ConnectionRefusedError):
        return "Nothing is listening on port %d. Is the server updated?" % TELE_PORT
    if isinstance(exc, (socket.timeout, TimeoutError)):
        return "The car didn't answer on port %d." % TELE_PORT
    if isinstance(exc, (ConnectionResetError, ConnectionAbortedError, BrokenPipeError)):
        return "The sensor stream was closed."
    return describe(exc)


def _quiet_close(sock):
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        sock.close()
    except OSError:
        pass


def load_host():
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return str(json.load(f).get("host", ""))
    except (OSError, ValueError):
        return ""


def save_host(host):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump({"host": host}, f)
    except OSError:
        pass


def out_path(prefix, ext):
    os.makedirs(OUT_DIR, exist_ok=True)
    base = os.path.join(OUT_DIR, time.strftime(prefix + "_%Y%m%d_%H%M%S"))
    p, n = base + ext, 1
    while os.path.exists(p):
        n += 1
        p = "%s_%d%s" % (base, n, ext)
    return p


def resolve_model():
    p = os.environ.get("RC_MODEL") or MODEL_PATH
    if "--model" in sys.argv[:-1]:
        p = sys.argv[sys.argv.index("--model") + 1]
    if os.path.isabs(p) or os.path.exists(p):
        return p
    alt = os.path.join(os.path.dirname(os.path.abspath(__file__)), p)
    return alt if os.path.exists(alt) else p


def match_target(dets, t, w, h):
    """Index of the detection that is the selected target (same class, nearest to where
    it was last seen), or None. `t` holds label + last centre as fractions of the image."""
    cands = [(i, d) for i, d in enumerate(dets) if d["label"] == t["label"]]
    if not cands:
        return None

    def dist(d):
        x1, y1, x2, y2 = d["box"]
        return math.hypot((x1 + x2) / 2 / w - t["cx"], (y1 + y2) / 2 / h - t["cy"])

    i, d = min(cands, key=lambda c: dist(c[1]))
    if len(cands) > 1 and dist(d) > MATCH_MAX:
        return None
    return i


# --------------------------------------------------------------------------- #
#  Networking: one Session per connection attempt
# --------------------------------------------------------------------------- #
class Session:
    """A single TCP connection. Sends happen on the caller's thread (lowest latency, and a
    failed send is known immediately); a reader thread collects the car's replies.
    Events for the UI are posted to `events` as tuples starting with (kind, session_id, ...)."""

    _ids = itertools.count(1)

    def __init__(self, host, port, events):
        self.id = next(Session._ids)
        self.host, self.port, self.events = host, port, events
        self.sock = None
        self.alive = False
        self.cancelled = False
        self.reason = "Connection closed"

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            sock = socket.create_connection((self.host, self.port), timeout=CONNECT_TIMEOUT)
        except OSError as exc:
            if not self.cancelled:
                self.events.put(("failed", self.id, describe(exc)))
            return
        if self.cancelled:
            _quiet_close(sock)
            return
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if hasattr(socket, "SIO_KEEPALIVE_VALS"):        # Windows: notice a dead link quickly
                sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 2000, 1000))
            else:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except (OSError, ValueError):
            pass
        sock.settimeout(1.0)
        self.sock = sock
        self.alive = True
        if self.cancelled:
            self.alive = False
            _quiet_close(sock)
            return
        self.events.put(("connected", self.id))
        self._read_loop()

    def _read_loop(self):
        buf = ""
        reason = "The car closed the connection."
        while self.alive:
            try:
                data = self.sock.recv(256)
            except socket.timeout:
                continue
            except OSError as exc:
                reason = describe(exc)
                break
            if not data:
                break
            buf += data.decode("ascii", "replace")
            while True:
                buf = buf.lstrip()
                if not buf:
                    break
                if buf.startswith("OK"):
                    buf = buf[2:]
                    self.events.put(("ack", self.id))
                elif "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    self.events.put(("reply", self.id, line.strip()))
                elif len(buf) >= 2:
                    self.events.put(("reply", self.id, buf.strip()))
                    buf = ""
                else:
                    break
        if self.alive:                       # we did not close it ourselves: the link was lost
            self.alive = False
            self.reason = reason
            _quiet_close(self.sock)
            self.events.put(("lost", self.id, reason))

    def send(self, text):
        """Returns (ok, error_message). Never reports success for a failed send."""
        if not self.alive:
            return False, self.reason
        try:
            self.sock.sendall(text.encode("ascii"))
            return True, ""
        except OSError as exc:
            self.alive = False
            self.reason = describe(exc)
            _quiet_close(self.sock)
            return False, self.reason

    def close(self, send_stop=True):
        self.cancelled = True
        if self.alive and send_stop:
            try:
                self.sock.sendall(("S" + TERMINATOR).encode("ascii"))
            except OSError:
                pass
        self.alive = False
        if self.sock is not None:
            _quiet_close(self.sock)


# --------------------------------------------------------------------------- #
#  Telemetry: JSON lines over TCP (wheel encoders + ultrasonic), background thread
# --------------------------------------------------------------------------- #
class TelemetryStream:
    """Connects to the car's telemetry port and keeps the newest "enc" and "us" message.
    Each stored message is a dict with an extra "_rx" key (time.perf_counter() on arrival).
    Events go to `events` as (kind, stream_id, ...): "live", "failed", "ended"."""

    _ids = itertools.count(1)

    def __init__(self, host, port, events):
        self.id = next(TelemetryStream._ids)
        self.host, self.port, self.events = host, port, events
        self.sock = None
        self.cancelled = False
        self.lock = threading.Lock()
        self.hello = None
        self.enc = None
        self.us = None
        self.missed = 0            # messages the car sent that we never saw (seq gaps)
        self._seq = {}

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def snapshot(self):
        with self.lock:
            return {"hello": self.hello, "enc": self.enc, "us": self.us}

    def close(self):
        self.cancelled = True
        if self.sock is not None:
            _quiet_close(self.sock)

    def _post(self, kind, *args):
        if not self.cancelled:
            self.events.put((kind, self.id) + args)

    def _handle(self, msg, now):
        if not isinstance(msg, dict):
            return
        kind = msg.get("type")
        if kind not in ("hello", "enc", "us"):
            return                                       # unknown message: ignore (forward compat)
        msg["_rx"] = now
        seq = msg.get("seq")
        if isinstance(seq, int) and kind in ("enc", "us"):
            prev = self._seq.get(kind)
            if prev is not None and seq > prev + 1:
                self.missed += seq - prev - 1
            self._seq[kind] = seq
        with self.lock:
            if kind == "hello":
                self.hello = msg
            elif kind == "enc":
                self.enc = msg
            else:
                self.us = msg

    def _run(self):
        try:
            sock = socket.create_connection((self.host, self.port), timeout=TELE_CONNECT_TIMEOUT)
        except OSError as exc:
            self._post("failed", describe_tele(exc))
            return
        self.sock = sock
        if self.cancelled:
            _quiet_close(sock)
            return
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        sock.settimeout(TELE_READ_TIMEOUT)

        reason = "The sensor stream closed."
        buf = b""
        live = False
        try:
            while not self.cancelled:
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    reason = "No sensor data is arriving from the car."
                    break
                except OSError as exc:
                    reason = describe_tele(exc)
                    break
                if not data:
                    break
                buf += data
                if len(buf) > 65536 and b"\n" not in buf:     # garbage without a newline
                    reason = "The sensor stream isn't in the expected format."
                    break
                now = time.perf_counter()
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8", "replace"))
                    except ValueError:
                        continue                              # damaged line: skip it
                    self._handle(msg, now)
                    if not live:
                        live = True
                        self._post("live")
        except Exception as exc:
            reason = "Sensor error: %s" % exc
        finally:
            _quiet_close(sock)
        self._post("ended", reason)


# --------------------------------------------------------------------------- #
#  Camera: raw H.264 over TCP, decoded on a background thread
# --------------------------------------------------------------------------- #
class CameraStream:
    """Connects to rpicam-vid's TCP listener, decodes H.264 and keeps only the newest frame
    (a slow UI can never build up a backlog, so the picture never lags behind).
    latest() -> (version, PIL image, arrival time from time.perf_counter()).
    Events go to `events` as (kind, stream_id, ...): "live", "failed", "ended"."""

    _ids = itertools.count(1)

    def __init__(self, host, port, events):
        self.id = next(CameraStream._ids)
        self.host, self.port, self.events = host, port, events
        self.sock = None
        self.cancelled = False
        self.lock = threading.Lock()
        self._frame, self._ver, self._ts = None, 0, 0.0
        self.fps = 0.0
        self.size = (0, 0)

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def latest(self):
        with self.lock:
            return self._ver, self._frame, self._ts

    def close(self):
        self.cancelled = True
        if self.sock is not None:
            _quiet_close(self.sock)

    def _post(self, kind, *args):
        if not self.cancelled:
            self.events.put((kind, self.id) + args)

    @staticmethod
    def _new_decoder():
        codec = av.CodecContext.create("h264", "r")
        try:
            codec.thread_type = "SLICE"               # no frame-threading delay
        except Exception:
            pass
        try:
            codec.options = {"flags": "low_delay"}
        except Exception:
            pass
        return codec

    def _run(self):
        try:
            sock = socket.create_connection((self.host, self.port), timeout=CAM_CONNECT_TIMEOUT)
        except OSError as exc:
            self._post("failed", describe_cam(exc))
            return
        self.sock = sock
        if self.cancelled:
            _quiet_close(sock)
            return
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        sock.settimeout(CAM_READ_TIMEOUT)

        reason = "The camera stream closed."
        try:
            codec = self._new_decoder()
            live, errors, count, t0 = False, 0, 0, time.perf_counter()
            while not self.cancelled:
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    reason = "No video is arriving from the car."
                    break
                except OSError as exc:
                    reason = describe_cam(exc)
                    break
                if not data:
                    break
                try:
                    last = None
                    for pkt in codec.parse(data):
                        for fr in codec.decode(pkt):
                            last, count = fr, count + 1
                    img = last.to_image() if last is not None else None
                    errors = 0
                except Exception:                    # damaged data: skip, next frame recovers
                    errors += 1
                    if errors > 60:
                        reason = "Couldn't decode the video stream."
                        break
                    continue
                if img is None:
                    continue
                if CAM_ROTATE:
                    img = img.rotate(CAM_ROTATE, expand=True)
                now = time.perf_counter()
                if now - t0 >= 1.0:
                    self.fps, count, t0 = count / (now - t0), 0, now
                self.size = img.size
                with self.lock:
                    self._frame, self._ver, self._ts = img, self._ver + 1, now
                if not live:
                    live = True
                    self._post("live")
        except Exception as exc:
            reason = "Video error: %s" % exc
        finally:
            _quiet_close(sock)
        self._post("ended", reason)


# --------------------------------------------------------------------------- #
#  Object detection (YOLO) on a background thread
# --------------------------------------------------------------------------- #
class Detector:
    """Runs YOLO on the newest camera frame whenever there is one (older frames are skipped,
    so detections always describe the latest picture). Also keeps the selected target matched
    so every result already carries `sel` = index of the target in `dets` (or None).

    result = {"ver", "ts", "w", "h", "dets": [{"label","name","conf","box"}], "sel"}"""

    def __init__(self, model_path):
        self.model_path = model_path
        self.enabled = False
        self.cam = None
        self.result = None
        self.status = "idle"         # idle | loading | ready | error
        self.message = ""
        self.ms = 0.0
        self.target = None           # {"label","name","cx","cy"} (fractions), set by the UI
        self._model = None
        self._np = None
        self._stop = False
        self._last_cam, self._last_ver = None, -1

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop = True

    def _fail(self, msg):
        self.status, self.message, self.enabled = "error", msg, False

    def _load(self):
        self.status, self.message = "loading", ""
        try:
            import numpy as np
            from ultralytics import YOLO
        except Exception:
            self._fail("Detection needs:  pip install ultralytics")
            return False
        if not os.path.exists(self.model_path):
            self._fail("Model file not found: %s" % self.model_path)
            return False
        try:
            model = YOLO(self.model_path)
            model.predict(np.zeros((480, 640, 3), dtype=np.uint8), imgsz=IMG_SIZE, verbose=False)
        except Exception as exc:
            self._fail("Couldn't load the model: %s" % exc)
            return False
        self._np, self._model = np, model
        self.status = "ready"
        return True

    def _run(self):
        while not self._stop:
            if not self.enabled:
                time.sleep(0.05)
                continue
            if self._model is None and not self._load():
                continue
            cam = self.cam
            if cam is None:
                self.result = None
                time.sleep(0.05)
                continue
            if cam is not self._last_cam:
                self._last_cam, self._last_ver = cam, -1
            ver, img, ts = cam.latest()
            if img is None or ver == self._last_ver:
                time.sleep(0.004)
                continue
            self._last_ver = ver
            try:
                np = self._np
                arr = np.ascontiguousarray(np.asarray(img)[:, :, ::-1])     # RGB -> BGR
                t0 = time.perf_counter()
                r = self._model.predict(arr, conf=CONFIDENCE, imgsz=IMG_SIZE, verbose=False)[0]
                self.ms = (time.perf_counter() - t0) * 1000
                dets = []
                if r.boxes is not None and len(r.boxes):
                    xyxy = r.boxes.xyxy.cpu().numpy()
                    conf = r.boxes.conf.cpu().numpy()
                    cls = r.boxes.cls.cpu().numpy().astype(int)
                    for b, c, k in zip(xyxy, conf, cls):
                        dets.append({"label": str(r.names[int(k)]), "conf": float(c),
                                     "box": tuple(float(v) for v in b)})
                dets.sort(key=lambda d: d["box"][0])                        # left to right
                totals, seen = {}, {}
                for d in dets:
                    totals[d["label"]] = totals.get(d["label"], 0) + 1
                for d in dets:
                    seen[d["label"]] = seen.get(d["label"], 0) + 1
                    d["name"] = d["label"] if totals[d["label"]] == 1 else \
                        "%s %d" % (d["label"], seen[d["label"]])
                w, h = img.size
                sel, t = None, self.target
                if t is not None:
                    sel = match_target(dets, t, w, h)
                    if sel is not None:
                        x1, y1, x2, y2 = dets[sel]["box"]
                        t["cx"], t["cy"] = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
                self.result = {"ver": ver, "ts": ts, "w": w, "h": h, "dets": dets, "sel": sel}
            except Exception as exc:
                self.message = "Detection error: %s" % exc
                time.sleep(0.1)


# --------------------------------------------------------------------------- #
#  Video recorder: encodes the live frames to an MP4 at a steady frame rate
# --------------------------------------------------------------------------- #
class Recorder:
    def __init__(self, get_cam, path, fps=REC_FPS):
        self.get_cam, self.path, self.fps = get_cam, path, fps
        self.t0 = time.time()
        self.error = ""
        self.done = False
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True
        self._thread.join(timeout=5)

    def _run(self):
        container = None
        try:
            container = av.open(self.path, "w")
            stream, n, period = None, 0, 1.0 / self.fps
            nxt = time.perf_counter()
            while not self._stop:
                cam = self.get_cam()
                item = cam.latest() if cam is not None else None
                if item is not None and item[1] is not None:
                    img = item[1]
                    if stream is None:
                        try:
                            stream = container.add_stream("libx264", rate=self.fps)
                            opts = {"preset": "veryfast", "crf": "23"}
                        except Exception:
                            stream = container.add_stream("mpeg4", rate=self.fps)
                            opts = {}
                        stream.width, stream.height = img.size[0] & ~1, img.size[1] & ~1
                        stream.pix_fmt = "yuv420p"
                        try:
                            stream.codec_context.options = opts
                        except Exception:
                            pass
                    if img.size != (stream.width, stream.height):
                        img = img.resize((stream.width, stream.height))
                    vf = av.VideoFrame.from_image(img)
                    vf.pts, vf.time_base = n, Fraction(1, self.fps)
                    n += 1
                    for pkt in stream.encode(vf):
                        container.mux(pkt)
                nxt += period
                delay = nxt - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -1.0:
                    nxt = time.perf_counter()
            if stream is not None:
                for pkt in stream.encode():
                    container.mux(pkt)
        except Exception as exc:
            self.error = "Recording failed: %s" % exc
        finally:
            try:
                if container is not None:
                    container.close()
            except Exception:
                pass
            self.done = True


# --------------------------------------------------------------------------- #
#  Widgets
# --------------------------------------------------------------------------- #
class AnimCanvas(tk.Canvas):
    """Canvas with eased hover / press values and a decaying 'flash' for send feedback."""

    FRAME_MS = 16

    def __init__(self, master, bg, **kw):
        kw.setdefault("highlightthickness", 0)
        kw.setdefault("bd", 0)
        super().__init__(master, bg=bg, **kw)
        self.hover = self.press = self.flash = 0.0
        self.hover_t = self.press_t = 0.0
        self._looping = False
        self.bind("<Configure>", lambda e: self._draw())

    def pulse(self):
        self.flash = 1.0
        self.kick()

    def kick(self):
        if not self._looping:
            self._looping = True
            self.after(1, self._tick)

    def _tick(self):
        busy = self.step()
        self._draw()
        if busy:
            self.after(self.FRAME_MS, self._tick)
        else:
            self._looping = False

    def step(self):
        busy = False
        for name, rate in (("hover", 0.22), ("press", 0.38)):
            cur, tgt = getattr(self, name), getattr(self, name + "_t")
            d = tgt - cur
            if abs(d) > 0.01:
                setattr(self, name, cur + d * rate)
                busy = True
            else:
                setattr(self, name, tgt)
        if self.flash > 0:
            self.flash = max(0.0, self.flash - 0.05)
            busy = True
        return busy

    def _draw(self):
        pass


class PadButton(AnimCanvas):
    """A large drive control. Reports press / release; the app decides what they mean."""

    def __init__(self, master, *, code, label, icon, on_press, on_release=None,
                 on_blocked=None, radius=26, strong=False, breathe=False, self_press=False):
        super().__init__(master, BG, width=1, height=1)
        self.code, self.label, self.icon = code, label, icon
        self.accent = COMMANDS[code][1]
        self.on_press, self.on_release, self.on_blocked = on_press, on_release, on_blocked
        self.radius, self.strong, self.breathe, self.self_press = radius, strong, breathe, self_press
        self.enabled = False
        self._mouse = False
        self.bind("<ButtonPress-1>", self._down)
        self.bind("<ButtonRelease-1>", self._up)
        self.bind("<Enter>", lambda e: self._set_hover(1.0))
        self.bind("<Leave>", lambda e: self._set_hover(0.0))

    # -- state ---------------------------------------------------------------
    def _set_hover(self, v):
        self.hover_t = v if self.enabled else 0.0
        self.kick()

    def set_enabled(self, on):
        self.enabled = on
        if not on:
            self.press_t = self.hover_t = 0.0
        self.config(cursor="hand2" if on else "arrow")
        self.kick()

    def set_pressed(self, on):
        self.press_t = 1.0 if on else 0.0
        self.kick()

    def step(self):
        busy = super().step()
        return busy or (self.breathe and self.enabled)

    # -- mouse ---------------------------------------------------------------
    def _down(self, _e):
        self.focus_set()                       # take keyboard focus away from the address box
        if not self.enabled:
            if self.on_blocked:
                self.on_blocked()
            return
        self._mouse = True
        if self.self_press:
            self.set_pressed(True)
        self.on_press()

    def _up(self, _e):
        if not self._mouse:
            return
        self._mouse = False
        if self.self_press:
            self.set_pressed(False)
        if self.on_release:
            self.on_release()

    # -- drawing -------------------------------------------------------------
    def _draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        pad = px(10)
        if w < pad * 3 or h < pad * 3:
            return
        accent, p, hv, fl = self.accent, self.press, self.hover, self.flash

        if self.enabled:
            wave = 0.5 + 0.5 * math.sin(time.time() * 2.2) if self.breathe else 0.0
            glow = max(p * 0.85, fl * 0.95, wave * 0.30 * (1 - p))
            base = (0.16 if self.strong else 0.09) + 0.07 * hv
            fill = mix(SURFACE, accent, base + (0.86 - base) * p)
            border = mix(SURFACE, accent, 0.34 + 0.30 * hv + 0.36 * p)
            fg = mix(accent, BG, p)
        else:
            glow = 0.0
            fill = mix(SURFACE, accent, 0.04)
            border = mix(BORDER, accent, 0.14)
            fg = mix(SURFACE_2, accent, 0.30)
        sub = mix(fg, fill, 0.45)

        inset = pad + 2 * S * p
        x1, y1, x2, y2 = inset, inset, w - inset, h - inset
        r = px(self.radius)

        if glow > 0.02:
            for i in range(5, 0, -1):
                g = i * pad / 5.6
                c = mix(BG, accent, glow * 0.36 * (1 - (i - 1) / 5.0))
                round_rect(self, x1 - g, y1 - g, x2 + g, y2 + g, r + g, outline=c, width=2, fill="")

        round_rect(self, x1, y1, x2, y2, r, fill=fill, outline=border, width=2)

        # content
        cx = (x1 + x2) / 2
        bw, bh = x2 - x1, y2 - y1
        unit = min(bw, bh * 1.15)
        s = max(px(9), min(px(34), unit * 0.15))
        if self.icon == "stop":
            s *= 1.25
        fpx = int(max(px(12), min(px(19), unit * 0.09)))
        lines = self.label.count("\n") + 1
        show_cap = bh > px(112)
        text_h = fpx * 1.3 * lines
        gap = s * 0.75
        total = 2 * s + gap + text_h
        top = (y1 + y2) / 2 - total / 2 - (px(9) if show_cap else 0)

        self._icon(cx, top + s, s, fg)
        bold = "bold" if self.code == "S" else "normal"
        self.create_text(cx, top + 2 * s + gap, text=self.label, anchor="n", justify="center",
                         fill=fg, font=(FONT_SB if self.code != "S" else FONT, -fpx, bold))

        if show_cap:
            cap = KEYCAP[self.code]
            cw, ch = px(16) + len(cap) * px(8), px(20)
            ky = y2 - px(14) - ch / 2
            round_rect(self, cx - cw / 2, ky - ch / 2, cx + cw / 2, ky + ch / 2, px(6),
                       outline=sub, fill="", width=1)
            self.create_text(cx, ky, text=cap, fill=sub, font=(FONT, -px(11)))

    def _icon(self, cx, cy, s, color):
        lw = max(px(3), s * 0.27)
        if self.icon == "stop":
            a = s * 0.8
            round_rect(self, cx - a, cy - a, cx + a, cy + a, a * 0.3, fill=color, outline="")
        elif self.icon in ("up", "down", "left", "right"):
            chevron(self, cx, cy, s, self.icon, color, lw)
        else:  # sharp turns: double chevron
            d = "left" if self.icon == "left2" else "right"
            k = s * 0.86
            chevron(self, cx, cy, k, d, color, lw, offset=-0.44 * k)
            chevron(self, cx, cy, k, d, color, lw, offset=0.44 * k)


class VideoView(AnimCanvas):
    """The live camera panel: rounded video with a HUD (LIVE, fps, REC, camera / full-screen
    toggles, current drive command, live sensor chips), detection boxes (click one to select
    it) and a glowing border in the colour of the active command. Shows a status card when
    there is no picture."""

    R = 24
    CHIP_BG = mix(VIDEO_BG, "#FFFFFF", 0.10)
    CHIP_BORDER = mix(VIDEO_BG, "#FFFFFF", 0.22)
    PH = {                                   # placeholder colour, draw a slash through the icon
        "idle": (MUTED, True),
        "paused": (MUTED, True),
        "connecting": (AMBER, False),
        "error": (RED, True),
        "nolib": (AMBER, True),
    }

    def __init__(self, master, on_toggle, on_fullscreen, on_pick):
        super().__init__(master, BG, width=1, height=1)
        self.on_toggle, self.on_fullscreen, self.on_pick = on_toggle, on_fullscreen, on_pick
        self.f_pill = tkfont.Font(family=FONT_SB, size=-px(12))
        self.f_title = tkfont.Font(family=FONT_SB, size=-px(18))
        self.f_sub = tkfont.Font(family=FONT, size=-px(12))

        self.mode, self.title, self.sub = "idle", "Camera offline", ""
        self.stream = None
        self.car_on = False
        self.cam_on = True
        self.fullscreen = False
        self.cmd = None
        self.accent = BLUE
        self.drive = self.drive_t = 0.0
        self.det = None                     # Detector (shared with the app)
        self.det_on = False
        self.rec_secs = None                # None = not recording
        self.banner = None                  # (text, colour) shown bottom-left
        self.sensors = None                 # callable -> fresh sensor values (see App.sensor_view)

        self._ver, self._img = -1, None
        self._ver_drawn, self._photo, self._photo_key = -2, None, None
        self._mask, self._mask_key = None, None
        self._hits, self._det_hits, self._hover = [], [], None

        self.bind("<ButtonPress-1>", self._click)
        self.bind("<Double-Button-1>", self._double)
        self.bind("<Motion>", self._motion)
        self.bind("<Leave>", lambda e: self._set_hover(None))

    # -- state from the app ----------------------------------------------------
    def set_mode(self, mode, title="", sub=""):
        self.mode, self.title, self.sub = mode, title, sub
        self.FRAME_MS = 33 if mode == "live" else 16
        self.kick()

    def attach(self, stream):
        self.stream = stream
        self._ver, self._img = -1, None
        self._ver_drawn, self._photo, self._photo_key = -2, None, None

    def set_car(self, on):
        self.car_on = on
        self.kick()

    def set_command(self, code):
        self.cmd = code
        if code and code != "S":
            self.accent = COMMANDS[code][1]
            self.drive_t = 1.0
        else:
            self.drive_t = 0.0
        self.kick()

    # -- interaction -----------------------------------------------------------
    def _hit_at(self, x, y):
        for x1, y1, x2, y2, name in self._hits:
            if x1 <= x <= x2 and y1 <= y <= y2:
                return name
        return None

    def _det_at(self, x, y):
        best = None
        for x1, y1, x2, y2, i in self._det_hits:
            if x1 <= x <= x2 and y1 <= y <= y2:
                area = (x2 - x1) * (y2 - y1)
                if best is None or area < best[0]:
                    best = (area, i)
        return None if best is None else best[1]

    def _set_hover(self, name):
        if name != self._hover:
            self._hover = name
            self.config(cursor="hand2" if name else "arrow")
            self.kick()

    def _motion(self, e):
        name = self._hit_at(e.x, e.y)
        if name is None and self._det_at(e.x, e.y) is not None:
            name = "det"
        self._set_hover(name)

    def _click(self, e):
        self.focus_set()
        name = self._hit_at(e.x, e.y)
        if name == "cam":
            self.on_toggle()
        elif name == "fs":
            self.on_fullscreen()
        elif name is None:
            i = self._det_at(e.x, e.y)
            if i is not None:
                self.on_pick(i)

    def _double(self, e):
        if self._hit_at(e.x, e.y) is None and self._det_at(e.x, e.y) is None:
            self.on_fullscreen()

    # -- animation -------------------------------------------------------------
    def step(self):
        busy = super().step()
        d = self.drive_t - self.drive
        if abs(d) > 0.01:
            self.drive += d * 0.25
            busy = True
        else:
            self.drive = self.drive_t
        if self.mode == "live" and self.stream is not None:
            ver, img, _ts = self.stream.latest()
            if img is not None and ver != self._ver:
                self._ver, self._img = ver, img
        return busy or self.mode in ("live", "connecting")

    # -- drawing helpers -------------------------------------------------------
    def _pill(self, x, cy, text, font, fg, bg, border, anchor="w", dot=None, pulse=False, hit=None):
        pad, dw, ph = px(12), (px(16) if dot else 0), px(28)
        pw = font.measure(text) + 2 * pad + dw
        x1 = x if anchor == "w" else (x - pw if anchor == "e" else x - pw / 2)
        x2 = x1 + pw
        if hit and hit == self._hover:
            bg, border = mix(bg, "#FFFFFF", 0.09), mix(border, "#FFFFFF", 0.20)
        round_rect(self, x1, cy - ph / 2, x2, cy + ph / 2, ph / 2, fill=bg, outline=border)
        tx = x1 + pad
        if dot:
            dx = tx + px(5)
            if pulse:
                t = (time.time() * 0.9) % 1.0
                rr = px(4) + px(6) * t
                self.create_oval(dx - rr, cy - rr, dx + rr, cy + rr, width=2,
                                 outline=mix(bg, dot, 0.7 * (1 - t)))
            d = px(4)
            self.create_oval(dx - d, cy - d, dx + d, cy + d, fill=dot, outline="")
            tx += dw
        self.create_text(tx, cy, text=text, anchor="w", fill=fg, font=font)
        if hit:
            self._hits.append((x1, cy - ph / 2, x2, cy + ph / 2, hit))
        return x1, x2

    def _cam_icon(self, cx, cy, s, color, slash):
        lw = max(px(3), s * 0.09)
        round_rect(self, cx - s, cy - s * 0.62, cx + s * 0.55, cy + s * 0.62, s * 0.22,
                   outline=color, fill="", width=lw)
        self.create_polygon(cx + s * 0.72, cy - s * 0.26, cx + s * 1.05, cy - s * 0.46,
                            cx + s * 1.05, cy + s * 0.46, cx + s * 0.72, cy + s * 0.26,
                            outline=color, fill="", width=lw, joinstyle="round")
        r = s * 0.27
        self.create_oval(cx - s * 0.22 - r, cy - r, cx - s * 0.22 + r, cy + r,
                         outline=color, width=lw)
        if slash:
            a, b = (cx - s * 1.15, cy - s * 0.95), (cx + s * 1.2, cy + s * 0.95)
            self.create_line(*a, *b, fill=VIDEO_BG, width=lw + px(6), capstyle="round")
            self.create_line(*a, *b, fill=color, width=lw, capstyle="round")

    def _brackets(self, x1, y1, x2, y2, color):
        i, ln = px(24), px(20)
        for sx, sy, bx, by in ((1, 1, x1, y1), (-1, 1, x2, y1), (1, -1, x1, y2), (-1, -1, x2, y2)):
            cx, cy = bx + sx * i, by + sy * i
            self.create_line(cx + sx * ln, cy, cx, cy, cx, cy + sy * ln, fill=color,
                             width=2, capstyle="round", joinstyle="round")

    def _get_mask(self, iw, ih, r):
        key = (iw, ih, r)
        if self._mask_key != key:
            big = Image.new("L", (iw * 2, ih * 2), 0)
            ImageDraw.Draw(big).rounded_rectangle((0, 0, iw * 2 - 1, ih * 2 - 1),
                                                  radius=r * 2, fill=255)
            self._mask = big.resize((iw, ih), RESAMPLE_HQ)
            self._mask_key = key
        return self._mask

    def _render(self, iw, ih, r):
        key = (iw, ih)
        if self._photo is not None and self._photo_key == key and self._ver_drawn == self._ver:
            return
        fw, fh = self._img.size
        sc = min(iw / fw, ih / fh)
        nw, nh = max(1, int(fw * sc)), max(1, int(fh * sc))
        frame = self._img.resize((nw, nh), RESAMPLE)
        canvas = Image.new("RGB", (iw, ih), _rgb(VIDEO_BG))
        canvas.paste(frame, ((iw - nw) // 2, (ih - nh) // 2))
        canvas.putalpha(self._get_mask(iw, ih, r))
        self._photo = ImageTk.PhotoImage(canvas)
        self._photo_key, self._ver_drawn = key, self._ver

    def _draw_dets(self, x1, y1, x2, y2):
        """Detection boxes on top of the picture; also registers them as click targets."""
        self._det_hits = []
        det = self.det
        if not self.det_on or det is None or self._img is None:
            return
        res = det.result
        if res is None:
            return
        fw, fh = self._img.size
        if (res["w"], res["h"]) != (fw, fh):
            return
        iw, ih = x2 - x1, y2 - y1
        sc = min(iw / fw, ih / fh)
        ox, oy = x1 + (iw - fw * sc) / 2, y1 + (ih - fh * sc) / 2
        sel = res.get("sel")
        dets = res["dets"]
        if sel is not None:                                   # the "centred" band the car steers to
            mx, half = ox + fw * sc / 2, DEADZONE * fw * sc / 2
            for xx in (mx - half, mx + half):
                self.create_line(xx, oy, xx, oy + fh * sc, fill=mix(VIDEO_BG, GREEN, 0.45), dash=(4, 6))
        order = [i for i in range(len(dets)) if i != sel] + ([sel] if sel is not None else [])
        for i in order:
            d = dets[i]
            b = d["box"]
            a0, a1, a2, a3 = ox + b[0] * sc, oy + b[1] * sc, ox + b[2] * sc, oy + b[3] * sc
            chosen = i == sel
            col = GREEN if chosen else BLUE
            round_rect(self, a0, a1, a2, a3, px(6), outline=col, fill="", width=3 if chosen else 2)
            txt = "%s %d%%" % (d["name"], round(d["conf"] * 100))
            cw, ch = self.f_pill.measure(txt) + px(14), px(20)
            cx1 = max(x1 + px(4), min(a0, x2 - cw - px(4)))
            cy1 = a1 - ch - px(2)
            if cy1 < y1 + px(4):
                cy1 = a1 + px(2)
            round_rect(self, cx1, cy1, cx1 + cw, cy1 + ch, px(6), fill=col, outline="")
            self.create_text(cx1 + px(7), cy1 + ch / 2, text=txt, anchor="w",
                             fill=VIDEO_BG, font=self.f_pill)
            self._det_hits.append((a0, a1, a2, a3, i))

    # -- drawing ---------------------------------------------------------------
    def _draw(self):
        self.delete("all")
        self._hits = []
        self._det_hits = []
        w, h = self.winfo_width(), self.winfo_height()
        m = px(10)                                  # same inset as the drive buttons
        if w < m * 4 or h < m * 4:
            return
        x1, y1, x2, y2 = m, m, w - m, h - m
        r = px(self.R)
        drive, acc = self.drive, self.accent

        if drive > 0.02:
            for i in range(5, 0, -1):
                g = i * m / 5.6
                c = mix(BG, acc, drive * 0.30 * (1 - (i - 1) / 5.0))
                round_rect(self, x1 - g, y1 - g, x2 + g, y2 + g, r + g, outline=c, width=2, fill="")

        round_rect(self, x1, y1, x2, y2, r, fill=VIDEO_BG, outline="")

        live = self.mode == "live" and self._img is not None
        if live:
            self._render(int(x2 - x1), int(y2 - y1), r)
            self.create_image(x1, y1, image=self._photo, anchor="nw")
            self._draw_dets(x1, y1, x2, y2)
        else:
            self._placeholder(x1, y1, x2, y2)

        if self.flash > 0.5:                        # camera-shutter flash for photos
            self.create_rectangle(x1, y1, x2, y2, fill="#FFFFFF", stipple="gray25", outline="")
        border = mix(mix(BORDER, acc, 0.85 * drive), "#FFFFFF", self.flash)
        round_rect(self, x1, y1, x2, y2, r, outline=border, width=2, fill="")
        self._hud(x1, y1, x2, y2, live)

    def _placeholder(self, x1, y1, x2, y2):
        key = self.mode if self.mode in self.PH else "connecting"
        color, slash = self.PH[key]
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        self._brackets(x1, y1, x2, y2, mix(VIDEO_BG, MUTED, 0.30))
        s = px(30)
        iy = cy - px(38)
        if key == "connecting":
            rr = s * 1.75
            self.create_oval(cx - rr, iy - rr, cx + rr, iy + rr, outline=mix(VIDEO_BG, color, 0.16), width=3)
            self.create_arc(cx - rr, iy - rr, cx + rr, iy + rr, start=-(time.time() * 260) % 360,
                            extent=95, style="arc", outline=color, width=3)
        self._cam_icon(cx, iy, s, color, slash)
        self.create_text(cx, iy + px(62), text=self.title, fill=TEXT, font=self.f_title)
        if self.sub:
            self.create_text(cx, iy + px(86), text=self.sub, fill=MUTED, font=self.f_sub,
                             anchor="n", justify="center", width=int(min((x2 - x1) * 0.72, px(560))))

    def _hud(self, x1, y1, x2, y2, live):
        ty = y1 + px(28)
        # top-right: controls (always available)
        a, _ = self._pill(x2 - px(18), ty, "Exit full screen" if self.fullscreen else "Full screen",
                          self.f_pill, TEXT, self.CHIP_BG, self.CHIP_BORDER, anchor="e", hit="fs")
        self._pill(a - px(8), ty, "Camera on" if self.cam_on else "Camera off", self.f_pill,
                   TEXT if self.cam_on else MUTED, self.CHIP_BG, self.CHIP_BORDER, anchor="e",
                   dot=GREEN if self.cam_on else MUTED, hit="cam")
        if not live:
            return
        # top-left: LIVE + stream stats + REC
        _, nx = self._pill(x1 + px(18), ty, "LIVE", self.f_pill, TEXT, self.CHIP_BG,
                           self.CHIP_BORDER, anchor="w", dot=RED, pulse=True)
        nx += px(8)
        st = self.stream
        if st is not None and st.size[0]:
            txt = "%dx%d" % st.size
            if st.fps >= 1:
                txt = "%d fps   %s" % (round(st.fps), txt)
            _, nx = self._pill(nx, ty, txt, self.f_pill, MUTED, self.CHIP_BG, self.CHIP_BORDER)
            nx += px(8)
        if self.rec_secs is not None:
            s = int(self.rec_secs)
            self._pill(nx, ty, "REC %02d:%02d" % (s // 60, s % 60), self.f_pill, TEXT,
                       mix(VIDEO_BG, RED, 0.35), RED, dot=RED, pulse=True)
        # bottom-left: what go-to-object is doing
        if self.banner:
            self._pill(x1 + px(18), y2 - px(30), self.banner[0], self.f_pill, TEXT,
                       mix(VIDEO_BG, self.banner[1], 0.35), self.banner[1], anchor="w",
                       dot=self.banner[1], pulse=True)
        # bottom-centre: what the car is doing right now
        if self.car_on:
            code = self.cmd or "S"
            name, color = COMMANDS[code][:2]
            driving = code != "S"
            self._pill((x1 + x2) / 2, y2 - px(30), name if driving else "Stopped", self.f_pill, TEXT,
                       mix(VIDEO_BG, color, 0.42) if driving else self.CHIP_BG,
                       color if driving else self.CHIP_BORDER, anchor="c", dot=color)
        # bottom-right: live sensor readings (distance, wheel speed)
        sv = self.sensors() if self.sensors else None
        if sv:
            by = y2 - px(30)
            us = sv["us"]
            if us is not None:
                cm = us["cm"]
                if cm is None:
                    txt, col = "Distance   no echo", MUTED
                else:
                    txt, col = "Distance   %.0f cm" % cm, distance_color(cm)
                self._pill(x2 - px(18), by, txt, self.f_pill, TEXT, mix(VIDEO_BG, col, 0.35), col,
                           anchor="e", dot=col)
                by -= px(36)
            enc = sv["enc"]
            if enc is not None:
                self._pill(x2 - px(18), by, "L %.0f   R %.0f rpm" % (enc["left"]["rpm"],
                                                                      enc["right"]["rpm"]),
                           self.f_pill, TEXT, self.CHIP_BG, self.CHIP_BORDER, anchor="e")


class ActionButton(AnimCanvas):
    """Pill button for Connect / Cancel / Disconnect and the side panel actions."""

    STYLES = {"primary": (GREEN, True), "neutral": (MUTED, False), "danger": (RED, False)}

    def __init__(self, master, bg, command, width, height):
        super().__init__(master, bg, width=px(width), height=px(height), cursor="hand2")
        self.command, self.text, self.kind = command, "Connect", "primary"
        self.bind("<ButtonPress-1>", lambda e: self._set("press_t", 1.0))
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Enter>", lambda e: self._set("hover_t", 1.0))
        self.bind("<Leave>", lambda e: self._set("hover_t", 0.0))

    def _set(self, name, v):
        setattr(self, name, v)
        self.kick()

    def _release(self, e):
        self._set("press_t", 0.0)
        if 0 <= e.x < self.winfo_width() and 0 <= e.y < self.winfo_height():
            self.command()

    def configure_style(self, text, kind):
        if (text, kind) == (self.text, self.kind):
            return
        self.text, self.kind = text, kind
        self.kick()

    def _draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 20:
            return
        color, filled = self.STYLES[self.kind]
        hv, p, fl = self.hover, self.press, self.flash
        m = px(4)
        if fl > 0.02:
            for i in range(3, 0, -1):
                g = i * px(1.2)
                round_rect(self, m - g, m - g, w - m + g, h - m + g, (h - 2 * m) / 2 + g,
                           outline=mix(self["bg"], color, fl * 0.5 * (1 - (i - 1) / 3.0)),
                           width=2, fill="")
        if filled:
            fill = mix(mix(color, "#FFFFFF", 0.10 * hv), BG, 0.18 * p)
            round_rect(self, m, m, w - m, h - m, (h - 2 * m) / 2, fill=fill, outline="")
            fg = BG
        else:
            fill = mix(SURFACE, color, 0.10 + 0.10 * hv + 0.16 * p)
            round_rect(self, m, m, w - m, h - m, (h - 2 * m) / 2, fill=fill,
                       outline=mix(SURFACE, color, 0.55 + 0.25 * hv), width=1)
            fg = color if self.kind == "danger" else mix(TEXT, MUTED, 0.3)
        self.create_text(w / 2, h / 2, text=self.text, fill=fg, font=(FONT_SB, 11))


class StatusPill(AnimCanvas):
    COLORS = {"disconnected": RED, "connecting": AMBER, "connected": GREEN}

    def __init__(self, master):
        super().__init__(master, BG, width=px(420), height=px(40))
        self.font = tkfont.Font(family=FONT, size=11, weight="bold")
        self.mode, self.text = "disconnected", "Disconnected"

    def set(self, mode, text):
        self.mode, self.text = mode, text
        self.kick()

    def step(self):
        busy = super().step()
        return busy or self.mode != "disconnected"

    def _draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 40:
            return
        color = self.COLORS[self.mode]
        pw = self.font.measure(self.text) + px(58)
        x2, y1, y2 = w - px(2), px(3), h - px(3)
        x1 = x2 - pw
        pill_bg = mix(BG, color, 0.10)
        round_rect(self, x1, y1, x2, y2, (y2 - y1) / 2, fill=pill_bg, outline=mix(BG, color, 0.38))
        cx, cy = x1 + px(22), h / 2
        speed = 1.6 if self.mode == "connecting" else 0.7
        ph = (time.time() * speed) % 1.0
        rr = px(5) + px(7) * ph
        self.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                         outline=mix(pill_bg, color, 0.7 * (1 - ph)), width=2)
        d = px(5)
        self.create_oval(cx - d, cy - d, cx + d, cy + d, fill=color, outline="")
        self.create_text(x1 + px(38), cy, text=self.text, anchor="w",
                         fill=mix(TEXT, color, 0.15), font=self.font)


class FeedbackBar(AnimCanvas):
    """Shows the last command sent and whether the car acknowledged it."""

    def __init__(self, master):
        super().__init__(master, BG, height=px(60))
        self.left, self.left_color = "Not connected", MUTED
        self.dot = None
        self.code = ""
        self.right, self.right_color = "", MUTED

    def show_command(self, code):
        name, color = COMMANDS[code][:2]
        self.left, self.left_color, self.dot, self.code = name, TEXT, color, code
        self.right, self.right_color = "Sent, waiting for OK", MUTED
        self.pulse()

    def status(self, text, color):
        self.right, self.right_color = text, color
        self.kick()

    def message(self, text, color=MUTED):
        self.left, self.left_color, self.dot, self.code = text, color, None, ""
        self.right = ""
        self.kick()

    def _draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 60:
            return
        round_rect(self, 1, 1, w - 1, h - 1, px(18), fill=SURFACE, outline=BORDER)
        cy, x = h / 2, px(28)
        if self.dot:
            if self.flash > 0.02:
                rr = px(6) + px(16) * (1 - self.flash)
                self.create_oval(x - rr, cy - rr, x + rr, cy + rr, width=2,
                                 outline=mix(SURFACE, self.dot, 0.75 * self.flash))
            d = px(6)
            self.create_oval(x - d, cy - d, x + d, cy + d, fill=self.dot, outline="")
            x += px(22)
        tid = self.create_text(x, cy, text=self.left, anchor="w", fill=self.left_color,
                               font=(FONT_SB, 14))
        if self.code:
            x2 = self.bbox(tid)[2] + px(14)
            cw, ch = px(14) + len(self.code) * px(9), px(22)
            round_rect(self, x2, cy - ch / 2, x2 + cw, cy + ch / 2, px(7),
                       fill=SURFACE_2, outline=BORDER)
            self.create_text(x2 + cw / 2, cy, text=self.code, fill=MUTED, font=(FONT, 10, "bold"))
        if self.right:
            self.create_text(w - px(28), cy, text=self.right, anchor="e",
                             fill=self.right_color, font=(FONT, 11))


class ConnBar(tk.Canvas):
    """Address box and connect button on one rounded strip."""

    PLACEHOLDER = "192.168.1.50"

    def __init__(self, master, on_toggle, on_enter):
        super().__init__(master, bg=BG, height=px(68), highlightthickness=0, bd=0)
        self._ph = False
        self.entry = tk.Entry(
            self, font=("Consolas", 13), bg=SURFACE_2, fg=TEXT, insertbackground=TEXT,
            relief="flat", bd=0, justify="left",
            disabledbackground=SURFACE, disabledforeground=MUTED,
            highlightthickness=1, highlightbackground=BORDER, highlightcolor=BLUE)
        self.entry.bind("<FocusIn>", self._ph_in)
        self.entry.bind("<FocusOut>", self._ph_out)
        self.entry.bind("<Return>", lambda e: on_enter())
        self.button = ActionButton(self, SURFACE, on_toggle, 156, 44)
        self.w_entry = self.create_window(0, 0, window=self.entry, anchor="w")
        self.w_btn = self.create_window(0, 0, window=self.button, anchor="e")
        self.bind("<Configure>", self._layout)
        self._ph_out()

    # placeholder handling
    def _ph_in(self, _e=None):
        if self._ph:
            self.entry.delete(0, "end")
            self.entry.config(fg=TEXT)
            self._ph = False

    def _ph_out(self, _e=None):
        if not self.entry.get() and str(self.entry["state"]) != "disabled":
            self.entry.insert(0, self.PLACEHOLDER)
            self.entry.config(fg=MUTED)
            self._ph = True

    def get_host(self):
        return "" if self._ph else self.entry.get().strip()

    def set_host(self, host):
        self._ph_in()
        self.entry.delete(0, "end")
        self.entry.insert(0, host)
        self.entry.config(fg=TEXT)
        self._ph = False

    def set_mode(self, mode):
        if mode == "disconnected":
            self.entry.config(state="normal")
            self.button.configure_style("Connect", "primary")
            self._ph_out()
        else:
            self._ph_in()
            self.entry.config(state="disabled")
            if mode == "connecting":
                self.button.configure_style("Cancel", "neutral")
            else:
                self.button.configure_style("Disconnect", "danger")

    def _layout(self, _e=None):
        w, h = self.winfo_width(), self.winfo_height()
        if w < 100:
            return
        self.delete("bg")
        rr = round_rect(self, 1, 1, w - 1, h - 1, px(20), fill=SURFACE, outline=BORDER, tags="bg")
        self.tag_lower(rr)
        self.create_text(px(28), h / 2, text="Raspberry Pi address", anchor="w", fill=MUTED,
                         font=(FONT, 10), tags="bg")
        ex = px(190)
        self.coords(self.w_entry, ex, h / 2)
        self.itemconfigure(self.w_entry, width=px(230), height=px(38))
        self.create_text(ex + px(230) + px(14), h / 2, text=":%d" % PORT, anchor="w",
                         fill=MUTED, font=("Consolas", 13), tags="bg")
        self.coords(self.w_btn, w - px(14), h / 2)


# --------------------------------------------------------------------------- #
#  Application
# --------------------------------------------------------------------------- #
class App:
    def __init__(self):
        global S
        if sys.platform == "win32":
            try:
                ctypes.windll.shcore.SetProcessDpiAwareness(1)
            except Exception:
                try:
                    ctypes.windll.user32.SetProcessDPIAware()
                except Exception:
                    pass
        self.root = root = tk.Tk()
        S = max(1.0, root.winfo_fpixels("1i") / 96.0)
        root.title("RC Controller")
        root.configure(bg=BG)
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        w0, h0 = min(px(1440), int(sw * 0.94)), min(px(880), int(sh * 0.92))
        root.geometry("%dx%d+%d+%d" % (w0, h0, (sw - w0) // 2, max(0, (sh - h0) // 2 - px(20))))
        root.minsize(min(px(1180), w0), min(px(720), h0))

        self.events = queue.Queue()
        self.cam_events = queue.Queue()
        self.tele_events = queue.Queue()
        self.session = None
        self.cam = None
        self.tele = None
        self.tele_live = False
        self.cam_wanted = True
        self.focus_mode = False
        self._cam_retry_job = None
        self._tele_retry_job = None
        self.state = "disconnected"
        self.held = []                 # [(source, code)], most recent last
        self.last_cmd = None
        self.seq = 0
        self.shown_seq = None
        self.pending = deque(maxlen=64)
        self._warned = False
        self._keys_down = set()
        self._release_jobs = {}

        # vision / follow / capture state
        self.det = Detector(resolve_model())
        self.det.enabled = VIDEO_OK
        self.target = None             # {"label","name","cx","cy"}
        self.follow = None             # dict while go-to-object is running
        self.rec = None
        self._ui_t = 0.0
        self._list_sig = None

        self._build()
        self.video.det, self.video.det_on = self.det, self.det.enabled
        self.video.sensors = self.sensor_view
        self._apply_state("disconnected")
        self._cam_idle()
        self._sync_side()
        self._sensor_ui()
        self.feedback.message("Enter the Pi's IP address and connect", MUTED)
        self.det.start()

        root.bind_all("<KeyPress>", self._key_down)
        root.bind_all("<KeyRelease>", self._key_up)
        root.bind_all("<F11>", lambda e: self.toggle_focus())
        root.bind("<FocusOut>", lambda e: root.after(60, self._check_focus))
        root.bind("<FocusIn>", self._focus_in)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(8, self._poll)
        root.after(250, self._watchdog)
        self._dark_title_bar()

    # -- UI construction -------------------------------------------------------
    def _build(self):
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=px(22), pady=(px(18), px(12)))

        hdr = tk.Frame(main, bg=BG)
        hdr.pack(fill="x", padx=px(10), pady=(0, px(12)))
        tk.Label(hdr, text="RC Controller", bg=BG, fg=TEXT, font=(FONT_SB, 20)).pack(side="left")
        self.pill = StatusPill(hdr)
        self.pill.pack(side="right")

        self.conn = ConnBar(main, self.toggle_connection, self._enter_pressed)
        self.conn.pack(fill="x", padx=px(10), pady=(0, px(6)))
        saved = load_host()
        if saved:
            self.conn.set_host(saved)

        tk.Label(main, bg=BG, fg=MUTED, font=(FONT, 9),
                 text="Hold a control to drive, release to stop.   Keys: W A S D or arrows, Q and E "
                      "sharp turns, Space stop, P photo, R record, G go to object, F11 full screen."
                 ).pack(side="bottom", pady=(px(8), 0))
        self.feedback = FeedbackBar(main)
        self.feedback.pack(side="bottom", fill="x", padx=px(10), pady=(px(6), 0))

        self.body = body = tk.Frame(main, bg=BG)
        body.pack(fill="both", expand=True)
        self._build_side(body)

        self.pads = pads = tk.Frame(body, bg=BG)
        pads.pack(side="left", fill="both", expand=True)
        for c, wgt in enumerate((2, 2, 6, 6, 2, 2)):
            pads.grid_columnconfigure(c, weight=wgt, uniform="cols")
        for r, wgt in enumerate((2, 12, 3)):
            pads.grid_rowconfigure(r, weight=wgt, uniform="rows")

        self.buttons = {}

        def add(code, label, icon, col, row, span=1, cspan=1, **kw):
            src = ("mouse", code)
            b = PadButton(pads, code=code, label=label, icon=icon,
                          on_press=lambda: self.press(src, code),
                          on_release=None if code == "S" else (lambda: self.release(src)),
                          on_blocked=self.blocked, **kw)
            b.grid(row=row, column=col, rowspan=span, columnspan=cspan, sticky="nsew")
            self.buttons[code] = b

        add("SL", "Sharp\nleft", "left2", 0, 0, 3)
        add("L", "Left", "left", 1, 0, 3)
        add("F", "Forward", "up", 2, 0, cspan=2)
        add("B", "Reverse", "down", 2, 2)
        add("S", "STOP", "stop", 3, 2, strong=True, breathe=True, self_press=True, radius=30)
        add("R", "Right", "right", 4, 0, 3)
        add("SR", "Sharp\nright", "right2", 5, 0, 3)

        self.video = VideoView(pads, self.toggle_camera, self.toggle_focus, self._pick)
        self.video.grid(row=1, column=2, columnspan=2, sticky="nsew")

    def _build_side(self, parent):
        side = tk.Frame(parent, bg=BG, width=px(268))
        side.pack(side="right", fill="y", padx=(px(4), px(10)), pady=px(10))
        side.pack_propagate(False)
        card = tk.Frame(side, bg=SURFACE, highlightthickness=1, highlightbackground=BORDER)
        card.pack(fill="both", expand=True)
        p = px(14)

        def label(text, fg=MUTED, font=(FONT, 9), pady=0):
            lb = tk.Label(card, text=text, bg=SURFACE, fg=fg, font=font, anchor="w",
                          justify="left", wraplength=px(236))
            lb.pack(fill="x", padx=p, pady=pady)
            return lb

        tk.Label(card, text="Vision", bg=SURFACE, fg=TEXT, font=(FONT_SB, 14),
                 anchor="w").pack(fill="x", padx=p, pady=(p, px(4)))
        self.det_btn = ActionButton(card, SURFACE, self.toggle_detect, 200, 40)
        self.det_btn.pack(fill="x", padx=px(8))
        self.det_status = label("", pady=(px(2)))
        label("Detected objects (car must be stopped to choose)", pady=(px(8), px(2)))

        lb_wrap = tk.Frame(card, bg=SURFACE_2, highlightthickness=1, highlightbackground=BORDER)
        lb_wrap.pack(fill="both", expand=True, padx=p, pady=(0, px(6)))
        self.listbox = tk.Listbox(
            lb_wrap, bg=SURFACE_2, fg=TEXT, bd=0, highlightthickness=0, activestyle="none",
            font=(FONT, 11), selectbackground=mix(SURFACE_2, GREEN, 0.40), selectforeground=TEXT,
            exportselection=False, takefocus=0, height=3)
        self.listbox.pack(fill="both", expand=True, padx=px(4), pady=px(4))
        self.listbox.bind("<<ListboxSelect>>", self._list_pick)

        self.target_lbl = label("", fg=TEXT, font=(FONT_SB, 10))
        self.follow_btn = ActionButton(card, SURFACE, self.toggle_follow, 200, 44)
        self.follow_btn.pack(fill="x", padx=px(8), pady=(px(4), px(6)))

        tk.Frame(card, bg=BORDER, height=1).pack(fill="x", padx=p, pady=(px(4), px(8)))
        tk.Label(card, text="Capture", bg=SURFACE, fg=TEXT, font=(FONT_SB, 14),
                 anchor="w").pack(fill="x", padx=p, pady=(0, px(4)))
        self.photo_btn = ActionButton(card, SURFACE, self.take_photo, 200, 40)
        self.photo_btn.pack(fill="x", padx=px(8))
        self.photo_btn.configure_style("Take photo   (P)", "neutral")
        self.rec_btn = ActionButton(card, SURFACE, self.toggle_record, 200, 40)
        self.rec_btn.pack(fill="x", padx=px(8))
        self.saved_lbl = label("Files go to " + OUT_DIR, pady=(px(2), 0))
        self.saved_lbl.bind("<Button-1>", lambda e: self._open_folder())
        self.saved_lbl.config(cursor="hand2")

        # live sensor readings, fed by the telemetry stream
        tk.Frame(card, bg=BORDER, height=1).pack(fill="x", padx=p, pady=(px(8), px(6)))
        head = tk.Frame(card, bg=SURFACE)
        head.pack(fill="x", padx=p, pady=(0, px(2)))
        tk.Label(head, text="Sensors", bg=SURFACE, fg=TEXT, font=(FONT_SB, 14),
                 anchor="w").pack(side="left")
        self.sens_state = tk.Label(head, text="", bg=SURFACE, fg=MUTED, font=(FONT, 9), anchor="e")
        self.sens_state.pack(side="right")
        mono = ("Consolas", 10)
        self.sens_l = label("", fg=TEXT, font=mono)
        self.sens_r = label("", fg=TEXT, font=mono)
        self.sens_d = label("", fg=TEXT, font=mono)
        self.dist_bar = tk.Canvas(card, height=px(8), bg=SURFACE_2, highlightthickness=0, bd=0)
        self.dist_bar.pack(fill="x", padx=p, pady=(px(3), px(10)))

    def _dark_title_bar(self):
        if sys.platform != "win32":
            return
        try:
            self.root.update_idletasks()
            hwnd = ctypes.windll.user32.GetParent(self.root.winfo_id())
            one = ctypes.c_int(1)
            for attr in (20, 19):
                ctypes.windll.dwmapi.DwmSetWindowAttribute(hwnd, attr, ctypes.byref(one),
                                                           ctypes.sizeof(one))
        except Exception:
            pass

    def toggle_focus(self, force=None):
        """Full screen: video-first layout with the address bar tucked away."""
        on = (not self.focus_mode) if force is None else force
        if on == self.focus_mode:
            return
        if on and self.state != "connected":
            self.blocked()
            return
        self.focus_mode = on
        self.video.fullscreen = on
        try:
            self.root.attributes("-fullscreen", on)
        except tk.TclError:
            pass
        if on:
            self.conn.pack_forget()
        else:
            self.conn.pack(fill="x", padx=px(10), pady=(0, px(6)), before=self.body)
        self.video.kick()

    # -- connection state ------------------------------------------------------
    def _apply_state(self, mode):
        self.state = mode
        host = self.session.host if self.session else ""
        if mode == "connected":
            self.pill.set(mode, "Connected to %s" % host)
        elif mode == "connecting":
            self.pill.set(mode, "Connecting to %s" % host)
        else:
            self.pill.set(mode, "Disconnected")
        self.conn.set_mode(mode)
        self.video.set_car(mode == "connected")
        for b in self.buttons.values():
            b.set_enabled(mode == "connected")

    def _enter_pressed(self):
        if self.state == "disconnected":
            self.connect()

    def toggle_connection(self):
        if self.state == "disconnected":
            self.connect()
        elif self.state == "connecting":
            self._drop("Connection cancelled", MUTED, send_stop=False)
        else:
            self._drop("Disconnected. The car stops when the link closes.", MUTED, send_stop=True)

    def connect(self):
        host = self.conn.get_host()
        if not host:
            self.feedback.message("Enter the Raspberry Pi's IP address first", AMBER)
            self.conn.entry.focus_set()
            return
        self.session = Session(host, PORT, self.events)
        self._apply_state("connecting")
        self.feedback.message("Connecting to %s:%d" % (host, PORT), MUTED)
        self.session.start()

    def _drop(self, message, color, send_stop=False):
        self.stop_follow(send_stop=False)
        self._rec_stop()
        sess, self.session = self.session, None
        if sess is not None:
            sess.close(send_stop=send_stop)
        self.held.clear()
        self.last_cmd = None
        self.shown_seq = None
        self.pending.clear()
        self._set_target(None)
        self.toggle_focus(False)
        self._apply_state("disconnected")
        self.video.set_command(None)
        self._cam_stop()
        self._cam_idle()
        self._tele_stop()
        self._sync_side()
        self._sensor_ui()
        self.feedback.message(message, color)

    # -- camera ----------------------------------------------------------------
    def _cam_idle(self):
        if self.state != "connected":
            self.video.set_mode("idle", "Camera offline", "Connect to the car to start the live view.")
        elif not self.cam_wanted:
            self.video.set_mode("paused", "Camera paused", "Turn the camera on to resume the live view.")

    def _cam_stop(self):
        if self._cam_retry_job is not None:
            self.root.after_cancel(self._cam_retry_job)
            self._cam_retry_job = None
        cam, self.cam = self.cam, None
        self.det.cam = None
        if cam is not None:
            cam.close()
        self.video.attach(None)

    def _cam_start(self, quiet=False):
        self._cam_stop()
        if self.state != "connected" or self.session is None or not self.cam_wanted:
            self._cam_idle()
            return
        if not VIDEO_OK:
            self.video.set_mode("nolib", "Video support isn't installed",
                                "Run:  pip install av pillow   then restart the app.\n"
                                "Driving works without it.")
            return
        cam = CameraStream(self.session.host, CAM_PORT, self.cam_events)
        self.cam = cam
        self.det.cam = cam
        self.video.attach(cam)
        if not quiet:
            self.video.set_mode("connecting", "Connecting to camera",
                                "%s:%d" % (self.session.host, CAM_PORT))
        cam.start()

    def _cam_retry(self):
        self._cam_retry_job = None
        if self.state == "connected" and self.cam is None and self.cam_wanted:
            self._cam_start(quiet=True)

    def toggle_camera(self):
        self.cam_wanted = not self.cam_wanted
        self.video.cam_on = self.cam_wanted
        self.video.kick()
        if self.state == "connected" and self.cam_wanted:
            self._cam_start()
        else:
            self._cam_stop()
            self._cam_idle()

    def _handle_cam(self, ev):
        kind, cid = ev[0], ev[1]
        if self.cam is None or cid != self.cam.id:
            return                                     # stale event from an old stream
        if kind == "live":
            self.video.set_mode("live")
            return
        self._cam_stop()
        title = "Camera unavailable" if kind == "failed" else "Video stopped"
        self.video.set_mode("error", title, ev[2] + " Retrying...")
        self._cam_retry_job = self.root.after(CAM_RETRY_MS, self._cam_retry)

    # -- telemetry (sensor stream) ---------------------------------------------
    def _tele_stop(self):
        if self._tele_retry_job is not None:
            self.root.after_cancel(self._tele_retry_job)
            self._tele_retry_job = None
        tele, self.tele = self.tele, None
        self.tele_live = False
        if tele is not None:
            tele.close()

    def _tele_start(self):
        self._tele_stop()
        if self.state != "connected" or self.session is None:
            return
        self.tele = TelemetryStream(self.session.host, TELE_PORT, self.tele_events)
        self.tele.start()

    def _tele_retry(self):
        self._tele_retry_job = None
        if self.state == "connected" and self.tele is None:
            self._tele_start()

    def _handle_tele(self, ev):
        kind, tid = ev[0], ev[1]
        if self.tele is None or tid != self.tele.id:
            return                                     # stale event from an old stream
        if kind == "live":
            self.tele_live = True
            return
        self._tele_stop()                              # failed / ended: show why, then retry
        self.sens_state.config(text="offline", fg=AMBER)
        self._tele_error = ev[2]
        self._tele_retry_job = self.root.after(TELE_RETRY_MS, self._tele_retry)

    _tele_error = ""

    def sensor_view(self):
        """Fresh sensor values for the UI, or None when there is no telemetry link.
        {"enc": {"left": {...}, "right": {...}} or None (stale), "us": {"cm": float|None} or None}"""
        tele = self.tele
        if tele is None or not self.tele_live:
            return None
        snap = tele.snapshot()
        now = time.perf_counter()
        enc, us = snap["enc"], snap["us"]
        try:
            if enc is not None and now - enc["_rx"] <= TELE_STALE_S:
                enc = {"left": {k: float(enc["left"][k]) for k in ("pulses", "rev", "rpm")},
                       "right": {k: float(enc["right"][k]) for k in ("pulses", "rev", "rpm")}}
            else:
                enc = None
            if us is not None and now - us["_rx"] <= TELE_STALE_S:
                cm = us.get("cm")
                us = {"cm": float(cm) if cm is not None else None}
            else:
                us = None
        except (KeyError, TypeError, ValueError):       # malformed message: treat as no data
            return None
        return {"enc": enc, "us": us}

    def _sensor_ui(self):
        """~10 Hz refresh of the Sensors section in the side panel."""
        sv = self.sensor_view()
        if self.state != "connected":
            state, scol = "", MUTED
        elif sv is not None:
            state, scol = "live", GREEN
        elif self.tele is None:
            state, scol = "offline", AMBER
        else:
            state, scol = "connecting...", MUTED
        if self.sens_state.cget("text") != state:
            self.sens_state.config(text=state, fg=scol)
        elif str(self.sens_state.cget("fg")) != scol:
            self.sens_state.config(fg=scol)

        dash = "--"
        enc = sv["enc"] if sv else None
        if enc:
            l, r = enc["left"], enc["right"]
            tl = "Left    %5.1f rpm  %6.2f rev" % (l["rpm"], l["rev"])
            tr = "Right   %5.1f rpm  %6.2f rev" % (r["rpm"], r["rev"])
        else:
            tl, tr = "Left    %s" % dash, "Right   %s" % dash
        us = sv["us"] if sv else None
        cm, dcol = None, MUTED
        if us is None:
            td = "Distance  %s" % dash
        elif us["cm"] is None:
            td = "Distance  no echo"
        else:
            cm = us["cm"]
            dcol = distance_color(cm)
            td = "Distance  %.1f cm" % cm
        for lbl, txt, col in ((self.sens_l, tl, TEXT), (self.sens_r, tr, TEXT), (self.sens_d, td, dcol if cm is not None else TEXT)):
            if lbl.cget("text") != txt:
                lbl.config(text=txt)
            if str(lbl.cget("fg")) != col:
                lbl.config(fg=col)

        bar = self.dist_bar
        bar.delete("all")
        bw, bh = bar.winfo_width(), bar.winfo_height()
        if cm is not None and bw > 4:                   # closer = fuller bar (0..200 cm scale)
            frac = 1.0 - min(cm, 200.0) / 200.0
            bar.create_rectangle(0, 0, max(px(3), bw * frac), bh, fill=dcol, outline="")

    # -- events from the network threads ---------------------------------------
    def _poll(self):
        try:
            while True:
                self._handle(self.events.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                self._handle_cam(self.cam_events.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                self._handle_tele(self.tele_events.get_nowait())
        except queue.Empty:
            pass
        now = time.perf_counter()
        if now - self._ui_t > 0.1:
            self._ui_t = now
            self._det_ui()
            self._sensor_ui()
        self.root.after(8, self._poll)

    def _handle(self, ev):
        kind, sid = ev[0], ev[1]
        if self.session is None or sid != self.session.id:
            return                                     # stale event from an old connection
        if kind == "connected":
            self._apply_state("connected")
            save_host(self.session.host)
            self.root.focus_set()
            self.send("S", force=True)                 # known-safe start; also proves the round trip
            self._cam_start()
            self._tele_start()
        elif kind == "failed":
            self._drop("Couldn't connect. " + ev[2], RED)
        elif kind == "lost":
            self._drop("Connection lost. " + ev[2], RED)
        elif kind == "ack":
            if self.pending:
                seq, _code, t0 = self.pending.popleft()
                if seq == self.shown_seq:
                    ms = (time.perf_counter() - t0) * 1000
                    self.feedback.status("OK from car in %.0f ms" % ms, GREEN)
                    self._warned = False
        elif kind == "reply":
            if self.pending:
                self.pending.popleft()
            self.feedback.status("Car replied: " + ev[2], AMBER)

    def _watchdog(self):
        if self.state == "connected" and self.pending and not self._warned:
            if time.perf_counter() - self.pending[0][2] > ACK_WARN_AFTER:
                self._warned = True
                self.feedback.status("No reply from the car", AMBER)
        r = self.rec
        if r is not None:
            self.video.rec_secs = time.time() - r.t0
            if r.error or r.done:
                err = r.error or "Recording stopped unexpectedly."
                self._rec_stop(quiet=True)
                self.feedback.message(err, RED)
        self.root.after(250, self._watchdog)

    # -- driving ---------------------------------------------------------------
    def send(self, code, force=False):
        if self.state != "connected" or self.session is None:
            return False
        if not force and code == self.last_cmd:
            return True                                # already in effect: don't spam the car
        ok, err = self.session.send(COMMANDS[code][2] + TERMINATOR)
        if not ok:
            self._drop("Connection lost. " + err, RED)
            return False
        self.seq += 1
        self.pending.append((self.seq, code, time.perf_counter()))
        self.shown_seq = self.seq
        self.last_cmd = code
        self._warned = False
        self.feedback.show_command(code)
        return True

    def press(self, source, code):
        if self.state != "connected":
            self.blocked()
            return
        if self.follow:                                # any manual input takes over immediately
            self.stop_follow("Manual control took over", AMBER)
        if code == "S":                                # STOP always goes out, and overrides everything
            self.held.clear()
            self.buttons["S"].pulse()
            self.send("S", force=True)
            self._sync_visuals()
            return
        self.held = [h for h in self.held if h[0] != source]
        self.held.append((source, code))
        self._apply()

    def release(self, source):
        before = len(self.held)
        self.held = [h for h in self.held if h[0] != source]
        if len(self.held) != before:
            self._apply()

    def _apply(self):
        active = self.held[-1][1] if self.held else "S"
        self.send(active)
        self._sync_visuals()

    def _sync_visuals(self):
        active = self.held[-1][1] if self.held else None
        for code, b in self.buttons.items():
            if code != "S":
                b.set_pressed(code == active)
        self.video.set_command(active)

    def blocked(self):
        msg = "Still connecting" if self.state == "connecting" else "Not connected. Connect to the car to drive."
        self.feedback.message(msg, AMBER)
        self.conn.button.pulse()

    def _release_all(self):
        if self.held:
            self.held.clear()
            self._apply()

    def _is_stopped(self):
        return not self.held and self.last_cmd in (None, "S") and self.follow is None

    # -- vision: detection, target selection -----------------------------------
    def toggle_detect(self):
        det = self.det
        if not VIDEO_OK:
            self.feedback.message("Detection needs the video packages:  pip install av pillow", AMBER)
            return
        on = not det.enabled
        if on and det.status == "error":
            det.status = "idle"
        if not on:
            self.stop_follow("Detection turned off, stopped", AMBER)
            self._set_target(None)
        det.enabled = on
        self.video.det_on = on
        self._sync_side()
        self.video.kick()

    def _set_target(self, t):
        self.target = t
        self.det.target = t
        res = self.det.result
        if res is not None and t is None:
            res["sel"] = None
        self._list_sig = None

    def _pick(self, idx):
        """Choose (or un-choose) a detected object as the target. Only while the car is stopped."""
        if self.state != "connected":
            self.blocked()
            return
        if not self._is_stopped():
            self.feedback.message("Stop the car before choosing a target", AMBER)
            self._list_sig = None
            return
        res = self.det.result
        if res is None or not (0 <= idx < len(res["dets"])):
            return
        if res.get("sel") == idx:                      # click the selected one again: clear
            self._set_target(None)
        else:
            d = res["dets"][idx]
            x1, y1, x2, y2 = d["box"]
            w, h = res["w"], res["h"]
            self._set_target({"label": d["label"], "name": d["name"],
                              "cx": (x1 + x2) / 2 / w, "cy": (y1 + y2) / 2 / h})
            res["sel"] = idx
        self._list_sig = None
        self._det_ui()
        self.video.kick()

    def _list_pick(self, _e):
        sel = self.listbox.curselection()
        self.root.focus_set()                          # arrow keys must drive the car, not the list
        if sel:
            self._pick(sel[0])

    def _det_ui(self):
        """~10 Hz refresh of the side panel from the latest detections."""
        det = self.det
        res = det.result if det.enabled else None
        if not det.enabled:
            txt = det.message if det.status == "error" else "Detection is off"
        elif det.status == "loading":
            txt = "Loading model..."
        elif det.status != "ready":
            txt = "Starting..."
        elif res is None:
            txt = "Waiting for video..."
        else:
            n = len(res["dets"])
            txt = "%d object%s   %.0f ms per frame" % (n, "" if n == 1 else "s", det.ms)
        if txt != self.det_status.cget("text"):
            self.det_status.config(text=txt)

        dets = res["dets"] if res else []
        sel = res.get("sel") if res else None
        sig = tuple((d["name"], self._tag(d, res)) for d in dets) + (sel,)
        if sig != self._list_sig:
            self._list_sig = sig
            self.listbox.delete(0, "end")
            for d in dets:
                self.listbox.insert("end", "%s     %s" % (d["name"], self._tag(d, res)))
            if sel is not None:
                self.listbox.selection_set(sel)
                self.listbox.see(sel)
        self._sync_side()

    @staticmethod
    def _tag(d, res):
        x1, _y1, x2, _y2 = d["box"]
        e = ((x1 + x2) / 2 - res["w"] / 2) / (res["w"] / 2)
        return "left" if e < -DEADZONE else ("right" if e > DEADZONE else "centre")

    def _sync_side(self):
        det = self.det
        self.det_btn.configure_style("Detection on" if det.enabled else "Detection off",
                                     "primary" if det.enabled else "neutral")
        res = det.result if det.enabled else None
        visible = res is not None and res.get("sel") is not None
        if self.follow:
            txt, col = "Going to: %s" % self.target["name"], GREEN
        elif self.target:
            txt = "Selected: %s%s" % (self.target["label"], "" if visible else "  (not in view)")
            col = TEXT if visible else AMBER
        else:
            txt, col = "Nothing selected", MUTED
        if txt != self.target_lbl.cget("text"):
            self.target_lbl.config(text=txt, fg=col)
        elif str(self.target_lbl.cget("fg")) != col:
            self.target_lbl.config(fg=col)
        if self.follow:
            self.follow_btn.configure_style("Stop following   (G)", "danger")
        else:
            self.follow_btn.configure_style("Go to selected   (G)", "primary")
        self.rec_btn.configure_style("Stop recording   (R)" if self.rec else "Record video   (R)",
                                     "danger" if self.rec else "neutral")

    # -- vision: go to the selected object (visual feedback loop) --------------
    def toggle_follow(self):
        if self.follow:
            self.stop_follow("Stopped going to the object", MUTED)
        else:
            self.start_follow()

    def start_follow(self):
        if self.follow:
            return
        if self.state != "connected":
            self.blocked()
            return
        det, res = self.det, self.det.result
        if not self._is_stopped():
            msg = "Stop the car first, then press Go"
        elif self.target is None:
            msg = "Pick an object from the list first"
        elif not det.enabled or det.status != "ready":
            msg = "Turn detection on and wait for the model to load"
        elif self.video.mode != "live":
            msg = "The camera isn't live"
        elif res is None or res.get("sel") is None:
            msg = "The selected object isn't in view"
        else:
            msg = ""
        if msg:
            self.feedback.message(msg, AMBER)
            return
        now = time.perf_counter()
        self.follow = {"t0": now, "ready_at": 0.0, "seen": -1, "misses": 0, "wait_since": now,
                       "cam_lost": None, "job": None}
        self.video.banner = ("Going to %s" % self.target["name"], GREEN)
        self.feedback.message("Going to %s" % self.target["name"], GREEN)
        self._sync_side()
        self._follow_tick()

    def stop_follow(self, msg=None, color=MUTED, send_stop=True):
        f, self.follow = self.follow, None
        if f is None:
            return
        if f["job"] is not None:
            self.root.after_cancel(f["job"])
        self.video.banner = None
        self.video.set_command(None)
        if send_stop and self.state == "connected":
            self.send("S", force=True)
        self._sync_side()
        if msg:
            self.feedback.message(msg, color)

    def _follow_tick(self):
        """Waiting phase: look for a detection from a frame captured AFTER the car settled."""
        f = self.follow
        if f is None:
            return
        f["job"] = None
        now = time.perf_counter()
        if now - f["t0"] > FOLLOW_TIMEOUT_S:
            self.stop_follow("Stopped: time limit reached", AMBER)
            return
        if self.video.mode != "live":
            f["cam_lost"] = f["cam_lost"] or now
            if now - f["cam_lost"] > 1.5:
                self.stop_follow("Stopped: lost the camera picture", RED)
                return
        else:
            f["cam_lost"] = None
        res = self.det.result
        if res is not None and res["ver"] != f["seen"] and res["ts"] > f["ready_at"]:
            f["seen"], f["wait_since"] = res["ver"], now
            if self._follow_decide(res):
                return                                   # a pulse started, or we stopped
        elif now - f["wait_since"] > FOLLOW_STALL_S:
            self.stop_follow("Stopped: no fresh video / detections", RED)
            return
        f["job"] = self.root.after(30, self._follow_tick)

    def _follow_decide(self, res):
        """Compare the object's centre x with the image centre and fire one short pulse.
        Returns True if a pulse was started or following ended."""
        f = self.follow
        idx = res.get("sel")
        if idx is None:
            f["misses"] += 1
            if f["misses"] >= FOLLOW_MAX_MISSES:
                self.stop_follow("Stopped: lost sight of the object", AMBER)
                return True
            return False
        f["misses"] = 0
        x1, y1, x2, y2 = res["dets"][idx]["box"]
        w, h = res["w"], res["h"]
        near_vert = y1 <= EDGE_MARGIN * h or y2 >= h - EDGE_MARGIN * h
        fills_width = x1 <= EDGE_MARGIN * w and x2 >= w - EDGE_MARGIN * w
        if near_vert or fills_width or max((x2 - x1) / w, (y2 - y1) / h) >= ARRIVE_FRAC:
            self.stop_follow("Arrived at %s" % self.target["name"], GREEN)
            return True
        err = ((x1 + x2) / 2 - w / 2) / (w / 2)         # -1 (far left) .. +1 (far right)
        if err < -DEADZONE:
            self._pulse("AL", TURN_PULSE_S)
        elif err > DEADZONE:
            self._pulse("AR", TURN_PULSE_S)
        else:
            self._pulse("AF", FWD_PULSE_S)
        return True

    def _pulse(self, code, secs):
        f = self.follow
        if not self.send(code, force=True):
            return                                       # link lost: _drop already cleaned up
        self.video.set_command(code)
        f["job"] = self.root.after(int(secs * 1000), self._pulse_end)

    def _pulse_end(self):
        f = self.follow
        if f is None:
            return
        f["job"] = None
        if not self.send("S", force=True):
            return
        self.video.set_command(None)
        now = time.perf_counter()
        f["ready_at"] = now + SETTLE_S                   # only frames captured after this count
        f["wait_since"] = now
        f["job"] = self.root.after(30, self._follow_tick)

    # -- capture: photo and video ----------------------------------------------
    def _live_frame(self):
        cam = self.cam
        if cam is None or self.video.mode != "live":
            return None
        return cam.latest()[1]

    def take_photo(self):
        img = self._live_frame()
        if img is None:
            self.feedback.message("No live video to take a photo of", AMBER)
            return
        try:
            path = out_path("photo", ".jpg")
            img.save(path, quality=95)
        except Exception as exc:
            self.feedback.message("Couldn't save the photo: %s" % exc, RED)
            return
        self.video.pulse()
        self.saved_lbl.config(text="Photo saved: " + os.path.basename(path))
        self.feedback.message("Photo saved: " + os.path.basename(path), GREEN)

    def toggle_record(self):
        if self.rec:
            self._rec_stop()
            return
        if self._live_frame() is None:
            self.feedback.message("No live video to record", AMBER)
            return
        try:
            path = out_path("video", ".mp4")
        except OSError as exc:
            self.feedback.message("Can't create the output folder: %s" % exc, RED)
            return
        self.rec = Recorder(lambda: self.cam, path)
        self.rec.start()
        self.video.rec_secs = 0.0
        self.saved_lbl.config(text="Recording to " + os.path.basename(path))
        self._sync_side()

    def _rec_stop(self, quiet=False):
        r, self.rec = self.rec, None
        if r is None:
            return
        r.stop()
        self.video.rec_secs = None
        if not r.error:
            self.saved_lbl.config(text="Video saved: " + os.path.basename(r.path))
            if not quiet:
                self.feedback.message("Video saved: " + os.path.basename(r.path), GREEN)
        self._sync_side()

    def _open_folder(self):
        try:
            os.makedirs(OUT_DIR, exist_ok=True)
            if sys.platform == "win32":
                os.startfile(OUT_DIR)
        except Exception:
            pass

    # -- keyboard --------------------------------------------------------------
    def _key_down(self, e):
        if isinstance(self.root.focus_get(), tk.Entry):
            return
        k = e.keysym.lower()
        hot = HOTKEYS.get(k)
        if hot:
            if k not in self._keys_down:                 # one action per physical key press
                self._keys_down.add(k)
                {"photo": self.take_photo, "record": self.toggle_record,
                 "follow": self.toggle_follow}[hot]()
            return
        code = KEYMAP.get(k)
        if code is None:
            return
        job = self._release_jobs.pop(k, None)
        if job:                                        # release was just an auto-repeat artefact
            self.root.after_cancel(job)
            return
        if k in self._keys_down:                       # auto-repeat: ignore
            return
        self._keys_down.add(k)
        self.press(("key", k), code)

    def _key_up(self, e):
        k = e.keysym.lower()
        if k not in self._keys_down:
            return
        if KEY_DEBOUNCE_MS:
            self._release_jobs[k] = self.root.after(KEY_DEBOUNCE_MS, lambda: self._finish_key(k))
        else:
            self._finish_key(k)

    def _finish_key(self, k):
        self._release_jobs.pop(k, None)
        self._keys_down.discard(k)
        self.release(("key", k))

    def _check_focus(self):
        if self.root.focus_displayof() is None:        # window lost focus: never keep driving blind
            self.stop_follow("Stopped: the window lost focus", AMBER)
            self._release_all()

    def _focus_in(self, e):
        if str(e.widget) == ".":
            self._keys_down.clear()

    # -- shutdown --------------------------------------------------------------
    def _on_close(self):
        self.stop_follow(send_stop=True)
        self._rec_stop(quiet=True)
        self.det.stop()
        self._cam_stop()
        self._tele_stop()
        if self.session is not None:
            self.session.close(send_stop=True)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    App().run()