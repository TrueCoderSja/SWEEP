#!/usr/bin/env python3
"""
RC Controller - manual drive console for the Raspberry Pi RC car, with live camera view.

Windows, Python 3.8+.
Drive controls use only the standard library (Tkinter). The live video needs two packages:

    pip install av pillow

(without them the app still drives the car and shows a hint where the video would be)

Run:  python rc_controller.py

Control protocol (TCP, port 6000): the app sends one text command per line
    F  B  L  R  SL  SR  S
and the car's server answers "OK". If the connection drops, the server stops the motors.

Camera (TCP, port 5000): the Pi runs
    rpicam-vid -t 0 --width 640 --height 480 --framerate 30 --codec h264 --inline \
               --intra 1 --bitrate 2000000 --flush --listen -o tcp://0.0.0.0:5000
This app connects to it after the car connection is up and decodes the raw H.264 itself
(no buffering, so latency stays low).

Driving model: HOLD a control to drive, RELEASE to stop (a stop is sent automatically).
STOP is a single press. Keyboard: W/A/S/D or arrows, Q/E = sharp turns, Space = STOP,
F11 = full screen (double-click the video does the same).
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

CAM_PORT = 5000            # rpicam-vid --listen port (same Pi as the control server)
CAM_CONNECT_TIMEOUT = 4.0
CAM_READ_TIMEOUT = 4.0     # no data for this long = stream considered dead
CAM_RETRY_MS = 2000        # how often to retry while the car link is up
CAM_ROTATE = 0             # 0, 90, 180 or 270 if the camera is mounted rotated

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
GREEN = "#38D996"    # connected

FONT = "Segoe UI"
FONT_SB = "Segoe UI Semibold"

COMMANDS = {
    "F": ("Forward", BLUE),
    "B": ("Reverse", BLUE),
    "L": ("Left", TEAL),
    "R": ("Right", TEAL),
    "SL": ("Sharp left", AMBER),
    "SR": ("Sharp right", AMBER),
    "S": ("STOP", RED),
}
KEYMAP = {
    "w": "F", "up": "F", "s": "B", "down": "B",
    "a": "L", "left": "L", "d": "R", "right": "R",
    "q": "SL", "e": "SR", "space": "S", "escape": "S",
}
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
#  Camera: raw H.264 over TCP, decoded on a background thread
# --------------------------------------------------------------------------- #
class CameraStream:
    """Connects to rpicam-vid's TCP listener, decodes H.264 and keeps only the newest frame
    (a slow UI can never build up a backlog, so the picture never lags behind).
    Events go to `events` as (kind, stream_id, ...): "live", "failed", "ended"."""

    _ids = itertools.count(1)

    def __init__(self, host, port, events):
        self.id = next(CameraStream._ids)
        self.host, self.port, self.events = host, port, events
        self.sock = None
        self.cancelled = False
        self.lock = threading.Lock()
        self._frame, self._ver = None, 0
        self.fps = 0.0
        self.size = (0, 0)

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def latest(self):
        with self.lock:
            return self._ver, self._frame

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
                    self._frame, self._ver = img, self._ver + 1
                if not live:
                    live = True
                    self._post("live")
        except Exception as exc:
            reason = "Video error: %s" % exc
        finally:
            _quiet_close(sock)
        self._post("ended", reason)


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
    """The live camera panel: rounded video with a HUD (LIVE, fps, camera / full-screen
    toggles, current drive command) and a glowing border in the colour of the active command.
    Shows a clear status card whenever there is no picture."""

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

    def __init__(self, master, on_toggle, on_fullscreen):
        super().__init__(master, BG, width=1, height=1)
        self.on_toggle, self.on_fullscreen = on_toggle, on_fullscreen
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

        self._ver, self._img = -1, None
        self._ver_drawn, self._photo, self._photo_key = -2, None, None
        self._mask, self._mask_key = None, None
        self._hits, self._hover = [], None

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

    def _set_hover(self, name):
        if name != self._hover:
            self._hover = name
            self.config(cursor="hand2" if name else "arrow")
            self.kick()

    def _motion(self, e):
        self._set_hover(self._hit_at(e.x, e.y))

    def _click(self, e):
        self.focus_set()
        name = self._hit_at(e.x, e.y)
        if name == "cam":
            self.on_toggle()
        elif name == "fs":
            self.on_fullscreen()

    def _double(self, e):
        if self._hit_at(e.x, e.y) is None:
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
            ver, img = self.stream.latest()
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

    # -- drawing ---------------------------------------------------------------
    def _draw(self):
        self.delete("all")
        self._hits = []
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
        else:
            self._placeholder(x1, y1, x2, y2)

        border = mix(BORDER, acc, 0.85 * drive)
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
        # top-left: LIVE + stream stats
        _, lx2 = self._pill(x1 + px(18), ty, "LIVE", self.f_pill, TEXT, self.CHIP_BG,
                            self.CHIP_BORDER, anchor="w", dot=RED, pulse=True)
        st = self.stream
        if st is not None and st.size[0]:
            txt = "%dx%d" % st.size
            if st.fps >= 1:
                txt = "%d fps   %s" % (round(st.fps), txt)
            self._pill(lx2 + px(8), ty, txt, self.f_pill, MUTED, self.CHIP_BG, self.CHIP_BORDER)
        # bottom-centre: what the car is doing right now
        if self.car_on:
            code = self.cmd or "S"
            name, color = COMMANDS[code]
            driving = code != "S"
            self._pill((x1 + x2) / 2, y2 - px(30), name if driving else "Stopped", self.f_pill, TEXT,
                       mix(VIDEO_BG, color, 0.42) if driving else self.CHIP_BG,
                       color if driving else self.CHIP_BORDER, anchor="c", dot=color)


class ActionButton(AnimCanvas):
    """Pill button for Connect / Cancel / Disconnect."""

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
        name, color = COMMANDS[code]
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
        w0, h0 = min(px(1280), int(sw * 0.92)), min(px(860), int(sh * 0.90))
        root.geometry("%dx%d+%d+%d" % (w0, h0, (sw - w0) // 2, max(0, (sh - h0) // 2 - px(20))))
        root.minsize(min(px(980), w0), min(px(700), h0))

        self.events = queue.Queue()
        self.cam_events = queue.Queue()
        self.session = None
        self.cam = None
        self.cam_wanted = True
        self.focus_mode = False
        self._cam_retry_job = None
        self.state = "disconnected"
        self.held = []                 # [(source, code)], most recent last
        self.last_cmd = None
        self.seq = 0
        self.shown_seq = None
        self.pending = deque(maxlen=64)
        self._warned = False
        self._keys_down = set()
        self._release_jobs = {}

        self._build()
        self._apply_state("disconnected")
        self._cam_idle()
        self.feedback.message("Enter the Pi's IP address and connect", MUTED)

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
                 text="Hold a control to drive, release to stop.   Keys: W A S D or arrows, "
                      "Q and E for sharp turns, Space to stop, F11 for full screen."
                 ).pack(side="bottom", pady=(px(8), 0))
        self.feedback = FeedbackBar(main)
        self.feedback.pack(side="bottom", fill="x", padx=px(10), pady=(px(6), 0))

        self.pads = pads = tk.Frame(main, bg=BG)
        pads.pack(fill="both", expand=True)
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

        self.video = VideoView(pads, self.toggle_camera, self.toggle_focus)
        self.video.grid(row=1, column=2, columnspan=2, sticky="nsew")

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
            self.conn.pack(fill="x", padx=px(10), pady=(0, px(6)), before=self.pads)
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
        sess, self.session = self.session, None
        if sess is not None:
            sess.close(send_stop=send_stop)
        self.held.clear()
        self.last_cmd = None
        self.shown_seq = None
        self.pending.clear()
        self.toggle_focus(False)
        self._apply_state("disconnected")
        self.video.set_command(None)
        self._cam_stop()
        self._cam_idle()
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
        self.root.after(250, self._watchdog)

    # -- driving ---------------------------------------------------------------
    def send(self, code, force=False):
        if self.state != "connected" or self.session is None:
            return False
        if not force and code == self.last_cmd:
            return True                                # already in effect: don't spam the car
        ok, err = self.session.send(code + TERMINATOR)
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

    # -- keyboard --------------------------------------------------------------
    def _key_down(self, e):
        if isinstance(self.root.focus_get(), tk.Entry):
            return
        k = e.keysym.lower()
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
            self._release_all()

    def _focus_in(self, e):
        if str(e.widget) == ".":
            self._keys_down.clear()

    # -- shutdown --------------------------------------------------------------
    def _on_close(self):
        self._cam_stop()
        if self.session is not None:
            self.session.close(send_stop=True)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    App().run()