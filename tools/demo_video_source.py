#!/usr/bin/env python3
"""
Demo video source service using MAVLink + FFmpeg.

Features:
- Sends HEARTBEAT at 1 Hz.
- Supervises an FFmpeg video stream and restarts it on failure. The pixel
  source is selectable: a synthetic test pattern (--video-source test,
  default, unchanged) or bare RTP/UDP H.264 passthrough from Gazebo's
  GstCameraPlugin (--video-source rtp).
- In --video-source=rtp mode only, cooperates with the Gazebo render gate
  (gazebo/docker/camera_gate.py) to break a bootstrap deadlock: that gate
  enables the Gazebo camera only once MediaMTX reports a reader on this
  service's RTSP path, but MediaMTX never reports a path -- let alone a
  reader -- that has no active publisher, and this service can't publish
  real RTP until the (still-disabled) camera is enabled. Nothing would ever
  start. The fix: always be publishing *something*, so the path always
  exists and a viewer can always attach. This service polls MediaMTX itself
  (same signal camera_gate.py uses, polled independently) and switches its
  own FFmpeg input between a cheap placeholder pattern (0 readers, or the
  real feed hasn't shown up yet) and the real RTP passthrough (1+ readers,
  once frames actually arrive) -- see _gate_worker(). Do not delete the
  placeholder path thinking it's dead code; without it nothing ever renders.
- Handles MAV_CMD_REQUEST_MESSAGE for camera-related MAVLink messages:
  CAMERA_INFORMATION
  CAMERA_SETTINGS
  VIDEO_STREAM_STATUS
  VIDEO_STREAM_INFORMATION
  CAMERA_CAPTURE_STATUS
  STORAGE_INFORMATION
  CAMERA_IMAGE_CAPTURED (param2 = image index)
- Full MAVLink Camera Protocol v2 component (see MavlinkCameraMixin):
  photo/video modes (MAV_CMD_SET_CAMERA_MODE), single and interval photo
  capture (MAV_CMD_IMAGE_START/STOP_CAPTURE, MAV_CMD_DO_DIGICAM_CONTROL),
  video recording (MAV_CMD_VIDEO_START/STOP_CAPTURE), storage
  (MAV_CMD_REQUEST_STORAGE_INFORMATION, MAV_CMD_STORAGE_FORMAT) and camera
  settings over the PARAM_EXT protocol, described to the GCS by a camera
  definition XML served over HTTP (CAMERA_INFORMATION.cam_definition_uri).
  Photos and videos are grabbed from --rtsp-uri (the live stream) by ffmpeg,
  with the settings applied, and stored under --storage-dir; each photo is
  reported with CAMERA_IMAGE_CAPTURED, geotagged from the autopilot's
  GLOBAL_POSITION_INT / ATTITUDE, with a file_url the GCS can download.
"""

import argparse
import base64
import http.server
import json
import math
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import zlib
from pathlib import Path
from urllib.parse import unquote, urlsplit
from xml.sax.saxutils import escape as xml_escape

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil


class _MavlinkUdpSender:
    """File-like adapter so pymavlink's MAVLink encoder writes frames to a UDP socket."""

    def __init__(self, sock: socket.socket, addr: tuple[str, int]) -> None:
        self._sock = sock
        self._addr = addr

    def write(self, data: bytes) -> None:
        self._sock.sendto(data, self._addr)


VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
VIDEO_NAME = "DFDEMO"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 24042
DEFAULT_SYSTEM_ID = 1
DEFAULT_COMPONENT_ID = 104
DEFAULT_HEARTBEAT_PERIOD = 1.0
DEFAULT_STREAM_ID = 0
DEFAULT_BITRATE_BPS = 2_000_000
DEFAULT_VIDEO_CODEC = "h264"
DEFAULT_FFMPEG_PKT_SIZE = 1400
DEFAULT_RTSP_PORT = 18554
DEFAULT_RTSP_MOUNT = "/demo"
DEFAULT_VIDEO_SOURCE = "test"
DEFAULT_RTP_PORT = 5600
DEFAULT_CAMERA_VENDOR = "DRAGANFLY"
DEFAULT_CAMERA_MODEL = "DF DEMO CAMERA"
DEFAULT_OVERLAY_FONT = "DejaVu Sans"

# Bootstrap placeholder / render-gate polling, only meaningful when
# --video-source=rtp (see the module docstring for why this exists).
DEFAULT_PLACEHOLDER_WIDTH = 320
DEFAULT_PLACEHOLDER_HEIGHT = 240
DEFAULT_PLACEHOLDER_FPS = 5
DEFAULT_RTP_GRACE_PERIOD = 8.0
DEFAULT_RTP_REVERT_HOLD = 10.0
DEFAULT_GATE_POLL_PERIOD = 2.0
DEFAULT_GATE_POLL_TIMEOUT = 2.0
DEFAULT_MEDIASERVER_API_PORT = 9997


def _default_rtsp_uri(host: str) -> str:
    return f"rtsp://{host}:{DEFAULT_RTSP_PORT}{DEFAULT_RTSP_MOUNT}"


def _default_rtp_sdp_path(rtp_port: int) -> str:
    return f"/tmp/demo_video_source_rtp_{rtp_port}.sdp"


def _overlay_textfile_path(component_id: int) -> str:
    return f"/tmp/demo_video_source_overlay_{component_id}.txt"


def _safe_mavlink_const(name: str, default: int = 0) -> int:
    return int(getattr(mavutil.mavlink, name, default))


CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM = _safe_mavlink_const("CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM")
VIDEO_STREAM_STATUS_FLAGS_RUNNING = _safe_mavlink_const("VIDEO_STREAM_STATUS_FLAGS_RUNNING")
VIDEO_STREAM_TYPE_RTSP = _safe_mavlink_const("VIDEO_STREAM_TYPE_RTSP")


# ---------------------------------------------------------------------------
# MAVLink Camera Protocol v2: modes, photo/video capture, storage, settings.
# https://mavlink.io/en/services/camera.html
# ---------------------------------------------------------------------------

CAMERA_CAP_FLAGS = (
    _safe_mavlink_const("CAMERA_CAP_FLAGS_CAPTURE_VIDEO", 1)
    | _safe_mavlink_const("CAMERA_CAP_FLAGS_CAPTURE_IMAGE", 2)
    | _safe_mavlink_const("CAMERA_CAP_FLAGS_HAS_MODES", 4)
    | _safe_mavlink_const("CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE", 8)
    | _safe_mavlink_const("CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE", 16)
    | CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
)
CAMERA_MODE_IMAGE = _safe_mavlink_const("CAMERA_MODE_IMAGE", 0)
CAMERA_MODE_VIDEO = _safe_mavlink_const("CAMERA_MODE_VIDEO", 1)

PARAM_EXT_TYPE_UINT32 = _safe_mavlink_const("MAV_PARAM_EXT_TYPE_UINT32", 5)
PARAM_EXT_TYPE_REAL32 = _safe_mavlink_const("MAV_PARAM_EXT_TYPE_REAL32", 9)
PARAM_ACK_ACCEPTED = _safe_mavlink_const("PARAM_ACK_ACCEPTED", 0)
PARAM_ACK_VALUE_UNSUPPORTED = _safe_mavlink_const("PARAM_ACK_VALUE_UNSUPPORTED", 1)
PARAM_ACK_FAILED = _safe_mavlink_const("PARAM_ACK_FAILED", 2)

# image_status values of CAMERA_CAPTURE_STATUS.
IMAGE_STATUS_IDLE = 0
IMAGE_STATUS_CAPTURING = 1
IMAGE_STATUS_INTERVAL_IDLE = 2
IMAGE_STATUS_INTERVAL_CAPTURING = 3

DEFAULT_STORAGE_DIR = "/data"
DEFAULT_HTTP_PORT = 8090
DEFAULT_CAPTURE_TIMEOUT = 45.0
# Seconds within which a repeated IMAGE_START_CAPTURE sequence number is
# treated as a retransmission rather than a new request.
CAPTURE_RETRANSMIT_WINDOW = 10.0
DEFAULT_CAMERA_PITCH_DEG = 0.0
# Optics reported in CAMERA_INFORMATION (used by QGC's survey planner). The
# horizontal FOV matches models/iris_dual_cam's <horizontal_fov>1.5</...>; the
# sensor is a nominal 1/2.3" (6.17 mm wide) with the stream's aspect ratio.
DEFAULT_HFOV_DEG = math.degrees(1.5)
SENSOR_WIDTH_MM = 6.17

# Resolution options shared by CAM_PHOTORES and CAM_VIDRES; None = native.
_RESOLUTIONS = {0: None, 1: (960, 540), 2: (640, 360)}
_RESOLUTION_OPTIONS = [(f"{VIDEO_WIDTH}x{VIDEO_HEIGHT}", 0), ("960x540", 1), ("640x360", 2)]
_JPEG_QSCALE = {0: 2, 1: 5, 2: 10}
# Light-source colour temperature simulated per white-balance preset.
_WB_KELVIN = {1: 5500, 2: 7000, 3: 3200, 4: 4000}


class CameraSetting:
    """One camera parameter: exposed through PARAM_EXT and the camera definition."""

    def __init__(self, name, ptype, default, description, options, control=True):
        self.name = name
        self.ptype = ptype  # "uint32" or "float" (camera definition type names)
        self.default = default
        self.description = description
        self.options = options  # [(label, value), ...]
        self.control = control  # False: not shown as a setting (CAM_MODE)

    @property
    def mav_type(self) -> int:
        return PARAM_EXT_TYPE_REAL32 if self.ptype == "float" else PARAM_EXT_TYPE_UINT32

    def coerce(self, value):
        return float(value) if self.ptype == "float" else int(round(value))

    def encode(self, value) -> bytes:
        if self.ptype == "float":
            return struct.pack("<f", float(value))
        return struct.pack("<I", int(value))

    def decode(self, raw: bytes):
        raw = raw.ljust(4, b"\0")
        if self.ptype == "float":
            return struct.unpack("<f", raw[:4])[0]
        return struct.unpack("<I", raw[:4])[0]

    def is_valid(self, value) -> bool:
        return any(abs(float(value) - float(v)) < 1e-4 for _, v in self.options)

    def format(self, value) -> str:
        return repr(float(value)) if self.ptype == "float" else str(int(value))


CAMERA_SETTINGS = [
    CameraSetting(
        "CAM_MODE", "uint32", CAMERA_MODE_IMAGE, "Camera Mode",
        [("Photo", CAMERA_MODE_IMAGE), ("Video", CAMERA_MODE_VIDEO)],
        control=False,
    ),
    CameraSetting(
        "CAM_EV", "float", 0.0, "Exposure Compensation",
        [(f"{v:+.1f}" if v else "0", v) for v in (-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0)],
    ),
    CameraSetting(
        "CAM_WBMODE", "uint32", 0, "White Balance",
        [("Auto", 0), ("Sunny", 1), ("Cloudy", 2), ("Tungsten", 3), ("Fluorescent", 4)],
    ),
    CameraSetting(
        "CAM_COLORMODE", "uint32", 0, "Color Mode",
        [("Natural", 0), ("Vivid", 1), ("Black & White", 2)],
    ),
    CameraSetting("CAM_PHOTORES", "uint32", 0, "Photo Resolution", _RESOLUTION_OPTIONS),
    CameraSetting(
        "CAM_PHOTOQUAL", "uint32", 0, "Photo Quality",
        [("Fine", 0), ("Normal", 1), ("Basic", 2)],
    ),
    CameraSetting("CAM_VIDRES", "uint32", 0, "Video Resolution", _RESOLUTION_OPTIONS),
]


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _quat_mul(a: list[float], b: list[float]) -> list[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


class MavlinkCameraMixin:
    """Camera Protocol v2 behaviour for DemoVideoSource.

    Expects the host class to provide: _args, _mav, _send_lock, _stop_event,
    _rtsp_uri, _notif_uri, _time_boot_ms(), _send_command_ack().

    Photos/videos are cut from the live stream (--rtsp-uri, i.e. MediaMTX),
    so pulling one is also what wakes a render-gated Gazebo camera; the first
    capture after an idle period therefore waits for Gazebo's first frame.
    The settings change what is captured (exposure, white balance, colour,
    resolution, JPEG quality), not the live stream, which is passed through
    untouched.
    """

    # ----------------------------------------------------------- setup ---
    def _camera_init(self) -> None:
        args = self._args
        self._storage_dir = Path(args.storage_dir)
        self._images_dir = self._storage_dir / "images"
        self._videos_dir = self._storage_dir / "videos"
        self._images_dir.mkdir(parents=True, exist_ok=True)
        self._videos_dir.mkdir(parents=True, exist_ok=True)
        self._settings_file = self._storage_dir / "settings.json"

        self._setting_defs = {s.name: s for s in CAMERA_SETTINGS}
        self._settings = {s.name: s.default for s in CAMERA_SETTINGS}
        self._load_settings()

        self._camera_lock = threading.RLock()
        self._camera_mode = int(self._settings["CAM_MODE"])
        self._image_count = self._scan_image_count()
        self._image_status = IMAGE_STATUS_IDLE
        self._image_interval = 0.0
        self._capture_thread: threading.Thread | None = None
        self._capture_stop = threading.Event()
        self._last_capture_sequence: tuple[int, float] | None = None  # (seq, monotonic)
        self._video_process: subprocess.Popen | None = None
        self._video_started = 0.0
        self._video_path: Path | None = None

        # Latest autopilot telemetry, used to geotag photos.
        self._vehicle_position = None
        self._vehicle_attitude = None
        self._mount_quat = _euler_to_quat(0.0, math.radians(args.camera_pitch_deg), 0.0)

        advertise_host = (
            args.http_advertise_host or urlsplit(self._notif_uri).hostname or "127.0.0.1"
        )
        advertise_port = args.http_advertise_port or args.http_port
        self._http_base = f"http://{advertise_host}:{advertise_port}"
        self._definition_xml, self._definition_version = self._build_camera_definition()
        self._http_server: http.server.ThreadingHTTPServer | None = None

    def _load_settings(self) -> None:
        try:
            saved = json.loads(self._settings_file.read_text())
        except (OSError, ValueError):
            return
        for name, value in saved.items():
            setting = self._setting_defs.get(name)
            if setting is not None and setting.is_valid(value):
                self._settings[name] = setting.coerce(value)

    def _save_settings(self) -> None:
        try:
            self._settings_file.write_text(json.dumps(self._settings, indent=2))
        except OSError as exc:
            print(f"Could not save camera settings: {exc}")

    def _scan_image_count(self) -> int:
        highest = -1
        for path in self._images_dir.glob("IMG_*.*"):
            try:
                highest = max(highest, int(path.stem.split("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        return highest + 1

    def _build_camera_definition(self) -> tuple[bytes, int]:
        lines = [
            '<?xml version="1.0" encoding="UTF-8" ?>',
            "<mavlinkcamera>",
            '  <definition version="{version}">',
            f"    <model>{xml_escape(self._args.camera_model)}</model>",
            f"    <vendor>{xml_escape(self._args.camera_vendor)}</vendor>",
            "  </definition>",
            "  <parameters>",
        ]
        for s in CAMERA_SETTINGS:
            control = "" if s.control else ' control="0"'
            lines.append(
                f'    <parameter name="{s.name}" type="{s.ptype}" '
                f'default="{s.format(s.default)}"{control}>'
            )
            lines.append(f"      <description>{xml_escape(s.description)}</description>")
            lines.append("      <options>")
            for label, value in s.options:
                lines.append(
                    f'        <option name="{xml_escape(label, {chr(34): "&quot;"})}" '
                    f'value="{s.format(value)}" />'
                )
            lines.append("      </options>")
            lines.append("    </parameter>")
        lines += ["  </parameters>", "</mavlinkcamera>", ""]
        template = "\n".join(lines)
        # The GCS caches definitions by model + version, so the version must
        # change whenever the content does: derive it from the content.
        version = zlib.crc32(template.encode("utf-8")) % 65535 + 1
        return template.replace("{version}", str(version)).encode("utf-8"), version

    @property
    def _definition_uri(self) -> str:
        return f"{self._http_base}/camera.xml"

    # ------------------------------------------------------------ HTTP ---
    def _start_http_server(self) -> None:
        camera = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802 (http.server API)
                camera._serve_http(self, send_body=True)

            def do_HEAD(self):  # noqa: N802
                camera._serve_http(self, send_body=False)

            def log_message(self, fmt, *fmt_args):
                pass

        try:
            server = http.server.ThreadingHTTPServer(("0.0.0.0", self._args.http_port), Handler)
        except OSError as exc:
            print(f"Camera HTTP server failed to start on :{self._args.http_port}: {exc}")
            return
        server.daemon_threads = True
        self._http_server = server
        threading.Thread(target=server.serve_forever, name="http-server", daemon=True).start()
        print(
            f"Camera HTTP server on :{self._args.http_port} "
            f"(advertised as {self._http_base}, definition v{self._definition_version})"
        )

    def _serve_http(self, handler, send_body: bool) -> None:
        path = unquote(urlsplit(handler.path).path)
        body = None
        file_path = None
        content_type = "application/octet-stream"
        if path == "/camera.xml":
            body = self._definition_xml
            content_type = "application/xml"
        else:
            parts = path.strip("/").split("/")
            if (
                len(parts) == 2
                and parts[0] in ("images", "videos")
                and parts[1]
                and not parts[1].startswith(".")
            ):
                candidate = self._storage_dir / parts[0] / parts[1]
                if candidate.is_file():
                    file_path = candidate
                    content_type = {
                        ".jpg": "image/jpeg",
                        ".json": "application/json",
                        ".mp4": "video/mp4",
                    }.get(candidate.suffix.lower(), content_type)

        if body is None and file_path is None:
            handler.send_error(404)
            return

        length = len(body) if body is not None else file_path.stat().st_size
        handler.send_response(200)
        handler.send_header("Content-Type", content_type)
        handler.send_header("Content-Length", str(length))
        handler.end_headers()
        if not send_body:
            return
        try:
            if body is not None:
                handler.wfile.write(body)
            else:
                with file_path.open("rb") as f:
                    shutil.copyfileobj(f, handler.wfile)
        except (BrokenPipeError, ConnectionResetError):
            pass

    # ----------------------------------------------------- telemetry ---
    def _handle_camera_message(self, msg) -> bool:
        """Non-command messages: telemetry for geotagging and PARAM_EXT."""
        msg_type = msg.get_type()
        if msg_type == "GLOBAL_POSITION_INT":
            if msg.get_srcSystem() == self._args.system_id:
                self._vehicle_position = msg
            return True
        if msg_type == "ATTITUDE":
            if msg.get_srcSystem() == self._args.system_id:
                self._vehicle_attitude = msg
            return True
        if msg_type.startswith("PARAM_EXT_"):
            if int(getattr(msg, "target_system", 0)) not in (0, self._args.system_id):
                return True
            if int(getattr(msg, "target_component", 0)) not in (0, self._args.component_id):
                return True
            self._handle_param_ext(msg)
            return True
        return False

    # ------------------------------------------------------- PARAM_EXT ---
    def _send_param_ext_value(self, setting: CameraSetting) -> None:
        index = CAMERA_SETTINGS.index(setting)
        with self._send_lock:
            self._mav.param_ext_value_send(
                setting.name.encode("ascii"),
                setting.encode(self._settings[setting.name]),
                setting.mav_type,
                len(CAMERA_SETTINGS),
                index,
            )

    def _send_param_ext_ack(self, setting: CameraSetting, result: int) -> None:
        with self._send_lock:
            self._mav.param_ext_ack_send(
                setting.name.encode("ascii"),
                setting.encode(self._settings[setting.name]),
                setting.mav_type,
                result,
            )

    @staticmethod
    def _param_id(raw) -> str:
        if isinstance(raw, bytes):
            raw = raw.decode("ascii", errors="ignore")
        return raw.split("\0", 1)[0].strip()

    def _handle_param_ext(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type == "PARAM_EXT_REQUEST_LIST":
            print("    PARAM_EXT_REQUEST_LIST")
            for setting in CAMERA_SETTINGS:
                self._send_param_ext_value(setting)
            return

        if msg_type == "PARAM_EXT_REQUEST_READ":
            index = int(msg.param_index)
            if 0 <= index < len(CAMERA_SETTINGS):
                setting = CAMERA_SETTINGS[index]
            else:
                setting = self._setting_defs.get(self._param_id(msg.param_id))
            if setting is not None:
                self._send_param_ext_value(setting)
            return

        if msg_type == "PARAM_EXT_SET":
            # param_value is a raw byte buffer (a little-endian uint32/float
            # for our types). pymavlink decodes char[] fields as text and
            # drops everything after the first NUL, which mangles most
            # numbers, so unpack the wire payload directly instead. (Taken
            # from the raw frame: get_payload() also returns header bytes.)
            frame = bytes(msg.get_msgbuf())
            header_len = 10 if frame[0] == 0xFD else 6
            payload = frame[header_len:header_len + frame[1]].ljust(147, b"\0")
            _, _, raw_id, raw_value, _ = struct.unpack("<BB16s128sB", payload[:147])
            name = self._param_id(raw_id)
            setting = self._setting_defs.get(name)
            if setting is None:
                print(f"    PARAM_EXT_SET unknown parameter {name!r}")
                return
            value = setting.decode(raw_value)
            result = self._apply_setting(setting, value)
            print(f"    PARAM_EXT_SET {name}={value} -> result {result}")
            self._send_param_ext_ack(setting, result)
            return

    def _apply_setting(self, setting: CameraSetting, value) -> int:
        if not setting.is_valid(value):
            return PARAM_ACK_VALUE_UNSUPPORTED
        value = setting.coerce(value)
        if setting.name == "CAM_MODE":
            if not self._set_camera_mode(value):
                return PARAM_ACK_FAILED
            return PARAM_ACK_ACCEPTED
        with self._camera_lock:
            self._settings[setting.name] = value
            self._save_settings()
        return PARAM_ACK_ACCEPTED

    # ---------------------------------------------------------- modes ---
    def _set_camera_mode(self, mode: int) -> bool:
        if mode not in (CAMERA_MODE_IMAGE, CAMERA_MODE_VIDEO):
            return False
        with self._camera_lock:
            if self._video_process is not None and mode != CAMERA_MODE_VIDEO:
                return False  # don't leave video mode mid-recording
            self._camera_mode = mode
            self._settings["CAM_MODE"] = mode
            self._save_settings()
        self._send_camera_settings()
        return True

    # ---------------------------------------------------- photo capture ---
    def _vehicle_snapshot(self) -> dict:
        pos = self._vehicle_position
        att = self._vehicle_attitude
        if att is not None:
            q = _quat_mul(_euler_to_quat(att.roll, att.pitch, att.yaw), self._mount_quat)
        else:
            q = list(self._mount_quat)
        return {
            "lat": int(pos.lat) if pos is not None else 0,
            "lon": int(pos.lon) if pos is not None else 0,
            "alt": int(pos.alt) if pos is not None else 0,
            "relative_alt": int(pos.relative_alt) if pos is not None else 0,
            "q": [float(v) for v in q],
        }

    def _filter_chain(self, resolution_setting: str) -> str:
        filters = []
        ev = float(self._settings["CAM_EV"])
        color = int(self._settings["CAM_COLORMODE"])
        saturation = {0: 1.0, 1: 1.4, 2: 0.0}.get(color, 1.0)
        if ev or saturation != 1.0:
            filters.append(f"eq=brightness={ev * 0.08:.3f}:saturation={saturation}")
        kelvin = _WB_KELVIN.get(int(self._settings["CAM_WBMODE"]))
        if kelvin:
            filters.append(f"colortemperature=temperature={kelvin}")
        size = _RESOLUTIONS.get(int(self._settings[resolution_setting]))
        if size:
            filters.append(f"scale={size[0]}:{size[1]}")
        return ",".join(filters)

    def _start_image_capture(self, interval: float, total: int) -> int:
        with self._camera_lock:
            if self._capture_thread is not None and self._capture_thread.is_alive():
                return mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
            self._capture_stop.clear()
            self._image_interval = interval if total != 1 else 0.0
            self._capture_thread = threading.Thread(
                target=self._image_capture_worker,
                args=(interval, total),
                name="image-capture",
                daemon=True,
            )
            self._capture_thread.start()
        return mavutil.mavlink.MAV_RESULT_ACCEPTED

    def _image_capture_worker(self, interval: float, total: int) -> None:
        interval_mode = total != 1
        taken = 0
        try:
            while not self._capture_stop.is_set() and not self._stop_event.is_set():
                started = time.monotonic()
                with self._camera_lock:
                    self._image_status = (
                        IMAGE_STATUS_INTERVAL_CAPTURING if interval_mode else IMAGE_STATUS_CAPTURING
                    )
                self._send_capture_status()
                self._take_photo()
                taken += 1
                if total and taken >= total:
                    break
                with self._camera_lock:
                    self._image_status = IMAGE_STATUS_INTERVAL_IDLE
                self._send_capture_status()
                self._capture_stop.wait(max(0.0, interval - (time.monotonic() - started)))
        finally:
            with self._camera_lock:
                self._image_status = IMAGE_STATUS_IDLE
                self._image_interval = 0.0
            self._send_capture_status()

    def _take_photo(self) -> None:
        snapshot = self._vehicle_snapshot()
        time_boot_ms = self._time_boot_ms()
        time_utc = int(time.time() * 1e6)
        with self._camera_lock:
            index = self._image_count
            self._image_count += 1
            vf = self._filter_chain("CAM_PHOTORES")
            qscale = _JPEG_QSCALE.get(int(self._settings["CAM_PHOTOQUAL"]), 2)
        name = f"IMG_{index:05d}.jpg"
        path = self._images_dir / name

        cmd = [
            "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
            "-rtsp_transport", "tcp", "-i", self._rtsp_uri,
            "-frames:v", "1",
        ]
        if vf:
            cmd += ["-vf", vf]
        cmd += ["-q:v", str(qscale), "-y", str(path)]

        ok = False
        try:
            result = subprocess.run(
                cmd, capture_output=True, timeout=self._args.capture_timeout
            )
            ok = result.returncode == 0 and path.is_file() and path.stat().st_size > 0
            if not ok:
                print(f"Photo {index} failed: {result.stderr.decode(errors='ignore').strip()}")
        except subprocess.TimeoutExpired:
            print(f"Photo {index} timed out after {self._args.capture_timeout}s")
        if not ok:
            path.unlink(missing_ok=True)

        info = {
            "image_index": index,
            "time_boot_ms": time_boot_ms,
            "time_utc": time_utc,
            **snapshot,
            "capture_result": 1 if ok else 0,
            "file_url": f"{self._http_base}/images/{name}" if ok else "",
        }
        # Sidecar so CAMERA_IMAGE_CAPTURED can be re-sent on request (even
        # after a restart) and the geotag travels with the picture.
        try:
            (self._images_dir / f"IMG_{index:05d}.json").write_text(json.dumps(info, indent=2))
        except OSError as exc:
            print(f"Could not write photo sidecar: {exc}")
        print(
            f"Photo {index} {'saved' if ok else 'FAILED'}: {name} "
            f"lat={snapshot['lat'] / 1e7:.7f} lon={snapshot['lon'] / 1e7:.7f} "
            f"rel_alt={snapshot['relative_alt'] / 1000:.1f}m"
        )
        self._send_image_captured(info)

    def _send_image_captured(self, info: dict) -> None:
        with self._send_lock:
            self._mav.camera_image_captured_send(
                info["time_boot_ms"],
                info["time_utc"],
                0,  # camera_id: deprecated, 0 = the sending component
                info["lat"],
                info["lon"],
                info["alt"],
                info["relative_alt"],
                info["q"],
                info["image_index"],
                info["capture_result"],
                info["file_url"].encode("ascii", errors="ignore")[:205],
            )

    def _load_image_info(self, index: int) -> dict | None:
        try:
            return json.loads((self._images_dir / f"IMG_{index:05d}.json").read_text())
        except (OSError, ValueError):
            return None

    # ---------------------------------------------------- video capture ---
    def _start_video_capture(self) -> int:
        with self._camera_lock:
            if self._video_process is not None:
                return mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
            name = time.strftime("VID_%Y%m%d_%H%M%S.mp4", time.gmtime())
            path = self._videos_dir / name
            vf = self._filter_chain("CAM_VIDRES")
            cmd = [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-rtsp_transport", "tcp", "-i", self._rtsp_uri, "-an",
            ]
            if vf:
                cmd += ["-vf", vf, "-c:v", "libx264", "-preset", "veryfast", "-crf", "23"]
            else:
                cmd += ["-c:v", "copy"]
            cmd += ["-movflags", "+faststart", "-y", str(path)]
            try:
                log = open(f"/tmp/video_capture_{self._args.component_id}.log", "wb")
                self._video_process = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log
                )
                log.close()
            except OSError as exc:
                print(f"Video capture failed to start: {exc}")
                return mavutil.mavlink.MAV_RESULT_FAILED
            self._video_started = time.monotonic()
            self._video_path = path
        print(f"Video recording started: {name}")
        self._send_capture_status()
        return mavutil.mavlink.MAV_RESULT_ACCEPTED

    def _stop_video_capture(self) -> None:
        with self._camera_lock:
            process = self._video_process
            path = self._video_path
            self._video_process = None
            self._video_path = None
        if process is None:
            return
        if process.poll() is None:
            # "q" makes ffmpeg finish the file cleanly (moov atom written).
            try:
                process.stdin.write(b"q")
                process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
        size = path.stat().st_size if path and path.is_file() else 0
        print(f"Video recording stopped: {path.name if path else '?'} ({size} bytes)")
        self._send_capture_status()

    def _recording_time_ms(self) -> int:
        if self._video_process is None:
            return 0
        return int((time.monotonic() - self._video_started) * 1000)

    # --------------------------------------------------- status/storage ---
    def _storage_usage(self) -> tuple[float, float, float]:
        usage = shutil.disk_usage(self._storage_dir)
        mib = 1024.0 * 1024.0
        return usage.total / mib, usage.used / mib, usage.free / mib

    def _send_capture_status(self) -> None:
        _, _, available = self._storage_usage()
        with self._camera_lock:
            image_status = self._image_status
            interval = self._image_interval
            video_status = 1 if self._video_process is not None else 0
            recording_ms = self._recording_time_ms()
            image_count = self._image_count
        with self._send_lock:
            self._mav.camera_capture_status_send(
                self._time_boot_ms(),
                image_status,
                video_status,
                interval,
                recording_ms,
                available,
                image_count,
            )

    def _send_storage_information(self) -> None:
        total, used, available = self._storage_usage()
        with self._send_lock:
            self._mav.storage_information_send(
                self._time_boot_ms(),
                1,  # storage_id
                1,  # storage_count
                mavutil.mavlink.STORAGE_STATUS_READY,
                total,
                used,
                available,
                0.0,
                0.0,
                _safe_mavlink_const("STORAGE_TYPE_OTHER", 254),
                b"SIM STORAGE",
            )

    def _format_storage(self, reset_image_log: bool) -> int:
        with self._camera_lock:
            busy = self._video_process is not None or (
                self._capture_thread is not None and self._capture_thread.is_alive()
            )
            if busy:
                return mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
            for directory in (self._images_dir, self._videos_dir):
                for path in directory.iterdir():
                    if path.is_file():
                        path.unlink(missing_ok=True)
            if reset_image_log:
                self._image_count = 0
        print(f"Storage formatted (image log {'reset' if reset_image_log else 'kept'})")
        return mavutil.mavlink.MAV_RESULT_ACCEPTED

    def _camera_status_worker(self) -> None:
        """1 Hz CAMERA_CAPTURE_STATUS while capturing; reaps a dead recorder."""
        while not self._stop_event.wait(1.0):
            with self._camera_lock:
                process = self._video_process
                capturing = self._capture_thread is not None and self._capture_thread.is_alive()
            if process is not None and process.poll() is not None:
                print(f"Video recorder exited unexpectedly (code {process.returncode})")
                with self._camera_lock:
                    self._video_process = None
                    self._video_path = None
                self._send_capture_status()
                continue
            if process is not None or capturing:
                self._send_capture_status()

    def _camera_start(self) -> None:
        self._start_http_server()
        threading.Thread(
            target=self._camera_status_worker, name="camera-status", daemon=True
        ).start()

    def _camera_stop(self) -> None:
        self._capture_stop.set()
        self._stop_video_capture()
        if self._http_server is not None:
            self._http_server.shutdown()

    # ------------------------------------------------------- dispatch ---
    def _handle_camera_request_message(self, msg, requested_msg_id: int) -> bool:
        L = mavutil.mavlink
        if requested_msg_id == L.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
            self._send_capture_status()
        elif requested_msg_id == L.MAVLINK_MSG_ID_STORAGE_INFORMATION:
            self._send_storage_information()
        elif requested_msg_id == L.MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
            info = self._load_image_info(int(msg.param2))
            if info is None:
                self._send_request_ack(msg, L.MAV_RESULT_FAILED)
                return True
            self._send_image_captured(info)
        else:
            return False
        self._send_request_ack(msg, L.MAV_RESULT_ACCEPTED)
        return True

    def _handle_camera_command(self, msg, command: int) -> bool:
        L = mavutil.mavlink
        ack = lambda result: self._send_command_ack(msg, command, result)  # noqa: E731

        if command == L.MAV_CMD_SET_CAMERA_MODE:
            mode = int(msg.param2)
            print(f"    MAV_CMD_SET_CAMERA_MODE mode={mode}")
            ok = self._set_camera_mode(mode)
            ack(L.MAV_RESULT_ACCEPTED if ok else L.MAV_RESULT_DENIED)
            if ok:
                self._send_param_ext_value(self._setting_defs["CAM_MODE"])
            return True

        if command == L.MAV_CMD_IMAGE_START_CAPTURE:
            interval, total, sequence = float(msg.param2), int(msg.param3), int(msg.param4)
            print(f"    MAV_CMD_IMAGE_START_CAPTURE interval={interval} total={total} seq={sequence}")
            # A retransmitted single-capture command repeats its sequence
            # number within a few seconds. Only treat it as a duplicate then:
            # a GCS that restarts begins again at 1 and must not be ignored.
            last = self._last_capture_sequence
            if (
                total == 1
                and sequence
                and last is not None
                and sequence == last[0]
                and time.monotonic() - last[1] < CAPTURE_RETRANSMIT_WINDOW
            ):
                ack(L.MAV_RESULT_ACCEPTED)
                return True
            if total != 1 and interval <= 0:
                ack(L.MAV_RESULT_DENIED)
                return True
            result = self._start_image_capture(interval, total)
            if result == L.MAV_RESULT_ACCEPTED and total == 1:
                self._last_capture_sequence = (sequence, time.monotonic())
            ack(result)
            return True

        if command == L.MAV_CMD_IMAGE_STOP_CAPTURE:
            print("    MAV_CMD_IMAGE_STOP_CAPTURE")
            self._capture_stop.set()
            ack(L.MAV_RESULT_ACCEPTED)
            return True

        if command == L.MAV_CMD_DO_DIGICAM_CONTROL:
            # Legacy trigger (e.g. ArduPilot CAMx_TYPE=MAVLink): param5 == 1 shoots.
            if int(msg.param5) == 1:
                print("    MAV_CMD_DO_DIGICAM_CONTROL shoot")
                ack(self._start_image_capture(0.0, 1))
            else:
                ack(L.MAV_RESULT_ACCEPTED)
            return True

        if command == L.MAV_CMD_VIDEO_START_CAPTURE:
            print("    MAV_CMD_VIDEO_START_CAPTURE")
            ack(self._start_video_capture())
            return True

        if command == L.MAV_CMD_VIDEO_STOP_CAPTURE:
            print("    MAV_CMD_VIDEO_STOP_CAPTURE")
            ack(L.MAV_RESULT_ACCEPTED)
            threading.Thread(target=self._stop_video_capture, daemon=True).start()
            return True

        if command == L.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            ack(L.MAV_RESULT_ACCEPTED)
            self._send_capture_status()
            return True

        if command == L.MAV_CMD_REQUEST_STORAGE_INFORMATION:
            ack(L.MAV_RESULT_ACCEPTED)
            self._send_storage_information()
            return True

        if command == L.MAV_CMD_STORAGE_FORMAT:
            print(f"    MAV_CMD_STORAGE_FORMAT format={msg.param2} reset_log={msg.param3}")
            if int(msg.param2) != 1:
                ack(L.MAV_RESULT_DENIED)
                return True
            result = self._format_storage(reset_image_log=int(msg.param3) == 1)
            ack(result)
            if result == L.MAV_RESULT_ACCEPTED:
                self._send_storage_information()
                self._send_capture_status()
            return True

        if command == L.MAV_CMD_RESET_CAMERA_SETTINGS:
            print("    MAV_CMD_RESET_CAMERA_SETTINGS")
            if int(msg.param1) != 1:
                ack(L.MAV_RESULT_ACCEPTED)
                return True
            with self._camera_lock:
                if self._video_process is not None:
                    ack(L.MAV_RESULT_TEMPORARILY_REJECTED)
                    return True
                self._settings = {s.name: s.default for s in CAMERA_SETTINGS}
                self._camera_mode = int(self._settings["CAM_MODE"])
                self._save_settings()
            ack(L.MAV_RESULT_ACCEPTED)
            for setting in CAMERA_SETTINGS:
                self._send_param_ext_value(setting)
            self._send_camera_settings()
            return True

        return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Demo MAVLink video source service")
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Target UDP host (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"Target UDP port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--system-id",
        type=int,
        default=DEFAULT_SYSTEM_ID,
        help=f"MAVLink source system id (default: {DEFAULT_SYSTEM_ID})",
    )
    parser.add_argument(
        "--component-id",
        type=int,
        default=DEFAULT_COMPONENT_ID,
        help=f"MAVLink source component id (default: {DEFAULT_COMPONENT_ID})",
    )
    parser.add_argument(
        "--heartbeat-period",
        type=float,
        default=DEFAULT_HEARTBEAT_PERIOD,
        help=f"Heartbeat period in seconds (default: {DEFAULT_HEARTBEAT_PERIOD})",
    )
    parser.add_argument(
        "--bitrate-bps",
        type=int,
        default=DEFAULT_BITRATE_BPS,
        help=f"Reported video bitrate in bits/s (default: {DEFAULT_BITRATE_BPS})",
    )
    parser.add_argument(
        "--video-codec",
        choices=("h264", "h265"),
        default=DEFAULT_VIDEO_CODEC,
        help=f"Video codec for FFmpeg stream (default: {DEFAULT_VIDEO_CODEC})",
    )
    parser.add_argument(
        "--stream-id",
        type=int,
        default=DEFAULT_STREAM_ID,
        help=f"Video stream id to report (default: {DEFAULT_STREAM_ID})",
    )
    parser.add_argument(
        "--ffmpeg-restart-delay",
        type=float,
        default=1.0,
        help="Seconds to wait before restarting FFmpeg after failure (default: 1.0)",
    )
    parser.add_argument(
        "--ffmpeg-monitor-period",
        type=float,
        default=0.5,
        help="Seconds between FFmpeg health checks (default: 0.5)",
    )
    parser.add_argument(
        "--show-ffmpeg-output",
        action="store_true",
        help="Show FFmpeg stdout/stderr in terminal (default: hidden)",
    )
    parser.add_argument(
        "--ffmpeg-pkt-size",
        type=int,
        default=DEFAULT_FFMPEG_PKT_SIZE,
        help=(
            "Maximum RTP packet size used by FFmpeg RTSP output "
            f"(default: {DEFAULT_FFMPEG_PKT_SIZE})"
        ),
    )
    parser.add_argument(
        "--request-timeout",
        type=float,
        default=1.0,
        help="Seconds to wait in MAVLink receive loop (default: 1.0)",
    )
    parser.add_argument(
        "--rtsp-uri",
        default=None,
        help=(
            "RTSP output URI to stream to "
            "(default: rtsp://<host>:18554/demo)"
        ),
    )
    parser.add_argument(
        "--notif-uri",
        default=None,
        help=(
            "URI reported in VIDEO_STREAM_INFORMATION "
            "(default: the --rtsp-uri value)"
        ),
    )
    parser.add_argument(
        "--video-source",
        choices=("test", "rtp", "none"),
        default=DEFAULT_VIDEO_SOURCE,
        help=(
            "FFmpeg input stage: 'test' generates a synthetic testsrc pattern "
            "(unchanged default behaviour); 'rtp' ingests bare RTP/UDP H.264 "
            f"from Gazebo's GstCameraPlugin instead; 'none' publishes no video at all "
            "(MAVLink camera component only -- the RTSP URI is served by "
            "something else, e.g. a MediaMTX proxy path) (default: {DEFAULT_VIDEO_SOURCE})"
        ),
    )
    parser.add_argument(
        "--rtp-port",
        type=int,
        default=DEFAULT_RTP_PORT,
        help=(
            "UDP port this container listens on for RTP from Gazebo, used "
            f"when --video-source=rtp (default: {DEFAULT_RTP_PORT})"
        ),
    )
    parser.add_argument(
        "--rtp-sdp",
        default=None,
        help=(
            "Path to write the generated SDP file describing the RTP input, "
            "used when --video-source=rtp "
            "(default: /tmp/demo_video_source_rtp_<rtp-port>.sdp)"
        ),
    )
    parser.add_argument(
        "--placeholder-width",
        type=int,
        default=DEFAULT_PLACEHOLDER_WIDTH,
        help=(
            "Width of the placeholder pattern published when 0 readers are "
            "attached (or the real feed hasn't shown up yet), used when "
            f"--video-source=rtp (default: {DEFAULT_PLACEHOLDER_WIDTH})"
        ),
    )
    parser.add_argument(
        "--placeholder-height",
        type=int,
        default=DEFAULT_PLACEHOLDER_HEIGHT,
        help=(
            "Height of the placeholder pattern, used when --video-source=rtp "
            f"(default: {DEFAULT_PLACEHOLDER_HEIGHT})"
        ),
    )
    parser.add_argument(
        "--placeholder-fps",
        type=int,
        default=DEFAULT_PLACEHOLDER_FPS,
        help=(
            "Frame rate of the placeholder pattern, used when "
            f"--video-source=rtp (default: {DEFAULT_PLACEHOLDER_FPS})"
        ),
    )
    parser.add_argument(
        "--rtp-grace-period",
        type=float,
        default=DEFAULT_RTP_GRACE_PERIOD,
        help=(
            "Seconds to wait for real RTP frames to arrive after switching "
            "to RTP mode before giving up and falling back to the "
            "placeholder (retried on the next gate poll); used when "
            f"--video-source=rtp (default: {DEFAULT_RTP_GRACE_PERIOD})"
        ),
    )
    parser.add_argument(
        "--rtp-revert-hold",
        type=float,
        default=DEFAULT_RTP_REVERT_HOLD,
        help=(
            "Seconds RTP mode is held open with zero readers before "
            "reverting to the placeholder. Must comfortably exceed the "
            "placeholder-kill -> RTP-publisher-up gap (~4-5s measured) plus "
            "the viewer's own reconnect cadence, or the switch can never "
            "complete; used when --video-source=rtp "
            f"(default: {DEFAULT_RTP_REVERT_HOLD})"
        ),
    )
    parser.add_argument(
        "--gate-poll-period",
        type=float,
        default=DEFAULT_GATE_POLL_PERIOD,
        help=(
            "Seconds between polls of this stream's own MediaMTX reader "
            f"count, used when --video-source=rtp (default: {DEFAULT_GATE_POLL_PERIOD})"
        ),
    )
    parser.add_argument(
        "--gate-poll-timeout",
        type=float,
        default=DEFAULT_GATE_POLL_TIMEOUT,
        help=(
            "Per-request timeout (seconds) for the MediaMTX reader-count "
            f"poll, used when --video-source=rtp (default: {DEFAULT_GATE_POLL_TIMEOUT})"
        ),
    )
    parser.add_argument(
        "--mediaserver-api-port",
        type=int,
        default=DEFAULT_MEDIASERVER_API_PORT,
        help=(
            "MediaMTX HTTP API port (MTX_API) to poll for this stream's own "
            "reader count. The host is derived from --rtsp-uri (MediaMTX is "
            "always the RTSP target, so it's always the right API host too) "
            f"-- used when --video-source=rtp (default: {DEFAULT_MEDIASERVER_API_PORT})"
        ),
    )
    parser.add_argument(
        "--camera-vendor",
        default=DEFAULT_CAMERA_VENDOR,
        help=f"Vendor name reported in CAMERA_INFORMATION (default: {DEFAULT_CAMERA_VENDOR})",
    )
    parser.add_argument(
        "--camera-model",
        default=DEFAULT_CAMERA_MODEL,
        help=f"Model name reported in CAMERA_INFORMATION (default: {DEFAULT_CAMERA_MODEL})",
    )
    parser.add_argument(
        "--overlay-text",
        default=None,
        help=(
            "Burn this label into the generated video (test pattern and rtp-mode "
            "placeholder) with ffmpeg drawtext. Not applied to --video-source=rtp "
            "passthrough, which copies the stream without re-encoding "
            "(default: no overlay)"
        ),
    )
    parser.add_argument(
        "--overlay-font",
        default=DEFAULT_OVERLAY_FONT,
        help=f"fontconfig font name for --overlay-text (default: {DEFAULT_OVERLAY_FONT})",
    )
    parser.add_argument(
        "--storage-dir",
        default=DEFAULT_STORAGE_DIR,
        help=(
            "Where captured photos (images/) and videos (videos/) and the "
            f"camera settings are stored (default: {DEFAULT_STORAGE_DIR})"
        ),
    )
    parser.add_argument(
        "--http-port",
        type=int,
        default=DEFAULT_HTTP_PORT,
        help=(
            "Port of the HTTP server that serves the camera definition "
            f"(/camera.xml) and captured files (default: {DEFAULT_HTTP_PORT})"
        ),
    )
    parser.add_argument(
        "--http-advertise-host",
        default=None,
        help=(
            "Host the GCS uses to reach the HTTP server, in cam_definition_uri "
            "and photo file URLs (default: the --notif-uri host)"
        ),
    )
    parser.add_argument(
        "--http-advertise-port",
        type=int,
        default=None,
        help="Port the GCS uses to reach the HTTP server (default: --http-port)",
    )
    parser.add_argument(
        "--capture-timeout",
        type=float,
        default=DEFAULT_CAPTURE_TIMEOUT,
        help=(
            "Seconds to wait for a photo to be grabbed from --rtsp-uri; covers a "
            f"render-gated camera's cold start (default: {DEFAULT_CAPTURE_TIMEOUT})"
        ),
    )
    parser.add_argument(
        "--camera-pitch-deg",
        type=float,
        default=DEFAULT_CAMERA_PITCH_DEG,
        help=(
            "Fixed mount pitch relative to the vehicle, used for the photo "
            "orientation quaternion: 0 = forward, -90 = straight down "
            f"(default: {DEFAULT_CAMERA_PITCH_DEG})"
        ),
    )
    parser.add_argument(
        "--hfov-deg",
        type=float,
        default=DEFAULT_HFOV_DEG,
        help=(
            "Horizontal field of view, used to derive the focal length reported "
            f"in CAMERA_INFORMATION (default: {DEFAULT_HFOV_DEG:.1f})"
        ),
    )
    return parser.parse_args()


class DemoVideoSource(MavlinkCameraMixin):
    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        self._rtsp_uri = args.rtsp_uri or _default_rtsp_uri(args.host)
        # URI advertised to the GCS in VIDEO_STREAM_INFORMATION. Defaults to the
        # stream's own URI, but can point elsewhere when the address clients
        # should pull from differs from the one ffmpeg pushes to.
        self._notif_uri = args.notif_uri or self._rtsp_uri
        self._boot_time_monotonic = time.monotonic()

        # Cached H.264 SPS/PPS (raw NAL bytes, not base64), only used when
        # --video-source=rtp. See _capture_param_sets_from_rtp / _write_rtp_sdp
        # for why: GstCameraPlugin's RTP pipeline sends them in-band only
        # once, right as the sensor's GStreamer pipeline (re)starts on
        # enable, and our own ffmpeg process has no way to guarantee it's
        # already listening at that exact instant. Once captured (by
        # _await_rtp_traffic parsing whatever RTP it sees while waiting),
        # they're kept for the rest of the process's life and baked into
        # the SDP so ffmpeg never has to catch that one-time burst again.
        self._cached_sps: bytes | None = None
        self._cached_pps: bytes | None = None
        # Continuously-bound RTP-sniffing socket, owned by the gate thread
        # (see _open_rtp_probe/_close_rtp_probe/_drain_rtp_probe). None
        # whenever it isn't currently bound (rtp mode, or before the gate
        # thread starts).
        self._rtp_probe_socket: socket.socket | None = None

        # Only relevant when --video-source=rtp, but harmless to compute
        # unconditionally; written to disk below before ffmpeg needs it.
        self._rtp_sdp_path = args.rtp_sdp or _default_rtp_sdp_path(args.rtp_port)
        if args.video_source == "rtp":
            self._write_rtp_sdp()

        # Render-gate cooperation state, only driven when --video-source=rtp
        # (see module docstring). Start in "placeholder" so the very first
        # ffmpeg launch publishes something immediately -- never leave a gap
        # where the path doesn't exist. In "test" mode this is set but never
        # read: _ffmpeg_command() only branches on it for "rtp".
        self._stream_mode = "placeholder"
        self._mode_lock = threading.Lock()
        # MediaMTX is always the RTSP target (--rtsp-uri points at it), so
        # its host and this stream's path name can be derived from that URI
        # instead of needing separate flags that could drift out of sync.
        parsed_rtsp = urlsplit(self._rtsp_uri)
        self._gate_host = parsed_rtsp.hostname
        self._gate_path = parsed_rtsp.path.lstrip("/")

        self._target_addr = (args.host, args.port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Explicit bind on all interfaces with an ephemeral port. Without this
        # the kernel only binds the socket on the first sendto(), and it picks
        # a local address based on the destination — so replies arriving via a
        # different interface than the one chosen for sendto() are silently
        # dropped.
        self._socket.bind(("0.0.0.0", 0))
        self._local_addr = self._socket.getsockname()
        self._socket.settimeout(args.request_timeout)

        self._mav = mavutil.mavlink.MAVLink(
            _MavlinkUdpSender(self._socket, self._target_addr),
            srcSystem=args.system_id,
            srcComponent=args.component_id,
        )

        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._ffmpeg_process: subprocess.Popen | None = None
        self._ffmpeg_lock = threading.Lock()

        self._camera_init()

    def _time_boot_ms(self) -> int:
        elapsed_s = time.monotonic() - self._boot_time_monotonic
        return int(elapsed_s * 1000) & 0xFFFFFFFF

    def _write_rtp_sdp(self) -> None:
        """Write the SDP file FFmpeg needs to ingest bare RTP/UDP H.264.

        MediaMTX cannot take RTP directly, and neither can FFmpeg's RTP
        demuxer without a description of the stream. This is a static,
        single-video-stream SDP; it is regenerated (not appended to) on
        every start (and again whenever _cached_sps/_cached_pps change --
        see _await_rtp_traffic) so a changed --rtp-port always takes effect
        and freshly-captured parameter sets reach ffmpeg immediately.

        When SPS/PPS have been captured, they're included out-of-band as
        `sprop-parameter-sets` (RFC 6184 sec. 8.2.1). Measured against the
        real stack: without this, ffmpeg's RTP demuxer has to rely on
        catching GstCameraPlugin's in-band SPS/PPS NALs, which it sends only
        once, right as the sensor's GStreamer pipeline (re)starts on enable
        -- a burst that arrives and is gone in tens of milliseconds. Our own
        ffmpeg process starting a beat later than that (spawn + SDP-parse
        latency alone is usually enough) means it never sees them and can
        never determine the stream's dimensions, so it fails with "Could not
        find codec parameters ... unspecified size" and the passthrough
        never succeeds, no matter how many times it's retried. Declaring the
        parameter sets in the SDP sidesteps the race entirely: ffmpeg has
        everything it needs before the first packet even arrives.
        """
        fmtp_line = ""
        if self._cached_sps and self._cached_pps:
            sps_b64 = base64.b64encode(self._cached_sps).decode("ascii")
            pps_b64 = base64.b64encode(self._cached_pps).decode("ascii")
            fmtp_line = (
                f"a=fmtp:96 packetization-mode=1;sprop-parameter-sets={sps_b64},{pps_b64}\n"
            )
        sdp = (
            "v=0\n"
            "o=- 0 0 IN IP4 127.0.0.1\n"
            "s=demo_video_source\n"
            "c=IN IP4 0.0.0.0\n"
            "t=0 0\n"
            f"m=video {self._args.rtp_port} RTP/AVP 96\n"
            "a=rtpmap:96 H264/90000\n"
            f"{fmtp_line}"
        )
        with open(self._rtp_sdp_path, "w") as f:
            f.write(sdp)
        print(
            f"Wrote RTP SDP to {self._rtp_sdp_path} (port={self._args.rtp_port}, "
            f"sprop-parameter-sets={'yes' if fmtp_line else 'no'})"
        )

    def _capture_param_sets_from_rtp(self, packet: bytes) -> bool:
        """Best-effort SPS/PPS extraction from one raw RTP/H.264 datagram.
        Returns True if this call newly captured a parameter set that
        wasn't already cached (i.e. the SDP is now stale and should be
        rewritten). Safe to call on any packet, including ones that turn
        out not to carry SPS/PPS -- it just does nothing in that case.

        Handles the two shapes GStreamer's rtph264pay realistically uses
        for parameter sets: a single NAL per packet (type 7=SPS, 8=PPS), or
        both aggregated into one STAP-A packet (type 24, RFC 6184 sec.
        5.7.1). Does not handle FU-A fragmentation (type 28) because SPS/PPS
        are always tiny and never fragmented in practice; FU-A is what
        large slice NALs use, which aren't parameter sets and are ignored
        here.
        """
        if len(packet) < 12:
            return False
        b0 = packet[0]
        header_len = 12 + 4 * (b0 & 0x0F)  # + 4 bytes per CSRC identifier
        if b0 & 0x10:  # extension header present
            if len(packet) < header_len + 4:
                return False
            ext_len_words = int.from_bytes(packet[header_len + 2 : header_len + 4], "big")
            header_len += 4 + 4 * ext_len_words
        if len(packet) <= header_len:
            return False
        payload = packet[header_len:]

        nal_type = payload[0] & 0x1F
        changed = False
        if nal_type == 24:  # STAP-A aggregate: (2-byte length + NAL)*
            offset = 1
            while offset + 2 <= len(payload):
                nal_len = int.from_bytes(payload[offset : offset + 2], "big")
                offset += 2
                if nal_len <= 0 or offset + nal_len > len(payload):
                    break
                changed = self._cache_param_set(payload[offset : offset + nal_len]) or changed
                offset += nal_len
        elif nal_type in (7, 8):
            changed = self._cache_param_set(payload)
        return changed

    def _cache_param_set(self, nal_bytes: bytes) -> bool:
        if not nal_bytes:
            return False
        nal_type = nal_bytes[0] & 0x1F
        if nal_type == 7 and nal_bytes != self._cached_sps:
            self._cached_sps = nal_bytes
            print(f"Gate: captured SPS ({len(nal_bytes)} bytes) from live RTP")
            return True
        if nal_type == 8 and nal_bytes != self._cached_pps:
            self._cached_pps = nal_bytes
            print(f"Gate: captured PPS ({len(nal_bytes)} bytes) from live RTP")
            return True
        return False

    def _ffmpeg_command(self) -> list[str]:
        # --video-source=test: unchanged, single mode, byte-for-byte the
        # original command list. This is the demo profile's fallback and
        # must never regress -- see support-tools/demo_video_source.md and
        # the compose-time verification in the gazebo plan.
        if self._args.video_source != "rtp":
            return self._test_pattern_command()

        # --video-source=rtp: which command depends on the render-gate state
        # (_gate_worker), not a static choice -- see module docstring.
        if self._stream_mode == "rtp":
            return self._rtp_passthrough_command()
        return self._placeholder_command()

    def _overlay_filter(self) -> str:
        """Extra lavfi stage for --overlay-text, or "" when unset -- so with no
        overlay the generated commands stay byte-identical to the originals.
        The label goes through drawtext's textfile= rather than text= to
        sidestep ffmpeg's two-level filter-graph escaping (':', ',', quotes
        and backslashes would all need escaping inline). Written once per
        start, like the RTP SDP file.
        """
        text = self._args.overlay_text
        if not text:
            return ""
        path = _overlay_textfile_path(self._args.component_id)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
        return (
            f",drawtext=font={self._args.overlay_font}:textfile={path}"
            ":fontsize=h/12:fontcolor=white:borderw=3:bordercolor=black"
            ":x=(w-text_w)/2:y=h-text_h-h/12"
        )

    def _test_pattern_command(self) -> list[str]:
        codec_library = "libx264" if self._args.video_codec == "h264" else "libx265"
        return [
            "ffmpeg",
            "-re",
            "-f",
            "lavfi",
            "-i",
            "testsrc=size=1280x720:rate=30" + self._overlay_filter() + ",format=yuv420p",
            "-c:v",
            codec_library,
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-pkt_size",
            str(self._args.ffmpeg_pkt_size),
            "-f",
            "rtsp",
            self._rtsp_uri,
        ]

    def _rtp_passthrough_command(self) -> list[str]:
        return [
            "ffmpeg",
            "-protocol_whitelist",
            "file,udp,rtp",
            "-i",
            self._rtp_sdp_path,
            "-c:v",
            "copy",
            "-pkt_size",
            str(self._args.ffmpeg_pkt_size),
            "-f",
            "rtsp",
            self._rtsp_uri,
        ]

    def _placeholder_command(self) -> list[str]:
        """Cheap stand-in published while nobody's watching or the real feed
        hasn't shown up yet. This is what keeps the MediaMTX path alive so a
        viewer has something to DESCRIBE/attach to in the first place --
        without it, the render gate and this service wait on each other
        forever (see module docstring). Small and low frame rate on purpose:
        it's discarded the moment real RTP is flowing, so it should cost as
        little CPU as possible while it's up.
        """
        codec_library = "libx264" if self._args.video_codec == "h264" else "libx265"
        return [
            "ffmpeg",
            "-re",
            "-f",
            "lavfi",
            "-i",
            (
                f"testsrc=size={self._args.placeholder_width}x{self._args.placeholder_height}"
                f":rate={self._args.placeholder_fps}" + self._overlay_filter() + ",format=yuv420p"
            ),
            "-c:v",
            codec_library,
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-pkt_size",
            str(self._args.ffmpeg_pkt_size),
            "-f",
            "rtsp",
            self._rtsp_uri,
        ]

    def _send_heartbeat(self) -> None:
        with self._send_lock:
            self._mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_CAMERA,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )

    def _send_camera_information(self) -> None:
        vendor = bytearray(32)
        vendor_name = self._args.camera_vendor.encode("ascii", errors="ignore")[:32]
        vendor[: len(vendor_name)] = vendor_name

        model = bytearray(32)
        model_name = self._args.camera_model.encode("ascii", errors="ignore")[:32]
        model[: len(model_name)] = model_name

        sensor_h = SENSOR_WIDTH_MM
        sensor_v = SENSOR_WIDTH_MM * VIDEO_HEIGHT / VIDEO_WIDTH
        focal_length = (sensor_h / 2) / math.tan(math.radians(self._args.hfov_deg) / 2)

        with self._send_lock:
            self._mav.camera_information_send(
                self._time_boot_ms(),
                vendor,
                model,
                0,
                focal_length,
                sensor_h,
                sensor_v,
                VIDEO_WIDTH,
                VIDEO_HEIGHT,
                0,
                CAMERA_CAP_FLAGS,
                self._definition_version,
                self._definition_uri.encode("ascii", errors="ignore")[:140],
                0,
            )

    def _send_camera_settings(self) -> None:
        with self._send_lock:
            self._mav.camera_settings_send(
                self._time_boot_ms(),
                self._camera_mode,
                0.0,
                0.0,
            )

    def _send_video_stream_status(self) -> None:
        with self._send_lock:
            self._mav.video_stream_status_send(
                self._args.stream_id,
                VIDEO_STREAM_STATUS_FLAGS_RUNNING,
                30.0,
                VIDEO_WIDTH,
                VIDEO_HEIGHT,
                self._args.bitrate_bps,
                0,
                0,
            )

    def _send_video_stream_information(self) -> None:
        name = bytearray(32)
        name[: len(VIDEO_NAME)] = VIDEO_NAME.encode("ascii")

        uri_bytes = self._notif_uri.encode("ascii", errors="ignore")[:160]
        uri = bytearray(160)
        uri[: len(uri_bytes)] = uri_bytes

        with self._send_lock:
            self._mav.video_stream_information_send(
                self._args.stream_id,
                1,
                VIDEO_STREAM_TYPE_RTSP,
                VIDEO_STREAM_STATUS_FLAGS_RUNNING,
                30.0,
                VIDEO_WIDTH,
                VIDEO_HEIGHT,
                self._args.bitrate_bps,
                0,
                0,
                name,
                uri,
                0,
            )

    def _send_request_ack(self, msg, result: int) -> None:
        with self._send_lock:
            self._mav.command_ack_send(
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                result,
                0,
                0,
                msg.get_srcSystem(),
                msg.get_srcComponent(),
            )

    def _send_command_ack(self, msg, command: int, result: int) -> None:
        with self._send_lock:
            self._mav.command_ack_send(
                command,
                result,
                0,
                0,
                msg.get_srcSystem(),
                msg.get_srcComponent(),
            )

    def _handle_request_message(self, msg) -> None:
        requested_msg_id = int(msg.param1)
        target_system = msg.get_srcSystem()
        target_component = msg.get_srcComponent()

        if requested_msg_id == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION:
            self._send_camera_information()
            self._send_request_ack(msg, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return

        if requested_msg_id == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_SETTINGS:
            self._send_camera_settings()
            self._send_request_ack(msg, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return

        if requested_msg_id == mavutil.mavlink.MAVLINK_MSG_ID_VIDEO_STREAM_STATUS:
            self._send_video_stream_status()
            self._send_request_ack(msg, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return

        if requested_msg_id == mavutil.mavlink.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
            self._send_video_stream_information()
            self._send_request_ack(msg, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return

        if self._handle_camera_request_message(msg, requested_msg_id):
            return

        self._send_request_ack(msg, mavutil.mavlink.MAV_RESULT_UNSUPPORTED)

    def _handle_request_camera_information(self, msg) -> None:
        self._send_camera_information()
        self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION, mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _handle_request_camera_settings(self, msg) -> None:
        self._send_camera_settings()
        self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS, mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _handle_request_video_stream_information(self, msg) -> None:
        requested_stream_id = int(msg.param1)
        if requested_stream_id not in (0, self._args.stream_id):
            self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, mavutil.mavlink.MAV_RESULT_DENIED)
            return

        self._send_video_stream_information()
        self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _handle_request_video_stream_status(self, msg) -> None:
        requested_stream_id = int(msg.param1)
        if requested_stream_id not in (0, self._args.stream_id):
            self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS, mavutil.mavlink.MAV_RESULT_DENIED)
            return

        self._send_video_stream_status()
        self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS, mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _handle_video_start_streaming(self, msg) -> None:
        requested_stream_id = int(msg.param1)
        if requested_stream_id not in (0, self._args.stream_id):
            self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING, mavutil.mavlink.MAV_RESULT_DENIED)
            return

        # Stream supervisor runs continuously, so accepting this command is enough.
        self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING, mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _handle_video_stop_streaming(self, msg) -> None:
        requested_stream_id = int(msg.param1)
        if requested_stream_id not in (0, self._args.stream_id):
            self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING, mavutil.mavlink.MAV_RESULT_DENIED)
            return

        self._send_command_ack(msg, mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING, mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _heartbeat_worker(self) -> None:
        print(f"Heartbeat thread started (period={self._args.heartbeat_period}s)")
        while not self._stop_event.is_set():
            self._send_heartbeat()
            self._stop_event.wait(self._args.heartbeat_period)
        print("Heartbeat thread stopped")

    def _command_worker(self) -> None:
        print(f"Command thread started, listening on {self._local_addr}")
        while not self._stop_event.is_set():
            try:
                data, peer = self._socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                if self._stop_event.is_set():
                    break
                print(f"UDP receive error: {exc}")
                continue

            #print(f"RX {len(data)} bytes from {peer[0]}:{peer[1]}")

            try:
                msgs = self._mav.parse_buffer(data) or []
            except mavutil.mavlink.MAVError as exc:
                print(f"MAVLink parse error: {exc}")
                continue

            for msg in msgs:
                msg_type = msg.get_type()
                src_sys = msg.get_srcSystem()
                src_comp = msg.get_srcComponent()

                if msg_type not in ("COMMAND_LONG", "COMMAND_INT"):
                    try:
                        self._handle_camera_message(msg)
                    except Exception as exc:
                        print(f"Camera message handler error for {msg_type}: {exc}")
                    continue

                try:
                    command = int(msg.command)
                    target_sys = int(getattr(msg, "target_system", 0))
                    target_comp = int(getattr(msg, "target_component", 0))
                    #print(
                    #    f"    command={command} "
                    #    f"target_sys={target_sys} target_comp={target_comp} "
                    #    f"param1={getattr(msg, 'param1', 0)}"
                    #)

                    # Drop commands not addressed to us. 0 is the MAVLink
                    # broadcast value for target_system / target_component.
                    if target_sys not in (0, self._args.system_id):
                        #print(f"    skipped (target_sys={target_sys})")
                        continue
                    if target_comp not in (0, self._args.component_id):
                        #print(f"    skipped (target_comp={target_comp})")
                        continue

                    print(f"  msg={msg_type} src_sys={src_sys} src_comp={src_comp}")

                    if command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                        print(f"    handling MAV_CMD_REQUEST_MESSAGE for msg_id={int(msg.param1)}")
                        self._handle_request_message(msg)
                        continue

                    if command == mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                        print(f"    handling MAV_CMD_REQUEST_CAMERA_INFORMATION")
                        self._handle_request_camera_information(msg)
                        continue

                    if command == mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                        print(f"    handling MAV_CMD_REQUEST_CAMERA_SETTINGS")
                        self._handle_request_camera_settings(msg)
                        continue

                    if command == mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                        print(f"    handling MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION")
                        self._handle_request_video_stream_information(msg)
                        continue

                    if command == mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS:
                        print(f"    handling MAV_CMD_REQUEST_VIDEO_STREAM_STATUS")
                        self._handle_request_video_stream_status(msg)
                        continue

                    if command == mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING:
                        print(f"    handling MAV_CMD_VIDEO_START_STREAMING")
                        self._handle_video_start_streaming(msg)
                        continue

                    if command == mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING:
                        print(f"    handling MAV_CMD_VIDEO_STOP_STREAMING")
                        self._handle_video_stop_streaming(msg)
                        continue

                    if self._handle_camera_command(msg, command):
                        continue

                    print(f"    unhandled command={command}, replying UNSUPPORTED")
                    self._send_command_ack(
                        msg, command, mavutil.mavlink.MAV_RESULT_UNSUPPORTED
                    )
                except Exception as exc:
                    print(f"Command handler error for command={int(msg.command)}: {exc}")

        print("Command thread stopped")

    def _terminate_ffmpeg(self) -> None:
        with self._ffmpeg_lock:
            process = self._ffmpeg_process
            self._ffmpeg_process = None

        if process is None:
            return

        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

    def _poll_own_path(self) -> tuple[int, bool]:
        """Reader count and "ready" (actively publishing frames) for this
        stream's own MediaMTX path. Mirrors gazebo/docker/camera_gate.py's
        reader_count() (same endpoint, same "unreachable/404 == 0 readers"
        treatment) but is deliberately a separate poller: camera_gate.py
        uses this signal to toggle the Gazebo camera sensor; this one uses
        it to toggle which ffmpeg command is running. Two independent
        readers of one read-only signal, not a duplicate control path.

        Never raises -- an unreachable mediaserver or an unpublished path
        (404) is a normal, expected state (e.g. right after startup, before
        the very first ffmpeg process has published anything), not a
        failure, so it's reported as "0 readers, not ready" rather than
        crashing this thread.
        """
        url = f"http://{self._gate_host}:{self._args.mediaserver_api_port}/v3/paths/get/{self._gate_path}"
        try:
            try:
                with urllib.request.urlopen(url, timeout=self._args.gate_poll_timeout) as resp:
                    body = resp.read()
            except urllib.error.HTTPError as exc:
                # Same dual-shape quirk camera_gate.py documents: "path not
                # found" arrives as either a non-2xx status or a 200 with a
                # status:error body, version-dependent.
                body = exc.read()
        except (urllib.error.URLError, OSError, TimeoutError):
            return 0, False

        try:
            data = json.loads(body)
        except ValueError:
            return 0, False

        if isinstance(data, dict) and data.get("status") == "error":
            return 0, False

        readers = data.get("readers")
        if readers is None:
            return 0, False
        return len(readers), bool(data.get("ready", False))

    def _set_stream_mode(self, mode: str) -> None:
        """Switch which ffmpeg command _ffmpeg_command() returns, and kick
        the existing supervisor loop (_ffmpeg_worker) into picking it up
        immediately by terminating the current process -- reusing its
        restart machinery instead of adding a second one. The supervisor's
        own poll loop notices the exit within --ffmpeg-monitor-period and
        restarts with whatever _ffmpeg_command() now returns.
        """
        with self._mode_lock:
            if self._stream_mode == mode:
                return
            self._stream_mode = mode

        print(f"Gate: switching stream mode -> {mode}")
        with self._ffmpeg_lock:
            process = self._ffmpeg_process
        if process is not None and process.poll() is None:
            process.terminate()

    # Secondary, internal-only safety margin after a switch to RTP mode
    # actually happens (see _gate_worker) -- not a CLI flag because by that
    # point we've already confirmed SPS/PPS are cached, so this is only
    # covering ffmpeg's own startup/handshake time, not "is Gazebo emitting
    # at all". Kept short and fixed for that reason.
    _RTP_CONFIRM_TIMEOUT = 3.0

    def _open_rtp_probe(self, retry_seconds: float = 3.0, quiet: bool = False) -> None:
        """(Re-)bind the RTP-sniffing socket used while in placeholder mode.
        No-op if already open. See _drain_rtp_probe for why this has to stay
        open continuously rather than being opened fresh per attempt.

        Retries briefly on EADDRINUSE: this is called right after reverting
        rtp mode -> placeholder, i.e. right after sending SIGTERM to the rtp
        ffmpeg process that was holding --rtp-port -- terminate() is async,
        so the kernel hasn't necessarily released the port back yet by the
        time we try to rebind it (measured: without the retry this reliably
        raced and lost, leaving the probe unbound and blind to the next
        SPS/PPS burst until some other event happened to call this again).

        Even the retry window can lose: the outgoing rtp-mode ffmpeg has
        been measured holding the port for ~10s after SIGTERM (it lingers
        tearing down its RTSP session). That's why _gate_worker also calls
        this once per tick (retry_seconds=0, quiet=True) whenever the probe
        is unbound in placeholder mode -- a failed re-bind here self-heals
        a tick or two later instead of leaving the probe blind until the
        next mode change.
        """
        if self._rtp_probe_socket is not None:
            return
        deadline = time.monotonic() + retry_seconds
        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(("0.0.0.0", self._args.rtp_port))
            except OSError as exc:
                sock.close()
                if time.monotonic() >= deadline or self._stop_event.is_set():
                    if not quiet:
                        print(
                            f"Gate: could not bind RTP probe socket on port "
                            f"{self._args.rtp_port}: {exc} -- will keep retrying "
                            "each gate tick"
                        )
                    return
                self._stop_event.wait(0.2)
                continue
            sock.setblocking(False)
            self._rtp_probe_socket = sock
            return

    def _close_rtp_probe(self) -> None:
        """Release the RTP-sniffing socket so the real ffmpeg process can
        bind --rtp-port itself. Only called right before switching to rtp
        mode (or on shutdown) -- see _gate_worker.
        """
        if self._rtp_probe_socket is None:
            return
        self._rtp_probe_socket.close()
        self._rtp_probe_socket = None

    def _drain_rtp_probe(self) -> None:
        """Non-blocking drain of whatever has arrived on the RTP probe
        socket since the last poll, parsing each datagram for SPS/PPS (see
        _capture_param_sets_from_rtp). Cheap -- a handful of recvfrom calls
        at most -- and called on every gate tick regardless of reader state.

        This socket is opened once and kept bound for as long as we're in
        placeholder mode, rather than being opened fresh and closed again
        around each individual attempt. Measured against the real stack:
        the fresh-per-attempt version left a real gap -- close the probe,
        wait out --gate-poll-period, reopen it -- and GstCameraPlugin's
        one-shot in-band SPS/PPS burst (sent once, right as the sensor's
        GStreamer pipeline starts on enable, never repeated while the
        camera stays continuously enabled) landed in that gap often enough
        to starve a stable, non-flapping reader of RTP forever: camera
        enabled continuously, real packets flowing continuously, and yet
        "no RTP within Xs" on every single attempt, because nothing was
        listening at the one moment that mattered. Never closing the
        socket except right before ffmpeg needs the port removes the gap
        entirely -- whatever arrives, arrives into a live socket.
        """
        probe = self._rtp_probe_socket
        if probe is None:
            return
        while True:
            try:
                data, _addr = probe.recvfrom(2048)
            except (BlockingIOError, OSError):
                return
            if data:
                self._capture_param_sets_from_rtp(data)

    def _wait_for_rtp_ready(self, timeout: float) -> bool:
        """Poll until this path reports "ready" (MediaMTX has accepted a
        publish with real stream parameters) or `timeout` elapses. Used as a
        secondary confirmation once ffmpeg has actually started in RTP mode
        -- see _gate_worker and _RTP_CONFIRM_TIMEOUT.
        """
        deadline = time.monotonic() + timeout
        while not self._stop_event.is_set():
            _readers, ready = self._poll_own_path()
            if ready:
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._stop_event.wait(min(0.5, remaining))
        return False

    def _gate_worker(self) -> None:
        """Only runs for --video-source=rtp (see start()); 'test' mode has
        no bootstrap problem to solve -- ffmpeg's own testsrc needs no
        external publisher, so it's never gated.
        """
        print(
            "Gate thread started, polling "
            f"http://{self._gate_host}:{self._args.mediaserver_api_port}/v3/paths/get/{self._gate_path} "
            f"every {self._args.gate_poll_period}s"
        )
        # Bound continuously for as long as we're in placeholder mode -- see
        # _drain_rtp_probe for why a fresh-per-attempt socket isn't enough.
        self._open_rtp_probe()
        last_waiting_log = 0.0
        try:
            while not self._stop_event.is_set():
                # Self-heal an unbound probe: the outgoing rtp-mode ffmpeg
                # can hold --rtp-port for ~10s after SIGTERM (measured),
                # outliving _open_rtp_probe's retry window on a revert. One
                # quiet re-attempt per tick converges as soon as the port
                # frees up.
                if self._stream_mode == "placeholder" and self._rtp_probe_socket is None:
                    self._open_rtp_probe(retry_seconds=0.0, quiet=True)
                self._drain_rtp_probe()
                readers, _ready = self._poll_own_path()

                if readers > 0:
                    if self._stream_mode != "rtp":
                        if self._cached_sps and self._cached_pps:
                            print(
                                f"Gate: {self._gate_path} has a reader and SPS/PPS are "
                                "cached, switching to RTP mode"
                            )
                            # Persist the cached params before ffmpeg reads
                            # the SDP, and free --rtp-port for it.
                            self._write_rtp_sdp()
                            self._close_rtp_probe()
                            self._set_stream_mode("rtp")
                            if (
                                not self._stop_event.is_set()
                                and not self._wait_for_rtp_ready(self._RTP_CONFIRM_TIMEOUT)
                            ):
                                print(
                                    "Gate: RTP mode didn't become ready in time after "
                                    "the switch, falling back to placeholder (will retry)"
                                )
                                self._set_stream_mode("placeholder")
                                self._open_rtp_probe()
                        else:
                            # Nothing to do but keep waiting -- _drain_rtp_probe
                            # above is what will eventually catch GstCameraPlugin's
                            # SPS/PPS burst, whenever the camera next gets enabled.
                            # Log occasionally (--rtp-grace-period-sized interval)
                            # rather than every --gate-poll-period tick.
                            now = time.monotonic()
                            if now - last_waiting_log >= self._args.rtp_grace_period:
                                print(
                                    f"Gate: {self._gate_path} has a reader but no SPS/PPS "
                                    f"captured yet -- staying on placeholder, listening on "
                                    f"--rtp-port {self._args.rtp_port}"
                                )
                                last_waiting_log = now
                elif self._stream_mode != "placeholder":
                    # Hold rtp mode open for a full reconnect window before
                    # tearing it back down. Measured against the real stack,
                    # twice: (1) the reader that just justified switching to
                    # rtp is routinely the reader that DIES from that very
                    # switch -- killing the placeholder to start the real
                    # ffmpeg process necessarily drops whoever was attached
                    # to it; (2) mediamtx's session log shows the whole
                    # placeholder-kill -> rtp-publisher-up gap is ~4-5s
                    # (supervisor restart delay + ffmpeg startup + RTSP
                    # ANNOUNCE), during which every reconnect attempt is
                    # bounced with "no stream is available", so the reader
                    # CANNOT be back yet when the gap ends. The original
                    # single 2s-debounce-and-recheck expired inside that
                    # gap, reverted, and the next reader attach restarted
                    # the whole cycle -- cam2 flapped placeholder->rtp->
                    # placeholder indefinitely without ever serving a frame.
                    # Instead, keep polling for up to --rtp-revert-hold
                    # seconds (default 10, mirroring the camera gate's
                    # disable hold-off) and only revert if no reader has
                    # come back in all that time.
                    deadline = time.monotonic() + self._args.rtp_revert_hold
                    readers_back = False
                    while not self._stop_event.is_set() and time.monotonic() < deadline:
                        self._stop_event.wait(1.0)
                        recheck_readers, _ = self._poll_own_path()
                        if recheck_readers > 0:
                            readers_back = True
                            break
                    if readers_back or self._stop_event.is_set():
                        continue
                    print(
                        f"Gate: {self._gate_path} has had no readers for "
                        f"{self._args.rtp_revert_hold}s, reverting to placeholder"
                    )
                    self._set_stream_mode("placeholder")
                    self._open_rtp_probe()

                self._stop_event.wait(self._args.gate_poll_period)
        finally:
            self._close_rtp_probe()
        print("Gate thread stopped")

    def _ffmpeg_worker(self) -> None:
        print("FFmpeg supervisor thread started")
        while not self._stop_event.is_set():
            cmd = self._ffmpeg_command()
            try:
                if self._args.show_ffmpeg_output:
                    process = subprocess.Popen(cmd)
                else:
                    process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
            except OSError as exc:
                print(f"Failed to start ffmpeg: {exc}")
                self._stop_event.wait(self._args.ffmpeg_restart_delay)
                continue

            with self._ffmpeg_lock:
                self._ffmpeg_process = process

            print(f"Started ffmpeg (pid={process.pid})")

            while not self._stop_event.is_set():
                return_code = process.poll()
                if return_code is not None:
                    print(f"ffmpeg exited with code {return_code}, restarting")
                    break
                self._stop_event.wait(self._args.ffmpeg_monitor_period)

            if self._stop_event.is_set():
                break

            self._stop_event.wait(self._args.ffmpeg_restart_delay)

        self._terminate_ffmpeg()
        print("FFmpeg supervisor thread stopped")

    def start(self) -> None:
        self._send_heartbeat()

        heartbeat_thread = threading.Thread(
            target=self._heartbeat_worker,
            name="heartbeat-worker",
            daemon=True,
        )
        command_thread = threading.Thread(
            target=self._command_worker,
            name="command-worker",
            daemon=True,
        )
        ffmpeg_thread = threading.Thread(
            target=self._ffmpeg_worker,
            name="ffmpeg-worker",
            daemon=True,
        )
        # Only started for --video-source=rtp: 'test' mode publishes
        # unconditionally and has no reader-driven mode to gate (see module
        # docstring / _gate_worker). Keeping it unstarted in 'test' mode is
        # what makes that mode's behaviour byte-for-byte unchanged.
        gate_thread: threading.Thread | None = None
        if self._args.video_source == "rtp":
            gate_thread = threading.Thread(
                target=self._gate_worker,
                name="gate-worker",
                daemon=True,
            )

        self._camera_start()
        heartbeat_thread.start()
        command_thread.start()
        if self._args.video_source != "none":
            ffmpeg_thread.start()
        if gate_thread is not None:
            gate_thread.start()

        def _request_stop(signum, _frame) -> None:
            print(f"Received signal {signum}, stopping")
            self._stop_event.set()

        signal.signal(signal.SIGINT, _request_stop)
        signal.signal(signal.SIGTERM, _request_stop)

        try:
            while not self._stop_event.is_set():
                time.sleep(0.2)
        finally:
            self._stop_event.set()
            self._camera_stop()
            self._terminate_ffmpeg()
            heartbeat_thread.join(timeout=2.0)
            command_thread.join(timeout=max(2.0, self._args.request_timeout + 0.5))
            if ffmpeg_thread.is_alive():
                ffmpeg_thread.join(timeout=2.0)
            if gate_thread is not None:
                gate_thread.join(timeout=2.0)
            self._socket.close()


def main() -> int:
    args = parse_args()
    rtsp_uri = args.rtsp_uri or _default_rtsp_uri(args.host)
    print(
        "Starting demo video source: "
        f"udp={args.host}:{args.port}, "
        f"sysid={args.system_id}, compid={args.component_id}, "
        f"video_source={args.video_source}, "
        f"codec={args.video_codec}, "
        f"rtsp={rtsp_uri}, "
        f"notif={args.notif_uri or rtsp_uri}, "
        f"vendor={args.camera_vendor!r}, model={args.camera_model!r}"
        + (f", overlay={args.overlay_text!r}" if args.overlay_text else "")
    )
    if args.video_source == "rtp":
        print(
            "  render-gate: placeholder="
            f"{args.placeholder_width}x{args.placeholder_height}@{args.placeholder_fps}fps, "
            f"rtp_grace_period={args.rtp_grace_period}s, "
            f"gate_poll_period={args.gate_poll_period}s, "
            f"mediaserver_api_port={args.mediaserver_api_port}"
        )
    service = DemoVideoSource(args)
    service.start()
    return 0


if __name__ == "__main__":
    sys.exit(main())