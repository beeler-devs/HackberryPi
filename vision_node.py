#!/usr/bin/env python3
"""
vision_node.py — Jetson Nano Vision Node
Captures frames at 100fps via Arducam OV9782, detects Aimlabs targets via
HSV color thresholding, and transmits X-axis centroid + crosshair coordinates
over UDP to the Raspberry Pi control node (Y-axis is user-controlled).

Latency priorities:
  - MJPEG over V4L2 to minimize USB bandwidth
  - Pre-allocated numpy buffers (no per-frame malloc)
  - Dedicated capture thread with shallow drop-on-full queue
  - Non-blocking UDP send (silent drop on failure)
"""

import cv2
import numpy as np
import socket
import struct
import threading
import queue
import time
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler

try:
    import Jetson.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    _HAS_GPIO = False

# ── Camera ───────────────────────────────────────────────────────────────────
CAMERA_INDEX      = 0        # /dev/video0  (change if camera is on another index)
CAPTURE_WIDTH     = 640      # pixels; use 1280 for higher accuracy at FPS cost
CAPTURE_HEIGHT    = 480      # pixels; use 720 for higher accuracy at FPS cost
CAPTURE_FPS       = 100      # OV9782 max; driver must negotiate MJPEG at this rate
# V4L2 manual exposure value in device-specific units.
# 100 ≈ 1ms on most V4L2 drivers (check with v4l2-ctl --list-ctrls on your device).
# Decrease to reduce motion blur; increase if image is too dark. If the overlay stream
# looks black with only the red crosshair visible, raise this (e.g. 500–2000 for indoors).
EXPOSURE_VALUE    = 800

# ── Static crosshair ─────────────────────────────────────────────────────────
# Physical center of your monitor as seen by the camera (in pixels).
# Measure once by overlaying a crosshair image and noting the pixel coords.
CROSSHAIR_X       = 320.0

# ── HSV target color bounds ───────────────────────────────────────────────────
# Aimlabs default target color is cyan/teal. OpenCV HSV: H[0,179] S[0,255] V[0,255].
# To calibrate: open a live HSV view (debug mode), sample the target, then tighten
# these bounds around the observed cluster.
# Widen H range if targets are missed; narrow S/V range to reject monitor glare.
HSV_LOWER         = np.array([88,  150, 150], dtype=np.uint8)  # (H_min, S_min, V_min)
HSV_UPPER         = np.array([97,  240, 240], dtype=np.uint8)  # (H_max, S_max, V_max)

# ── Morphological noise reduction ─────────────────────────────────────────────
# Erode kills isolated noise pixels; dilate restores blob size.
# Larger kernel = stronger noise rejection but slower and blurs fine edges.
MORPH_KERNEL_SIZE = 5   # pixels; odd number; 3 is fastest, 7 for heavy noise
MORPH_ERODE_ITER  = 1   # iterations; increase for more aggressive noise rejection
MORPH_DILATE_ITER = 2   # dilate more than erode to fill holes inside targets

# ── Contour filtering ─────────────────────────────────────────────────────────
MIN_CONTOUR_AREA  = 50      # px²; discard specular noise below this
MAX_CONTOUR_AREA  = 40000   # px²; discard accidental large colored regions

# ── Target selection strategy ─────────────────────────────────────────────────
# 'weighted': area-weighted centroid across ALL valid contours.
#             Best when target blob fragments — X tracks center of mass.
# 'closest': pick contour whose centroid is nearest to the crosshair.
#            Best for Aimlabs (targets always near center; ignores UI chrome).
# 'largest': pick contour with maximum area.
#            Faster but may jump to a large off-center blob.
TARGET_STRATEGY   = 'weighted'

# ── Detection mode ───────────────────────────────────────────────────────
# 'hsv':  HSV color thresholding (Aimlabs cyan targets, runs on Jetson w/ Arducam)
# 'yolo': YOLO11n neural network (Valorant enemy detection, runs on laptop w/ GPU)
DETECTION_MODE    = 'hsv'

# ── Capture source ───────────────────────────────────────────────────────
# 'camera': Arducam via V4L2 (Jetson Nano)
# 'screen': Desktop screen capture via mss (laptop running Valorant)
CAPTURE_SOURCE    = 'camera'
SCREEN_REGION     = None   # None = full primary monitor, or {'left': x, 'top': y, 'width': w, 'height': h}

# ── YOLO settings (only used when DETECTION_MODE == 'yolo') ──────────────
YOLO_MODEL_PATH   = 'lib/NeuromuscularAimAssist/training_results/detect/valorant_train/run_a100/weights/best.pt'
YOLO_CONF_THRESH  = 0.25   # minimum confidence to accept a detection
YOLO_IOU_THRESH   = 0.45   # NMS IOU threshold
YOLO_HALF         = True   # FP16 inference (faster on GPU, set False for CPU-only)
YOLO_IMG_SIZE     = 640    # YOLO internal resize (matches training size)

# ── Network ───────────────────────────────────────────────────────────────────
UDP_TARGET_IP     = "10.0.0.2"   # Raspberry Pi IP on the direct Ethernet link
UDP_TARGET_PORT   = 5005
UDP_SOURCE_IP     = "10.0.0.1"  # Jetson: 10.0.0.1, Laptop: 10.0.0.3 (or "0.0.0.0" to auto-select)

# ── Threading ─────────────────────────────────────────────────────────────────
# Keep shallow: depth=2 means we hold at most one "in-flight" frame.
# Increasing this adds latency without throughput benefit.
CAPTURE_QUEUE_MAXSIZE = 2

# ── Solenoid trigger (Jetson GPIO) ────────────────────────────────────────────
SOLENOID_ENABLED    = False   # set True to fire solenoid from the Jetson
SOLENOID_PIN        = 18      # Jetson GPIO pin (BOARD numbering)
FIRE_THRESHOLD_PX   = 5.0     # fire when |tx - cx| < this (pixels)
TRIGGER_MIN_MS      = 80      # minimum solenoid dwell time (ms)
TRIGGER_RANGE_MS    = 20      # random additional dwell (ms)

# ── Debug (disable in production — imshow costs ~5ms per frame) ───────────────
DEBUG_SHOW = False   # set True to open an OpenCV window showing the HSV mask

# ── Overlay stream (view CV overlay in browser on Mac/other device) ───────────
# When True, serves MJPEG at http://<jetson-ip>:STREAM_PORT — open in browser to see overlay.
STREAM_OVERLAY = True
STREAM_PORT   = 8080
STREAM_FPS    = 15   # max overlay frames per second (lower = less bandwidth)
STREAM_JPEG_QUALITY = 85  # 1–100; lower = smaller frames, faster

# Shared state for overlay stream: latest JPEG bytes + lock (written by VisionProcessor).
_stream_overlay_buffer = {"jpeg": None, "lock": threading.Lock()}

# ── Solenoid trigger state ────────────────────────────────────────────────────
import random as _random
_solenoid_active = threading.Event()   # set = solenoid currently firing

def _solenoid_init():
    """Initialize GPIO for solenoid output. Call once at startup."""
    if not _HAS_GPIO:
        print("[WARN] Jetson.GPIO not available — solenoid disabled", file=sys.stderr)
        return False
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SOLENOID_PIN, GPIO.OUT, initial=GPIO.LOW)
    return True

def _solenoid_fire():
    """Fire the solenoid in a detached thread (non-blocking, debounced)."""
    if _solenoid_active.is_set():
        return
    _solenoid_active.set()

    def _pulse():
        try:
            GPIO.output(SOLENOID_PIN, GPIO.HIGH)
            dwell_ms = TRIGGER_MIN_MS + _random.randint(0, TRIGGER_RANGE_MS)
            time.sleep(dwell_ms / 1000.0)
            GPIO.output(SOLENOID_PIN, GPIO.LOW)
            print(f"[DEBUG] Solenoid released (dwell={dwell_ms} ms)")
        finally:
            _solenoid_active.clear()

    threading.Thread(target=_pulse, daemon=True, name="SolenoidPulse").start()

def _solenoid_cleanup():
    """Release GPIO resources."""
    if _HAS_GPIO:
        try:
            GPIO.output(SOLENOID_PIN, GPIO.LOW)
            GPIO.cleanup(SOLENOID_PIN)
        except Exception:
            pass

# ─────────────────────────────────────────────────────────────────────────────
class FrameGrabber(threading.Thread):
    """
    Dedicated camera capture thread.
    Decouples camera I/O from vision processing so the processor never stalls
    waiting for the next frame — it always sees the most recent captured frame.
    """

    def __init__(self, cap_queue: queue.Queue) -> None:
        super().__init__(daemon=True, name="FrameGrabber")
        self._queue = cap_queue
        self._stop_event = threading.Event()

        # Open with V4L2 backend explicitly — avoids GStreamer/FFMPEG buffering.
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {CAMERA_INDEX}")

        # MJPEG is mandatory: raw YUV at 100fps@640x480 = ~147 MB/s, far above USB 2.0.
        # MJPEG compresses to ~5 MB/s, well within the 60 MB/s USB 2.0 ceiling.
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          CAPTURE_FPS)

        # Disable auto-exposure BEFORE setting manual value.
        # V4L2 enum: 1 = manual, 3 = aperture-priority (auto).
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE,      EXPOSURE_VALUE)

        # Verify negotiated parameters — log mismatches but continue.
        actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        if (actual_w, actual_h) != (CAPTURE_WIDTH, CAPTURE_HEIGHT):
            print(f"[WARN] Camera negotiated {actual_w}x{actual_h} "
                  f"(requested {CAPTURE_WIDTH}x{CAPTURE_HEIGHT})", file=sys.stderr)
        if abs(actual_fps - CAPTURE_FPS) > 5:
            print(f"[WARN] Camera negotiated {actual_fps:.1f}fps "
                  f"(requested {CAPTURE_FPS}fps)", file=sys.stderr)

        self._cap = cap

    def run(self) -> None:
        cap = self._cap
        q   = self._queue
        while not self._stop_event.is_set():
            ret, frame = cap.read()   # hardware-timed block; returns at ~10ms intervals
            if not ret:
                continue
            try:
                # put_nowait: raises queue.Full if at capacity.
                # We discard rather than accumulate to bound latency.
                q.put_nowait(frame)
            except queue.Full:
                pass   # newest frame will replace on next successful put

    def stop(self) -> None:
        self._stop_event.set()
        self._cap.release()


# ─────────────────────────────────────────────────────────────────────────────
class ScreenGrabber(threading.Thread):
    """
    Desktop screen capture thread for laptop-based YOLO mode.
    Uses mss to grab screenshots and feeds them into the same queue
    interface as FrameGrabber, so the rest of the pipeline is identical.
    """

    def __init__(self, cap_queue: queue.Queue) -> None:
        super().__init__(daemon=True, name="ScreenGrabber")
        self._queue = cap_queue
        self._stop_event = threading.Event()

        import mss
        self._sct = mss.mss()
        self._region = SCREEN_REGION or self._sct.monitors[1]  # primary monitor

        actual_w = self._region['width']
        actual_h = self._region['height']
        print(f"[INFO] Screen capture: {actual_w}x{actual_h} "
              f"-> resize to {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}")

    def run(self) -> None:
        sct = self._sct
        region = self._region
        q = self._queue
        while not self._stop_event.is_set():
            img = sct.grab(region)
            # mss returns BGRA; drop alpha channel to get BGR for OpenCV
            frame = np.array(img)[:, :, :3]
            if frame.shape[1] != CAPTURE_WIDTH or frame.shape[0] != CAPTURE_HEIGHT:
                frame = cv2.resize(frame, (CAPTURE_WIDTH, CAPTURE_HEIGHT))
            try:
                q.put_nowait(frame)
            except queue.Full:
                pass

    def stop(self) -> None:
        self._stop_event.set()


# ─────────────────────────────────────────────────────────────────────────────
class VisionProcessor:
    """
    Consumes frames from the capture queue, runs detection (HSV or YOLO),
    and transmits target centroids over UDP.
    """

    def __init__(self, cap_queue: queue.Queue) -> None:
        self._queue = cap_queue
        self._running = True

        if DETECTION_MODE == 'yolo':
            from ultralytics import YOLO
            self._yolo_model = YOLO(YOLO_MODEL_PATH)
            print(f"[INFO] YOLO model loaded: {YOLO_MODEL_PATH}")
        else:
            # Pre-allocate output buffers once. Passing dst= to OpenCV functions
            # avoids a malloc+memset per call — critical at 100fps.
            self._hsv_buf  = np.empty((CAPTURE_HEIGHT, CAPTURE_WIDTH, 3), dtype=np.uint8)
            self._mask_buf = np.empty((CAPTURE_HEIGHT, CAPTURE_WIDTH),    dtype=np.uint8)

            # Structuring element for morphological ops — built once, reused every frame.
            self._kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
            )

        # UDP socket — setblocking(False) makes sendto non-blocking without a timeout.
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((UDP_SOURCE_IP, 0))  # Force traffic over Ethernet
        self._sock.setblocking(False)
        self._dest = (UDP_TARGET_IP, UDP_TARGET_PORT)

        # Static crosshair X position for distance computation.
        self._cx = CROSSHAIR_X

        # Solenoid GPIO init (Jetson-side trigger).
        self._solenoid_ready = False
        if SOLENOID_ENABLED:
            self._solenoid_ready = _solenoid_init()

    # ── Core vision pipeline ──────────────────────────────────────────────────
    def process_frame(self, frame: np.ndarray):
        """Dispatch to the active detection backend."""
        if DETECTION_MODE == 'yolo':
            return self._process_frame_yolo(frame)
        return self._process_frame_hsv(frame)

    def _process_frame_yolo(self, frame: np.ndarray):
        """
        Run YOLO11n inference on a BGR frame.
        Returns target_x (center of largest detection) or None.
        """
        results = self._yolo_model(
            frame, conf=YOLO_CONF_THRESH, iou=YOLO_IOU_THRESH,
            verbose=False, half=YOLO_HALF, imgsz=YOLO_IMG_SIZE
        )
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None

        xyxy = boxes.xyxy.cpu().numpy()  # (N, 4): x1, y1, x2, y2
        areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
        best_idx = areas.argmax()
        x1, _, x2, _ = xyxy[best_idx]
        return (x1 + x2) / 2.0

    def _process_frame_hsv(self, frame: np.ndarray):
        """
        Run HSV color thresholding on a BGR frame.
        Returns target_x in pixels, or None if no target found.
        All operations use pre-allocated buffers to avoid heap allocation.
        """
        # Step 1: BGR -> HSV in-place. dst= reuses self._hsv_buf allocation.
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV, dst=self._hsv_buf)

        # Step 2: Threshold to isolate target color. dst= avoids another allocation.
        cv2.inRange(self._hsv_buf, HSV_LOWER, HSV_UPPER, dst=self._mask_buf)

        # Step 3: Morphological opening (erode->dilate) removes noise specks.
        cv2.erode(self._mask_buf,  self._kernel, dst=self._mask_buf,
                  iterations=MORPH_ERODE_ITER)
        cv2.dilate(self._mask_buf, self._kernel, dst=self._mask_buf,
                   iterations=MORPH_DILATE_ITER)

        # Step 4: Find external contours only (no hierarchy needed = faster).
        contours, _ = cv2.findContours(
            self._mask_buf,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        # Step 5: Filter by area to reject noise and large false positives.
        valid = [c for c in contours
                 if MIN_CONTOUR_AREA < cv2.contourArea(c) < MAX_CONTOUR_AREA]
        if not valid:
            return None

        # Step 6: Select target X by configured strategy.
        if TARGET_STRATEGY == 'weighted':
            total_m00 = 0.0
            total_m10 = 0.0
            for c in valid:
                M = cv2.moments(c)
                total_m00 += M['m00']
                total_m10 += M['m10']
            if total_m00 == 0:
                return None
            tx = total_m10 / total_m00
        elif TARGET_STRATEGY == 'closest':
            def sq_dist(c):
                M = cv2.moments(c)
                if M['m00'] == 0:
                    return float('inf')
                px = M['m10'] / M['m00']
                return (px - self._cx) ** 2
            best = min(valid, key=sq_dist)
            M = cv2.moments(best)
            if M['m00'] == 0:
                return None
            tx = M['m10'] / M['m00']
        else:
            best = max(valid, key=cv2.contourArea)
            M = cv2.moments(best)
            if M['m00'] == 0:
                return None
            tx = M['m10'] / M['m00']
        return tx

    # ── Main processing loop ──────────────────────────────────────────────────
    def run(self) -> None:
        """
        Consume frames from queue, detect targets, transmit UDP packets.
        Runs on the main thread (or call from a dedicated thread if preferred).
        """
        while self._running:
            # Block up to 50ms for a frame; prevents busy-spin if camera stalls.
            try:
                frame = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue

            result = self.process_frame(frame)

            # Optional: build overlay and feed MJPEG stream for browser viewing (e.g. on Mac).
            if STREAM_OVERLAY:
                overlay = frame.copy()
                mid_y = CAPTURE_HEIGHT // 2
                cv2.line(overlay, (int(self._cx), 0), (int(self._cx), CAPTURE_HEIGHT), (0, 0, 255), 2)
                if result is not None:
                    tx = result
                    cv2.circle(overlay, (int(tx), mid_y), 8, (0, 255, 0), 2)
                # Small mask thumbnail in corner (HSV mode only)
                if DETECTION_MODE == 'hsv':
                    small = cv2.resize(self._mask_buf, (160, 120))
                    small_bgr = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
                    overlay[0:120, 0:160] = small_bgr
                _, jpeg = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
                with _stream_overlay_buffer["lock"]:
                    _stream_overlay_buffer["jpeg"] = jpeg.tobytes()

            # Only transmit when a target is detected.
            # Sending stale "last known" position would cause the servo to hold
            # on a phantom location after a target disappears.
            if result is None:
                if DEBUG_SHOW and DETECTION_MODE == 'hsv':
                    cv2.imshow("mask", self._mask_buf)
                    cv2.waitKey(1)
                continue

            tx = result
            ts = time.monotonic()   # seconds; used by Pi for staleness check

            # Pack as big-endian binary: 8+4+4 = 16 bytes total.
            # Format: double timestamp | float tx | float cx
            # (cx = crosshair static X center; tx = target centroid X)
            packet = struct.pack(
                '!dff',
                ts,
                tx,           # target centroid X
                self._cx      # crosshair reference X (static)
            )
            try:
                self._sock.sendto(packet, self._dest)
                self._send_count = getattr(self, '_send_count', 0) + 1
                if self._send_count == 1 or self._send_count % 100 == 0:
                    print(f"[UDP] sent pkt #{self._send_count}: tx={tx:.1f} cx={self._cx:.1f} ts={ts:.3f}")
            except OSError as e:
                self._fail_count = getattr(self, '_fail_count', 0) + 1
                if self._fail_count <= 3 or self._fail_count % 100 == 0:
                    print(f"[UDP] send failed (#{self._fail_count}): {e}")

            # ── Solenoid trigger (Jetson-side) ────────────────────────────────
            if SOLENOID_ENABLED and self._solenoid_ready:
                pixel_dist = abs(tx - self._cx)
                if pixel_dist < FIRE_THRESHOLD_PX:
                    _solenoid_fire()

            if DEBUG_SHOW:
                if DETECTION_MODE == 'hsv':
                    dbg = cv2.cvtColor(self._mask_buf, cv2.COLOR_GRAY2BGR)
                else:
                    dbg = frame.copy()
                mid_y = CAPTURE_HEIGHT // 2
                cv2.circle(dbg, (int(tx), mid_y), 5, (0, 255, 0), -1)
                cv2.circle(dbg, (int(self._cx), mid_y), 3, (0, 0, 255), -1)
                cv2.imshow("debug", dbg)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._running = False

    def stop(self) -> None:
        self._running = False
        self._sock.close()
        if self._solenoid_ready:
            _solenoid_cleanup()


# ─────────────────────────────────────────────────────────────────────────────
class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """Serves the latest overlay frame as MJPEG (multipart/x-mixed-replace) for browser viewing."""

    def do_GET(self) -> None:
        if self.path == "/health":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"OK\n")
            return
        if self.path != "/" and self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
        interval = 1.0 / STREAM_FPS if STREAM_FPS > 0 else 1.0 / 15
        buf = _stream_overlay_buffer
        try:
            while True:
                with buf["lock"]:
                    jpeg = buf["jpeg"]
                if jpeg is not None:
                    try:
                        self.wfile.write(boundary % len(jpeg))
                        self.wfile.write(jpeg)
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, format: str, *args) -> None:
        pass  # suppress per-request logs


def _run_stream_server(port: int) -> None:
    try:
        server = HTTPServer(("0.0.0.0", port), MJPEGStreamHandler)
        print(f"[INFO] Overlay stream server listening on 0.0.0.0:{port} (try http://<jetson-ip>:{port}/ or /health)")
        server.serve_forever()
    except OSError as e:
        print(f"[WARN] Overlay stream server failed to start: {e}", file=sys.stderr)


# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    cap_queue = queue.Queue(maxsize=CAPTURE_QUEUE_MAXSIZE)

    if CAPTURE_SOURCE == 'screen':
        grabber = ScreenGrabber(cap_queue)
    else:
        grabber = FrameGrabber(cap_queue)
    processor = VisionProcessor(cap_queue)

    grabber.start()

    stream_server_thread = None
    if STREAM_OVERLAY:
        stream_server_thread = threading.Thread(
            target=_run_stream_server,
            args=(STREAM_PORT,),
            daemon=True,
            name="StreamServer",
        )
        stream_server_thread.start()

    print(f"[INFO] Vision node started — streaming to {UDP_TARGET_IP}:{UDP_TARGET_PORT}")
    print(f"[INFO] Detection: {DETECTION_MODE}, capture: {CAPTURE_SOURCE}")
    if DETECTION_MODE == 'yolo':
        print(f"[INFO] YOLO model: {YOLO_MODEL_PATH} (conf={YOLO_CONF_THRESH}, iou={YOLO_IOU_THRESH})")
    else:
        print(f"[INFO] Camera: {CAPTURE_WIDTH}x{CAPTURE_HEIGHT} @ {CAPTURE_FPS}fps MJPEG")
        print(f"[INFO] HSV bounds: lower={tuple(HSV_LOWER)}  upper={tuple(HSV_UPPER)}")
    if DEBUG_SHOW:
        print("[INFO] Debug display enabled (press 'q' to quit)")
    if STREAM_OVERLAY:
        print(f"[INFO] Overlay stream: http://<this-machine-ip>:{STREAM_PORT}/ or /stream")

    try:
        processor.run()   # blocks until processor.stop() or KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        processor.stop()
        grabber.stop()
        grabber.join(timeout=2.0)
        if DEBUG_SHOW:
            cv2.destroyAllWindows()
        print("[INFO] Vision node stopped.")


if __name__ == "__main__":
    main()
