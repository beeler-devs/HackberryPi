#!/usr/bin/env python3
"""
vision_node.py — Jetson Nano Vision Node
Captures frames at 100fps via Arducam OV9782, detects Aimlabs targets via
HSV color thresholding, and transmits centroid + crosshair coordinates over UDP
to the Raspberry Pi control node.

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

# ── Camera ───────────────────────────────────────────────────────────────────
CAMERA_INDEX      = 0        # /dev/video0  (change if camera is on another index)
CAPTURE_WIDTH     = 640      # pixels; use 1280 for higher accuracy at FPS cost
CAPTURE_HEIGHT    = 480      # pixels; use 720 for higher accuracy at FPS cost
CAPTURE_FPS       = 100      # OV9782 max; driver must negotiate MJPEG at this rate
# V4L2 manual exposure value in device-specific units.
# 100 ≈ 1ms on most V4L2 drivers (check with v4l2-ctl --list-ctrls on your device).
# Decrease to reduce motion blur; increase if image is too dark.
EXPOSURE_VALUE    = 100

# ── Static crosshair ─────────────────────────────────────────────────────────
# Physical center of your monitor as seen by the camera (in pixels).
# Measure once by overlaying a crosshair image and noting the pixel coords.
CROSSHAIR_X       = 320.0
CROSSHAIR_Y       = 240.0

# ── HSV target color bounds ───────────────────────────────────────────────────
# Aimlabs default target color is cyan/teal. OpenCV HSV: H[0,179] S[0,255] V[0,255].
# To calibrate: open a live HSV view (debug mode), sample the target, then tighten
# these bounds around the observed cluster.
# Widen H range if targets are missed; narrow S/V range to reject monitor glare.
HSV_LOWER         = np.array([85,  120, 120], dtype=np.uint8)  # (H_min, S_min, V_min)
HSV_UPPER         = np.array([100, 255, 255], dtype=np.uint8)  # (H_max, S_max, V_max)

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
# 'closest': pick contour whose centroid is nearest to the crosshair.
#            Best for Aimlabs (targets always near center; ignores UI chrome).
# 'largest': pick contour with maximum area.
#            Faster but may jump to a large off-center blob.
TARGET_STRATEGY   = 'closest'

# ── Network ───────────────────────────────────────────────────────────────────
UDP_TARGET_IP     = "192.168.1.50"   # Raspberry Pi IP on the direct Ethernet link
UDP_TARGET_PORT   = 5005

# ── Threading ─────────────────────────────────────────────────────────────────
# Keep shallow: depth=2 means we hold at most one "in-flight" frame.
# Increasing this adds latency without throughput benefit.
CAPTURE_QUEUE_MAXSIZE = 2

# ── Debug (disable in production — imshow costs ~5ms per frame) ───────────────
DEBUG_SHOW = False   # set True to open an OpenCV window showing the HSV mask


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
class VisionProcessor:
    """
    Consumes frames from the capture queue, runs HSV color detection,
    and transmits target centroids over UDP.
    """

    def __init__(self, cap_queue: queue.Queue) -> None:
        self._queue = cap_queue
        self._running = True

        # Pre-allocate output buffers once. Passing dst= to OpenCV functions
        # avoids a malloc+memset per call — critical at 100fps.
        self._hsv_buf  = np.empty((CAPTURE_HEIGHT, CAPTURE_WIDTH, 3), dtype=np.uint8)
        self._mask_buf = np.empty((CAPTURE_HEIGHT, CAPTURE_WIDTH),    dtype=np.uint8)

        # Structuring element for morphological ops — built once, reused every frame.
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )

        # UDP socket — non-blocking; sendto drops silently on timeout.
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.001)   # 1ms timeout; drop rather than stall
        self._dest = (UDP_TARGET_IP, UDP_TARGET_PORT)

        # Static crosshair as numpy arrays for fast distance computation.
        self._cx = CROSSHAIR_X
        self._cy = CROSSHAIR_Y

    # ── Core vision pipeline ──────────────────────────────────────────────────
    def process_frame(self, frame: np.ndarray):
        """
        Run the full detection pipeline on a single BGR frame.
        Returns (target_x, target_y) in pixels, or None if no target found.
        All operations use pre-allocated buffers to avoid heap allocation.
        """
        # Step 1: BGR → HSV in-place. dst= reuses self._hsv_buf allocation.
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV, dst=self._hsv_buf)

        # Step 2: Threshold to isolate target color. dst= avoids another allocation.
        cv2.inRange(self._hsv_buf, HSV_LOWER, HSV_UPPER, dst=self._mask_buf)

        # Step 3: Morphological opening (erode→dilate) removes noise specks.
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

        # Step 6: Select best contour by configured strategy.
        if TARGET_STRATEGY == 'closest':
            # Squared distance avoids sqrt — order is preserved for min().
            def sq_dist(c):
                M = cv2.moments(c)
                if M['m00'] == 0:
                    return float('inf')
                px = M['m10'] / M['m00']
                py = M['m01'] / M['m00']
                return (px - self._cx) ** 2 + (py - self._cy) ** 2
            best = min(valid, key=sq_dist)
        else:
            # 'largest': fastest; fine when only one target is visible at a time.
            best = max(valid, key=cv2.contourArea)

        # Step 7: Compute centroid via image moments.
        M = cv2.moments(best)
        if M['m00'] == 0:
            return None   # degenerate contour (all pixels on one line)
        tx = M['m10'] / M['m00']
        ty = M['m01'] / M['m00']
        return (tx, ty)

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

            # Only transmit when a target is detected.
            # Sending stale "last known" position would cause the servo to hold
            # on a phantom location after a target disappears.
            if result is None:
                if DEBUG_SHOW:
                    cv2.imshow("mask", self._mask_buf)
                    cv2.waitKey(1)
                continue

            tx, ty = result
            ts = time.monotonic()   # seconds; used by Pi for staleness check

            # Pack as big-endian binary: 8+4+4+4+4 = 24 bytes total.
            # Format: double timestamp | float cx | float cy | float tx | float ty
            # (cx/cy = crosshair static center; tx/ty = target centroid)
            packet = struct.pack(
                '!dffff',
                ts,
                tx, ty,                  # target centroid
                self._cx, self._cy       # crosshair reference (static)
            )
            try:
                self._sock.sendto(packet, self._dest)
            except OSError:
                pass   # timeout or network error — drop and continue

            if DEBUG_SHOW:
                # Draw crosshair and target on the mask for visual calibration.
                dbg = cv2.cvtColor(self._mask_buf, cv2.COLOR_GRAY2BGR)
                cv2.circle(dbg, (int(tx), int(ty)), 5, (0, 255, 0), -1)
                cv2.circle(dbg, (int(self._cx), int(self._cy)), 3, (0, 0, 255), -1)
                cv2.imshow("mask", dbg)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._running = False

    def stop(self) -> None:
        self._running = False
        self._sock.close()


# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    cap_queue = queue.Queue(maxsize=CAPTURE_QUEUE_MAXSIZE)

    grabber   = FrameGrabber(cap_queue)
    processor = VisionProcessor(cap_queue)

    grabber.start()
    print(f"[INFO] Vision node started — streaming to {UDP_TARGET_IP}:{UDP_TARGET_PORT}")
    print(f"[INFO] Camera: {CAPTURE_WIDTH}x{CAPTURE_HEIGHT} @ {CAPTURE_FPS}fps MJPEG")
    print(f"[INFO] HSV bounds: lower={tuple(HSV_LOWER)}  upper={tuple(HSV_UPPER)}")
    if DEBUG_SHOW:
        print("[INFO] Debug display enabled (press 'q' to quit)")

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
