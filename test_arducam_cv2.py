#!/usr/bin/env python3
"""
test_arducam_cv2.py — Simple Arducam display test using OpenCV.
Shows live feed from the camera. Press 'q' to quit.
"""
import cv2
import sys

WIDTH = 640
HEIGHT = 480

def try_open_camera():
    """Try multiple backends, indices, and formats to find a working camera."""
    if sys.platform.startswith("linux"):
        backends = [("V4L2", cv2.CAP_V4L2)]
    else:
        backends = [
            ("DSHOW", cv2.CAP_DSHOW),
            ("MSMF", cv2.CAP_MSMF),
            ("ANY", cv2.CAP_ANY),
        ]
    mjpeg = cv2.VideoWriter_fourcc(*'MJPG')
    for idx in range(0, 4):
        for name, backend in backends:
            print(f"  Trying index={idx}, backend={name}...", flush=True)
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue
            # Try MJPG format — many USB cameras need this to stream
            cap.set(cv2.CAP_PROP_FOURCC, mjpeg)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            # Verify we can actually grab a frame
            ret, _ = cap.read()
            if ret:
                print(f"Camera working: index={idx}, backend={name}", flush=True)
                return cap
            print(f"  Opened but can't grab frames: index={idx}, backend={name}", flush=True)
            cap.release()
    return None

def main():
    print("Opening camera...", flush=True)
    cap = try_open_camera()
    if cap is None:
        raise RuntimeError("Cannot open any camera (tried indices 0-3 with multiple backends)")
    print("Camera opened.", flush=True)

    # Use camera defaults — no resolution/FPS override to avoid MSMF format errors
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
    print(f"Arducam display test — {w}x{h} — press 'q' to quit", flush=True)

    # Prime: discard initial frames (some cameras need this)
    for _ in range(5):
        cap.read()

    cv2.namedWindow("Arducam", cv2.WINDOW_NORMAL)
    cv2.moveWindow("Arducam", 100, 100)  # Position window on-screen

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            # Retry once — transient MSMF errors can occur
            ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame", file=sys.stderr, flush=True)
            break
        frame_count += 1
        if frame_count == 1:
            print("Displaying feed...", flush=True)

        cv2.imshow("Arducam", frame)
        if cv2.waitKey(30) & 0xFF == ord('q'):  # 30ms gives window time to paint
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
