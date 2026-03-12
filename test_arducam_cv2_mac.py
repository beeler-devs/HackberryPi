#!/usr/bin/env python3
"""
test_arducam_cv2_mac.py — Arducam display test on macOS.
Opens the camera BY NAME via ffmpeg's avfoundation input, completely
bypassing the unstable numeric index assignment that macOS shuffles
between sessions.  Press 'q' to quit.
"""
import cv2
import re
import sys
import subprocess
import numpy as np

CAMERA_NAME = "Arducam OV9782 USB Camera"
WIDTH = 640
HEIGHT = 480
FPS = 30
FRAME_SIZE = WIDTH * HEIGHT * 3


def get_avfoundation_devices():
    """Return dict of {index: name} from AVFoundation via ffmpeg."""
    try:
        out = subprocess.run(
            ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
            capture_output=True, text=True, timeout=10,
        )
        devices = {}
        in_video = False
        for line in out.stderr.splitlines():
            if "AVFoundation video devices" in line:
                in_video = True
                continue
            if "AVFoundation audio devices" in line:
                break
            if in_video:
                m = re.search(r"\[(\d+)\]\s+(.+)", line)
                if m:
                    devices[int(m.group(1))] = m.group(2).strip()
        return devices
    except Exception:
        return {}


def find_camera_name(preferred=CAMERA_NAME):
    """Find the exact AVFoundation device name, preferring `preferred`.

    Falls back to the first non-FaceTime, non-screen device.
    """
    devices = get_avfoundation_devices()
    if not devices:
        return None

    print("--- AVFoundation video devices ---", flush=True)
    for idx in sorted(devices):
        print(f"  [{idx}] {devices[idx]}", flush=True)
    print("", flush=True)

    # Exact match
    for name in devices.values():
        if name == preferred:
            return name

    # Substring match (e.g. "Arducam" in name)
    keyword = preferred.split()[0].lower()  # "arducam"
    for name in devices.values():
        if keyword in name.lower():
            return name

    # Any external camera (not FaceTime / screen)
    for name in devices.values():
        low = name.lower()
        if "facetime" not in low and "screen" not in low and "built-in" not in low:
            return name

    return None


def main():
    if not sys.platform.startswith("darwin"):
        print("This script is for macOS. Use test_arducam_cv2.py on Linux/Windows.", file=sys.stderr)
        sys.exit(1)

    camera = find_camera_name()
    if camera is None:
        raise RuntimeError(
            "Cannot find Arducam or any external camera. "
            "Check that the camera is connected and Camera permission is granted."
        )

    print(f"Opening '{camera}' via ffmpeg avfoundation ...", flush=True)

    # Open camera BY NAME — ffmpeg resolves the name to the current
    # AVFoundation index internally, so we never touch a numeric index.
    proc = subprocess.Popen(
        [
            "ffmpeg",
            "-f", "avfoundation",
            "-framerate", str(FPS),
            "-video_size", f"{WIDTH}x{HEIGHT}",
            "-pixel_format", "uyvy422",
            "-i", camera,
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-v", "warning",
            "-",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    print(f"Arducam display test — {WIDTH}x{HEIGHT}@{FPS} — press 'q' to quit", flush=True)

    cv2.namedWindow("Arducam (Mac)", cv2.WINDOW_NORMAL)
    cv2.moveWindow("Arducam (Mac)", 100, 100)

    frame_count = 0
    try:
        while True:
            raw = proc.stdout.read(FRAME_SIZE)
            if len(raw) != FRAME_SIZE:
                stderr = proc.stderr.read().decode(errors="replace").strip()
                if stderr:
                    print(f"ffmpeg: {stderr}", file=sys.stderr, flush=True)
                print("End of stream", file=sys.stderr, flush=True)
                break
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
            frame_count += 1
            if frame_count == 1:
                print("Displaying feed...", flush=True)
            cv2.imshow("Arducam (Mac)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        proc.wait()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
