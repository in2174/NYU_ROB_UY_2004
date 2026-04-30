"""
gesture_recognition.py  ─  Pi camera → MediaPipe → /gesture_command

Publishes gesture commands as ROS2 String messages.
Runs at ~10 Hz; uses a vote buffer to suppress false positives.

Gesture map:
  Open_Palm     → "stop"
  Closed_Fist   → "walk"
  Pointing_Up   → "stand"
  Thumb_Down    → "sit"
  Victory       → "spin"

Image pipeline:
  1. Kill any process holding the camera on startup
  2. Lock camera exposure + white balance (stops autoexposure flicker during gait)
  3. Flip + centre-crop  (reduces fisheye distortion at edges)
  4. Adaptive gamma      (responds to real frame brightness, not fixed curve)
  5. CLAHE               (local contrast — helps in uneven / fishnet lighting)
  6. Sharpen             (sharpens finger edges so landmarks land more accurately)
  7. Resize to 320×320   (consistent input size for MediaPipe)

use sudo fuser /dev/media* to find thing using camera
use sudo kill -9 <#> to kill that process 
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from picamera2 import Picamera2
import pygame
import subprocess
import time
import urllib.request
import os
from collections import deque


# ─── Gesture → command map ────────────────────────────────────────────────────
GESTURE_MAP = {
    "Open_Palm":   "stop",
    "Closed_Fist": "walk",
    "Pointing_Up": "stand",
    "Thumb_Down":  "sit",
    "Victory":     "spin",
}

# ─── Camera settings ─────────────────────────────────────────────────────────
CAMERA_WIDTH   = 640
CAMERA_HEIGHT  = 480
EXPOSURE_TIME  = 10000   # microseconds  (try 8000–15000 for typical indoor)
ANALOGUE_GAIN  = 5.0     # increase if image is too dark
COLOUR_GAINS   = (1.5, 1.4)   # (red, blue) — neutral white balance

MODEL_URL  = (
    "https://storage.googleapis.com/mediapipe-models/"
    "gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task"
)
MODEL_PATH = "/tmp/gesture_recognizer.task"

# ─── Vote buffer ─────────────────────────────────────────────────────────────
VOTE_BUFFER_SIZE = 12     # number of recent frames to consider
VOTE_THRESHOLD   = 6      # need this many agreeing votes to publish
CONFIDENCE_MIN   = 0.45   # discard predictions below this score


# ═══════════════════════════════════════════════════════════════════════════════
#  Node
# ═══════════════════════════════════════════════════════════════════════════════

class GestureRecognitionNode(Node):

    def __init__(self):
        super().__init__('gesture_recognition')

        self.publisher_ = self.create_publisher(String, '/gesture_command', 10)

        # ── Download model if needed ──────────────────────────────────────────
        if not os.path.exists(MODEL_PATH):
            self.get_logger().info("Downloading MediaPipe model…")
            urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
            self.get_logger().info("Model downloaded.")

        # ── MediaPipe recognizer ──────────────────────────────────────────────
        base_opts = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
        opts      = mp_vision.GestureRecognizerOptions(
            base_options=base_opts,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=1,
        )
        self.recognizer = mp_vision.GestureRecognizer.create_from_options(opts)

        # ── Kill stale camera processes ───────────────────────────────────────
        subprocess.run(
            "sudo fuser -k /dev/media* /dev/video* 2>/dev/null",
            shell=True, check=False,
        )
        time.sleep(1.0)

        # ── Pi camera ────────────────────────────────────────────────────────
        self.cam = Picamera2()
        config   = self.cam.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
        )
        self.cam.configure(config)
        self.cam.start()
        time.sleep(0.5)   # let autoexposure settle, then lock it

        # Lock exposure + white balance so gait motion doesn't cause flicker
        self.cam.set_controls({
            "AeEnable":     False,
            "ExposureTime": EXPOSURE_TIME,
            "AnalogueGain": ANALOGUE_GAIN,
            "AwbEnable":    False,
            "ColourGains":  COLOUR_GAINS,
        })

        # ── pygame display ────────────────────────────────────────────────────
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Gesture Recognition")
        self.font = pygame.font.SysFont("monospace", 28)

        # ── State ─────────────────────────────────────────────────────────────
        self._vote_buffer        = deque(maxlen=VOTE_BUFFER_SIZE)
        self._last_published     = None
        self._frame_timestamp_ms = 0
        self._last_detected      = "none"

        # Process ~10 Hz — enough for gestures, low enough to not overwhelm Pi
        self.timer = self.create_timer(1.0 / 10.0, self._process_frame)
        self.get_logger().info("Gesture recognition ready.")

    # ─────────────────────────────────────────────────────────────────────────

    def _process_frame(self):
        # ── Capture ───────────────────────────────────────────────────────────
        try:
            frame = self.cam.capture_array()
        except Exception as e:
            self.get_logger().warn(f"Camera capture failed: {e}")
            return

        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = frame[:, :, :3]

        # ── Mirror (MediaPipe trained on selfie-mirror convention) ───────────
        frame = np.ascontiguousarray(np.fliplr(frame))

        h, w   = frame.shape[:2]
        cy, cx = h // 2, w // 2

        # ── Image enhancement ─────────────────────────────────────────────────
        # 1. Adaptive gamma
        gray            = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        mean_brightness = float(np.mean(gray))
        gamma     = float(np.interp(mean_brightness,
                                     [0,   50,  120, 200, 255],
                                     [0.25, 0.45, 1.0, 1.1, 1.2]))
        inv_gamma = 1.0 / gamma
        lut       = np.array([((i / 255.0) ** inv_gamma) * 255
                               for i in range(256)], dtype=np.uint8)
        frame = cv2.LUT(frame, lut)

        # 2. CLAHE on luminance channel
        lab     = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)
        l, a, b = cv2.split(lab)
        clahe   = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
        l       = clahe.apply(l)
        frame   = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2RGB)

        # 3. Sharpen
        kernel = np.array([[0, -1,  0],
                            [-1,  5, -1],
                            [0, -1,  0]], dtype=np.float32)
        frame = cv2.filter2D(frame, -1, kernel)
        frame = np.clip(frame, 0, 255).astype(np.uint8)

        # ── Crop and recognise ────────────────────────────────────────────────
        # Scan three overlapping centre crops — highest confidence wins.
        # Centre crops avoid worst fisheye distortion at the edges.
        best_score   = 0.0
        best_gesture = None

        for crop_ratio in (0.60, 0.70, 0.80):
            ch  = int(h * crop_ratio)
            cw  = int(w * crop_ratio)
            y0, y1 = cy - ch // 2, cy + ch // 2
            x0, x1 = cx - cw // 2, cx + cw // 2
            crop = frame[y0:y1, x0:x1]
            crop = cv2.resize(crop, (320, 320))
            crop = np.ascontiguousarray(crop)

            try:
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=crop)
                result = self.recognizer.recognize_for_video(
                    mp_img, self._frame_timestamp_ms)
                self._frame_timestamp_ms += 100

                if result.gestures:
                    top = result.gestures[0][0]
                    if top.score > best_score:
                        best_score   = top.score
                        best_gesture = (
                            top.category_name
                            if top.score >= CONFIDENCE_MIN
                            else None
                        )
            except Exception as e:
                self.get_logger().warn(
                    f"Recognition error: {e}", throttle_duration_sec=2.0)
        else:
            self._frame_timestamp_ms += 100

        # ── Vote and publish ──────────────────────────────────────────────────
        command = GESTURE_MAP.get(best_gesture) if best_gesture else None
        self._update_and_publish(command)

        if best_gesture:
            self._last_detected = f"{best_gesture} → {command} ({best_score:.2f})"
        else:
            self._last_detected = "none"

        # ── pygame display (70 % crop for preview) ───────────────────────────
        ch2  = int(h * 0.70)
        cw2  = int(w * 0.70)
        disp = frame[cy - ch2//2 : cy + ch2//2, cx - cw2//2 : cx + cw2//2]
        surf = pygame.surfarray.make_surface(disp.swapaxes(0, 1))
        surf = pygame.transform.scale(surf, (640, 480))
        self.screen.blit(surf, (0, 0))

        label  = self.font.render(self._last_detected, True, (0, 255, 0))
        label2 = self.font.render(
            f"buf: {list(self._vote_buffer)[-5:]}", True, (200, 200, 0))
        self.screen.blit(label,  (10, 10))
        self.screen.blit(label2, (10, 44))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pass

    # ─────────────────────────────────────────────────────────────────────────

    def _update_and_publish(self, command):
        self._vote_buffer.append(command)

        votes = [c for c in self._vote_buffer if c is not None]
        if not votes:
            return

        winner = max(set(votes), key=votes.count)
        if votes.count(winner) >= VOTE_THRESHOLD and winner != self._last_published:
            msg      = String()
            msg.data = winner
            self.publisher_.publish(msg)
            self._last_published = winner
            self._vote_buffer.clear()
            self.get_logger().info(f">>> Published: '{winner}'")

    def destroy_node(self):
        self.cam.stop()
        pygame.quit()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = GestureRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
