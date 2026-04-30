"""
  "Open_Palm"    → "stop"
  "Closed_Fist"  → "walk"
  "Thumb_Down"   → "sit"
  "Thumb_Up"     → "stand"
  "Victory"      → "spin"

  use sudo fuser /dev/media* to find thing using camera
  use sudo kill -9 <#> to kill that process 
"""

import pygame
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from picamera2 import Picamera2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import urllib.request
import os
import numpy as np
from collections import deque
import cv2


# ── MediaPipe model ───────────────────────────────────────────────────────────
MODEL_PATH = "gesture_recognizer.task"
MODEL_URL  = (
    "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
    "gesture_recognizer/float16/1/gesture_recognizer.task"
)

def download_model_if_needed():
    if not os.path.exists(MODEL_PATH):
        print(f"Downloading MediaPipe gesture model to {MODEL_PATH} ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Download complete.")


# ── Gesture → command mapping ─────────────────────────────────────────────────
GESTURE_MAP = {
    "Open_Palm":   "stop",
    "Closed_Fist": "walk",
    "Thumb_Down":  "sit",
    "Thumb_Up":    "stand",
    "Victory":     "spin",
}


# ── Node ──────────────────────────────────────────────────────────────────────
class GestureRecognitionNode(Node):

    def __init__(self):
        super().__init__("gesture_recognition")

        self.publisher_ = self.create_publisher(String, "/gesture_command", 10)
        self.get_logger().info("Gesture recognition node started.")

        download_model_if_needed()

        # ── MediaPipe recognizer ──────────────────────────────────────────────
        base_options = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
        options = mp_vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=1,
        )
        self.recognizer = mp_vision.GestureRecognizer.create_from_options(options)

        # ── Pi CSI camera via picamera2 ───────────────────────────────────────
        self.cam = Picamera2()
        config = self.cam.create_preview_configuration()
        self.cam.configure(config)
        self.cam.start()
        # Let auto-exposure handle varying lighting conditions
        self.cam.set_controls({
            "AeEnable": True,
            "AwbEnable": True,
        })
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Gesture Preview")
        self.font = pygame.font.SysFont("monospace", 28)
        self._last_detected = "none"
        self.get_logger().info("Camera started.")

        # ── State ─────────────────────────────────────────────────────────────
        self._vote_buffer        = deque(maxlen=15)
        self._last_published     = None
        self._frame_timestamp_ms = 0

        # Process one frame at ~10 Hz
        self.timer = self.create_timer(1.0 / 10.0, self._process_frame)

    # ─────────────────────────────────────────────────────────────────────────

    def _process_frame(self):
        try:
            frame = self.cam.capture_array()
        except Exception as e:
            self.get_logger().warn(f"Failed to capture frame: {e}")
            return

        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = frame[:, :, :3]

        # Flip horizontally — matches MediaPipe's selfie-style training data
        frame = np.fliplr(frame)

        # Mild sharpening only — auto-exposure handles brightness/contrast
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        frame  = cv2.filter2D(frame, -1, kernel)

        h, w   = frame.shape[:2]
        cy, cx = h // 2, w // 2

        # Try two crop sizes, take whichever gives higher confidence
        best_gesture = None
        best_score   = 0.0

        for ratio in (0.75, 0.9):
            ch, cw  = int(h * ratio), int(w * ratio)
            cropped = frame[cy - ch//2 : cy + ch//2, cx - cw//2 : cx + cw//2]
            cropped = cv2.resize(cropped, (224, 224))
            cropped = np.ascontiguousarray(cropped)
            try:
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cropped)
                result   = self.recognizer.recognize_for_video(mp_image, self._frame_timestamp_ms)
                if result.gestures:
                    top = result.gestures[0][0]
                    if top.score > best_score:
                        best_score   = top.score
                        best_gesture = top.category_name if top.score >= 0.40 else None
            except Exception as e:
                self.get_logger().warn(f"Recognition error: {e}")

        self._frame_timestamp_ms += 100

        command = GESTURE_MAP.get(best_gesture) if best_gesture else None
        self._update_and_publish(command)

        if best_gesture:
            self._last_detected = f"{best_gesture} -> {command} ({best_score:.2f})"
        else:
            self._last_detected = "none"

        # Display at 0.75 crop
        ch, cw  = int(h * 0.75), int(w * 0.75)
        display = frame[cy - ch//2 : cy + ch//2, cx - cw//2 : cx + cw//2]
        surface = pygame.surfarray.make_surface(display.swapaxes(0, 1))
        surface = pygame.transform.scale(surface, (640, 480))
        self.screen.blit(surface, (0, 0))

        label = self.font.render(self._last_detected, True, (0, 255, 0))
        self.screen.blit(label, (10, 10))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pass

    def _update_and_publish(self, command):
        self._vote_buffer.append(command)

        votes = [c for c in self._vote_buffer if c is not None]
        if not votes:
            return

        winner = max(set(votes), key=votes.count)
        if votes.count(winner) >= 7 and winner != self._last_published:
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


# ── Entry point ───────────────────────────────────────────────────────────────

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
