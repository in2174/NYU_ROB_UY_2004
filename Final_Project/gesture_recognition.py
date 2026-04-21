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

# How many consecutive frames must show the same gesture before publishing.
CONFIRMATION_FRAMES = 5


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
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=1,
        )
        self.recognizer = mp_vision.GestureRecognizer.create_from_options(options)

        # ── Pi CSI camera via picamera2 ───────────────────────────────────────
        # picamera2 captures directly as RGB888 — no color conversion needed
        self.cam = Picamera2()
        config = self.cam.create_preview_configuration()
        self.cam.configure(config)
        self.cam.start()
        pygame.init()
        self.screen = pygame.display.set_mode((320,240))
        pygame.display.set_caption("Gesture Preview")
        self.font = pygame.font.SysFont("monospace", 20)
        self._last_frame = None
        self._last_detected = "none"
        self.get_logger().info("Camera started.")

        # ── State ─────────────────────────────────────────────────────────────
        self._pending_gesture    = None
        self._confirmation_count = 0
        self._last_published     = None

        # Process one frame at ~15 Hz
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


        frame    = np.ascontiguousarray(frame)
        try: 
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
            result   = self.recognizer.recognize(mp_image)

            detected = self._extract_gesture(result)
            command  = GESTURE_MAP.get(detected) if detected else None

            self._update_and_publish(command)

            if detected:
                self._last_detected = f"{detected} -> {command}"
            else:
                self._last_detected = "none"
        except Exception as e:
            self.get_logger().warn(f"Recognition error: {e}")
        
        surface = pygame.surfarray.make_surface(frame.swapaxes(0,1))
        self.screen.blit(surface, (0,0))

        label = self.font.render(self._last_detected, True, (0, 255, 0))
        self.screen.blit(label, (10,10))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                pass

    def _extract_gesture(self, result):
        """Return top gesture name if confidence >= 0.72, else None."""
        if not result.gestures:
            return None
        top = result.gestures[0][0]
        if top.score < 0.5:
            return None
        return top.category_name

    def _update_and_publish(self, command):
        """
        Only publish when the same command appears CONFIRMATION_FRAMES times
        in a row AND it differs from the last published command.
        """
        if command is None:
            self._pending_gesture    = None
            self._confirmation_count = 0
            return

        if command == self._pending_gesture:
            self._confirmation_count += 1
        else:
            self._pending_gesture    = command
            self._confirmation_count = 1

        if (self._confirmation_count >= CONFIRMATION_FRAMES
                and command != self._last_published):
            msg      = String()
            msg.data = command
            self.publisher_.publish(msg)
            self._last_published     = command
            self._confirmation_count = 0
            self.get_logger().info(f">>> Published gesture command: '{command}'")

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
