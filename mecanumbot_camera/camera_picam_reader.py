from picamera2 import Picamera2
import cv2

class PiCamReader:
    def __init__(self, width=640, height=480, fps=30):
        self.picam2 = Picamera2()

        config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "BGR888"},
            controls={
                "FrameDurationLimits": (1000000//fps, 1000000//fps),
                "AwbEnable": True,
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

    def read(self):
        try:
            # BGR888 → OpenCV-nek tökéletes, nem kell átkonvertálni
            return self.picam2.capture_array("main")
        except Exception as e:
            print("READ ERROR:", e)
            return None

    def stop(self):
        self.picam2.stop()
