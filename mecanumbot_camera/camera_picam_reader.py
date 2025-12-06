from picamera2 import Picamera2
import cv2

class PiCamReader:
    def __init__(self, width=640, height=480, fps=30):
        self.picam2 = Picamera2()

        config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={
                "AwbEnable": True,
                "AwbMode": 2,
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

    def read(self):
        try:
            frame = self.picam2.capture_array()
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except:
            return None

    def stop(self):
        self.picam2.stop()
