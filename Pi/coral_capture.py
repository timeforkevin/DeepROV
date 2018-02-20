import cv2
import numpy as np
import time

class NoWebCamException(Exception):
    pass

class CoralCapture:
    def __init__(self, fps, folder):
        self.camera = cv2.VideoCapture(0)
        self.fps = fps
        self.folder = folder
        if not self.camera.isOpened():
            raise NoWebCamException

    def continuous_capture(self, interval):
        while(True):
            self.single_capture(interval)

    def numbered_capture(self, count, interval):
        for i in range(count):
            self.single_capture(interval)

    def single_capture(self, interval):
        frame_width = int(self.camera.get(3))
        frame_height = int(self.camera.get(4))

        file_name = time.strftime(
            "{}/%a_%d-%m-%y_%H-%M-%S.avi.".format(self.folder), time.localtime())
        print("writing to file: {}".format(file_name))
        video_writter = cv2.VideoWriter(
            file_name,
            cv2.VideoWriter_fourcc('M','J','P','G'),
            self.fps,
            (frame_width,frame_height)
        )
        current_time = time.time()

        while(time.time() - current_time < interval):
            ret, frame = self.camera.read()
            if ret:
                video_writter.write(frame)

        video_writter.release()

    def cleanup(self):
        self.camera.release()
        cv2.destroyAllWindows()
