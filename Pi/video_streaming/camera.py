#!/usr/bin/python
# -*- coding: utf-8 -*-
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import time
import cv2
import numpy as np


class VideoCamera(object):

    def __init__(self, camera_num):
        self.vs = VideoStream(src=camera_num).start()
        self.fps = FPS().start()
        self.classes = [
            'background',
            'aeroplane',
            'bicycle',
            'bird',
            'boat',
            'bottle',
            'bus',
            'car',
            'cat',
            'chair',
            'cow',
            'diningtable',
            'dog',
            'horse',
            'motorbike',
            'person',
            'pottedplant',
            'sheep',
            'sofa',
            'train',
            'tvmonitor'
        ]
        # rows = open('synset_words.txt').read().strip().split('\n')
        # self.classes = [r[r.find(' ') + 1:].split(',')[0] for r in rows]
        # print(self.classes)
        self.colors = np.random.uniform(0, 255,
                size=(len(self.classes), 3))
        self.net = \
            cv2.dnn.readNetFromCaffe('MobileNetSSD_deploy.prototxt.txt'
                , 'MobileNetSSD_deploy.caffemodel')
        self.confidence = 0.2

    def __del__(self):
        self.fps.stop()
        self.vs.stop()


    def get_frame(self):
        frame = self.vs.read()
        self.fps.update()
        (ret, jpeg) = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()

    def analyze_frame(self):
        frame = self.vs.read()
        self.fps.update()
        frame = imutils.resize(frame, width=400)

        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                0.007843, (300, 300), 127.5)

        self.net.setInput(blob)
        detections = self.net.forward()

        # loop over the detections

        for i in np.arange(0, detections.shape[2]):

            # extract the confidence (i.e., probability) associated with
            # the prediction

            confidence = detections[0, 0, i, 2]

            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence

            if confidence > self.confidence:

                # extract the index of the class label from the
                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object

                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype('int')

                # draw the prediction on the frame

                label = '{}: {:.2f}%'.format(self.classes[idx],
                        confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY),
                              self.colors[idx], 2)
                y = (startY - 15 if startY - 15 > 15 else startY + 15)
                cv2.putText(
                    frame,
                    label,
                    (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    self.colors[idx],
                    2,
                    )


        (ret, jpeg) = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()
