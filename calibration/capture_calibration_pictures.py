#!/usr/bin/env python

import os
import cv2
import shutil


if __name__ == '__main__':
    camera = cv2.VideoCapture(1)
    cv2.namedWindow("Frames for calibration")

    if os.path.exists('calibration_res'):
        shutil.rmtree('calibration_res')
    os.makedirs('calibration_res')

    snapshot_counter = 0
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to grab frame")
            break

        cv2.imshow("Frames for calibration", frame)

        k = cv2.waitKey(1)
        if k % 256 == 27:
            break
        elif k % 256 == 32:
            filename = 'calibration_res/calibration_frame_{}.png'.format(
                snapshot_counter)
            cv2.imwrite(filename, frame)
            print('File {} saved.'.format(filename))
            snapshot_counter += 1
