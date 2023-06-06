# this code works only on the raspberry
import numpy as np
import cv2 as cv
from picamera2 import Picamera2
import sys
from Motor import *
import math
import time

PWM = Motor()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()


while True:
    frame = picam2.capture_array()
    # Our operations on the frame come here
#     gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
# [reduce_noise]
    # Reduce the noise to avoid false circle detection
#     gray = cv.medianBlur(gray, 5)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    lower_blue = np.array([60, 35, 140])
    upper_blue = np.array([180, 255, 255])

    mask = cv.inRange(hsv, lower_blue, upper_blue)

    result = cv.bitwise_and(frame, frame, mask=mask)

    hsv_test = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    show_test = cv.medianBlur(hsv_test, 5)

#     hsv = cv.medianBlur(hsv, 5)

    # [houghcircles]
    rows = show_test.shape[0]
    circles = cv.HoughCircles(show_test, cv.HOUGH_GRADIENT, 1, rows / 8,
                              param1=100, param2=30,
                              minRadius=50, maxRadius=200)
    # [draw]
    if circles is not None:
        print("Found a circle")
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            try:
                center = (i[0], i[1])
    #             print(f'i[0] = {i[0]}')
    #            print(f'i[1] = {i[1]}')
                print(f"Center value: {center}")
                # circle center
                cv.circle(show_test, center, 1, (2, 61, 163), 4)
                # circle outline
                radius = i[2]
                print(f"Radius: {radius}")
                # print(f'i[2] = {i[2]}')
                # print(f'radius = {radius}')
                cv.circle(show_test, center, radius, (255, 0, 255), 5)
                # "size": (640, 480)
                k_w = 25
                k_v = 500
                # angular velocity
                # center = (x=i[0], y=i[1])
                # distance = math.sqrt((i[0]**2))
                # print(f"Distance: {distance}")
                err_dist = i[0] - 320
                # print(f"err_dist = {err_dist}")
                # w = k_w * (0 - err_dist) * -1

                # if i[0] < 320:
                # err_dist = i[0] - 320
                # else:
                #   err_dist =320- i[0]
                # print(f"err_dist = {err_dist}")
                w = k_w * (0 - err_dist)

                # linear velocity
                # r_d => btwn 150-170
                r_d = 100
                r_diff = (r_d - radius)
                # v = k_v * r_diff
                v = 0
                print(f"V = {v}")
                print(f"W = {w}")
                u = np.array([v - w, v + w])
                u[u > 1500] = 1500
                u[u < -1500] = -1500

                print(f"U val: {u}")
                print(f"U[0] = {u[0]}")
                print(f"U[1] = {u[1]}")
                PWM.setMotorModel(u[0], u[0], u[1], u[1])
                time.sleep(0.1)
#                 if radius > 80:
#                     print("Close enough, motors have stopped")
#                     PWM.setMotorModel(0,0,0,0)
            except KeyboardInterrupt:
                PWM.setMotorModel(0, 0, 0, 0)
                sys.exit("\n Exiting Program")

    else:
        print("No Circles Found")
        PWM.setMotorModel(0, 0, 0, 0)

    # [display]
#     cv.imshow('frame', frame)
#     cv.imshow('mask',mask)
    cv.imshow("detected circles", show_test)
    cv.waitKey(1)


# When everything done, release the capture
cv.destroyAllWindows()
