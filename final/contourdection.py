# import cv2 as cv
# import numpy as np
# import imutils

# # https://www.codersarts.com/post/determine-color-contours-and-center-using-opencv

# image = cv.imread('image_1.jpg')

# # img_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

# # for the color yellow:
# lower = np.array([25, 100, 100])
# upper = np.array([30, 255, 255])

# yellow = cv.inRange(hsv, lower, upper)

# cnts1 = cv.findContours(yellow, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
# cnts1 = imutils.grab_contours(cnts1)

# for c in cnts1:
#     cv.drawContours(image, [c], -1, (0, 255, 0), 3)
#     # comptue center of contour
#     M = cv.moments(c)
#     if M["m00"] != 0:
#         cX = int(M["m10"]/M["m00"])
#         cY = int(M["m01"]/M["m00"])
#     else:
#         cX, cY = 0, 0
#     cv.circle(image, (cX, cY), 7, (255, 255, 255), 1)
#     cv.putText(image, "red", (cX-20, cY-20),
#                cv.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 1)

#     cv.imshow('res', image)
#     k = cv.waitKey(0)
#     if k == 27:
#         break

# https://stackoverflow.com/questions/57262974/tracking-yellow-color-object-with-opencv-python


'''
list:
# ultrasonic sensor:
# opencv (contours)
# autonomous functionality

'''

# script for ultrasonic sensor:
import numpy as np
import cv2 as cv
# from Motor import *
# from Ultrasonic import *
import time

# motor
# PWM = Motor()
# speed_forward = 1000
# speed_backward = -1000


# ultrasonic sensor:
# sensor = Ultrasonic()

# get vals from sensor:


# def ultraSensor():
#     data = sensor.get_distance()
#     print(f"obs dist: {data:.2f} CM")
#     time.sleep(0.1)
#     return data


cap = cv.VideoCapture(0)
# rgba(192,189,83,255)
# 192,189,83
while True:
    # frame = np.array(ImageGrab.grab())
    ret, img = cap.read()
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower = np.array([25, 180, 90])
    upper = np.array([30, 200, 200])

    mask = cv.inRange(hsv, lower, upper)

    cnts = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
        x, y, w, h = cv.boundingRect(c)
        cv.rectangle(img, (x, y), (x+w, y+h), (36, 255, 12), 2)

    cv.imshow('mask', mask)
    cv.imshow('img', img)

    # ultraSensor()

    if cv.waitKey(25) & 0xFF == ord('q'):
        cv.destroyAllWindows()
        break
