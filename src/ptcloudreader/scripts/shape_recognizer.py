#! /usr/bin/env python

from matplotlib.image import interpolations_names
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib

# matplotlib.use('Agg')

w = 600 
h = 300

image_recieved = np.zeros((h, w))
# plt.ion()

# # here we are creating sub plots
# figure, ax = plt.subplots(figsize=(10, 8))
# # line1, _ = ax.imshow(image_recieved)

def callback(msg):
    global image_recieved

    image_recieved = np.zeros((h, w))

    # rematrixification
    global_counter = 0
    for y in range(h):
        for x in range(w):
            image_recieved[y][x] = msg.data[global_counter]
            global_counter += 1

    image_recieved = np.uint8(image_recieved)

    # plt.imshow(image_recieved)

    # ax.clear()
    # ax.imshow(image_recieved)

    # ax.show()
    # line1.set_data(image_recieved)
    # figure.canvas.flush_events()

    # gray = cv2.cvtColor(image_recieved, cv2.COLOR_BGR2GRAY)

    # # setting threshold of gray image
    _, threshold = cv2.threshold(image_recieved.copy(), 150, 255, cv2.THRESH_OTSU)

    threshold = cv2.bitwise_not(threshold)

    kernel = np.ones((5, 5), np.uint8)
    opened = cv2.erode(threshold, kernel, iterations=7)

    # # plt.imshow(opened, 'gray')
    # # plt.show()

    # using a findContours() function
    _, contours, _ = cv2.findContours(opened, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # list for storing names of shapes
    for contour in contours[1:]:
        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.05 * cv2.arcLength(contour, True), True)

        # using drawContours() function

        x, y = (0, 0)

        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

        # putting shape name at center of each shape
        if len(approx) == 3:
            # cv2.putText(img, 'Triangle', (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            pass

        elif len(approx) == 4:
            # cv2.putText(img, 'Quadrilateral', (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            pass

        elif len(approx) == 5:
            print(x, y)
            # cv2.putText(image_recieved, 'Pentagon', (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, 10, 2)
            # cv2.drawContours(image_recieved, [contour], 0, 10, 5)

        # elif len(approx) == 6:
        #     cv2.putText(image_recieved, 'Hexagon', (x, y),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, 10, 2)
        #     cv2.drawContours(image_recieved, [contour], 0, 10, 5)

        # else:
        #     cv2.putText(image_recieved, 'circle', (x, y),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, 10, 2)

    # plt.imshow(image_recieved)
    # plt.show()

    # # displaying the image after drawing contours
    # cv2.imshow('shapes', image_recieved)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # print("gaming time")

def listener():
    topic = rospy.get_param('~topic', '/matrix_image')
    rospy.Subscriber(topic, Float64MultiArray, callback)
    # ax.imshow(image_recieved)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    plt.show()
    listener()

    scale_percent = 1000
    width = int(image_recieved.shape[1] * scale_percent / 100)
    height = int(image_recieved.shape[0] * scale_percent / 100)
    dim = (width, height)

    # image_recieved = np.uint8(image_recieved)
    # image_recieved.resize(1024, 1908)
    image_recieved = cv2.resize(image_recieved, dim, interpolation = cv2.INTER_AREA)

    # plt.imshow(image_recieved)

    # ax.clear()
    # ax.imshow(image_recieved)

    # ax.show()
    # line1.set_data(image_recieved)
    # figure.canvas.flush_events()

    # gray = cv2.cvtColor(image_recieved, cv2.COLOR_BGR2GRAY)

    # # setting threshold of gray image
    _, threshold = cv2.threshold(image_recieved.copy(), 127, 255, cv2.THRESH_OTSU)

    # plt.imshow(threshold, 'gray')
    # plt.show()

    # threshold = cv2.bitwise_not(threshold)

    kernel = np.ones((2, 2), np.uint8)
    opened = cv2.erode(threshold, kernel, iterations=0)

    opened = cv2.dilate(opened, kernel, iterations=2)

    plt.imshow(opened, 'gray')
    plt.show()

    # using a findContours() function
    _, contours, _ = cv2.findContours(opened, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # list for storing names of shapes
    for contour in contours[1:]:
        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(contour, 0.1 * cv2.arcLength(contour, True), True)

        # using drawContours() function

        x, y = (0, 0)

        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

        # putting shape name at center of each shape
        if len(approx) == 3:
            # cv2.putText(img, 'Triangle', (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            pass

        elif len(approx) == 4:
            # cv2.putText(img, 'Quadrilateral', (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            pass

        elif len(approx) == 5:
            print(x, y)
            cv2.putText(image_recieved, 'Pentagon', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2, 2)
            cv2.drawContours(image_recieved, [contour], 0, 2, 5)

        # elif len(approx) == 6:
        #     cv2.putText(image_recieved, 'Hexagon', (x, y),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, 10, 2)
        #     cv2.drawContours(image_recieved, [contour], 0, 10, 5)

        # else:
        #     cv2.putText(image_recieved, 'circle', (x, y),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, 10, 2)

    plt.imshow(image_recieved)
    plt.show()

    # # displaying the image after drawing contours
    # cv2.imshow('shapes', image_recieved)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # print("gaming time")

