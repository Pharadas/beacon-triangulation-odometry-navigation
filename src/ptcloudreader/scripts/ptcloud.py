#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from visualization_msgs import InteractiveMarker

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct

import pickle

class Point:
    def __init__(self, x, y, z, intensity, ring, time):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.ring = ring
        self.time = time

def cartesian_to_spherical(vec):
    x = vec[0]
    y = vec[1]
    z = vec[2]

    return np.array([
        np.linalg.norm(vec),
        np.arctan2(y, x),
        np.arctan2(np.sqrt(x ** 2 + y ** 2), z)
    ])

max_scans = 10
points = [[] for i in range(max_scans)]
global_counter = 0

def callback(msg):
    global points
    global global_counter

    print("gaming time")
    # publisher = rospy.Publisher('matrix_image', Float64MultiArray, queue_size=10)

    new_points = []
    retroreflector_points = []

    print(struct.iter_unpack('ffffHf', msg.data))

    points[global_counter] = new_points
    global_counter += 1
    global_counter %= max_scans
    print(global_counter)

def listener():
    topic = rospy.get_param('~topic', '/velodyne_points')
    # rospy.Subscriber(topic, PointCloud2, callback)
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    listener()

    print("saving")

    with open('velodyne_point_cloud_4.pkl', 'wb') as file:
        pickle.dump(points, file)
