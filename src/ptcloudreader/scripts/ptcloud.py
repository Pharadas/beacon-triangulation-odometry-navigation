#! /usr/bin/env python

from typing import dataclass_transform
from numpy.core.fromnumeric import shape
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct

import pickle
from dataclasses import dataclass

@dataclass
class Point:
    x: np.float32
    y: np.float32
    z: np.float32
    intensity: np.float32
    ring: np.uint16
    time: np.float32

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

max_scans = 100
points = [[] for i in range(max_scans)]
global_counter = 0

def callback(msg):
    global points
    global global_counter

    print("gaming time")
    # publisher = rospy.Publisher('matrix_image', Float64MultiArray, queue_size=10)

    new_points = []
    retroreflector_points = []

    for i in range(0, len(msg.data), 16):
        x = struct.unpack('f', msg.data[i : i+4])
        y = struct.unpack('f', msg.data[i+4 : i+8])
        z = struct.unpack('f', msg.data[i+8 : i+12])
        intensity = struct.unpack('f', msg.data[i+12 : i+16])
        ring = struct.unpack('H', msg.data[i+16 : i+18])
        time = struct.unpack('f', msg.data[i+18 : i+22])

        new_points.append(Point(x, y, z, intensity, ring, time))

    points[global_counter] = new_points
    global_counter %= max_scans

def listener():
    topic = rospy.get_param('~topic', '/yrl_pub/yrl_cloud')
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    listener()

    with open('velodyne_point_cloud_1.pkl', 'wb') as file:
        pickle.dump(points, file)
