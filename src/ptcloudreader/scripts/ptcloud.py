#! /usr/bin/env python

from numpy.core.fromnumeric import shape
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct

def cartesian_to_spherical(vec):
    x = vec[0]
    y = vec[1]
    z = vec[2]

    return np.array([
        np.linalg.norm(vec),
        np.arctan2(y, x),
        np.arctan2(np.sqrt(x ** 2 + y ** 2), z)
    ])

last_width = 0

number_of_lines = 300
list_of_line_scans = [[] for _ in range(number_of_lines)]
global_counter = 0
should_start_writing_to_topic = False

def callback(msg):
    print("gaming time")
    # publisher = rospy.Publisher('matrix_image', Float64MultiArray, queue_size=10)
    global global_counter
    global list_of_line_scans
    global number_of_lines
    global should_start_writing_to_topic

    global last_width

    if global_counter >= number_of_lines:
        should_start_writing_to_topic = True
        global_counter = 0

    if should_start_writing_to_topic:
        matrix_image_msg = Float64MultiArray()

        xs = []
        ys = []
        zs = []
        intensities = []
        rs = []
        thetas = []
        phis = []

        N = 600
        phis_N = 300

        for line in list_of_line_scans:
            intensities.extend(line[3])
            rs.extend(line[4])
            thetas.extend(line[5])
            phis.extend(line[6])

        min_thetas = min(thetas)
        max_thetas = max(thetas)
        thetas_length = max_thetas - min_thetas

        min_phis = min(phis)
        max_phis = max(phis)
        phis_length = max_phis - min_phis

        image = np.zeros((phis_N, N))

        for i in range(len(rs)):
            if (thetas[i] > min_thetas and thetas[i] < max_thetas) and (rs[i] < 3 and rs[i] > 2):
                theta_index = int(((thetas[i] - min_thetas) / thetas_length) * N - 1)
                phi_index = int(((phis[i] - min_phis) / phis_length) * phis_N - 1)

                image[phi_index][theta_index] = rs[i]

        final_image = []

        # shouldnt do this, should make it flat from the beginning
        for i in image:
            final_image.extend(i)

        matrix_image_msg.data = final_image
        matrix_image_msg.layout.data_offset = 0
        publisher.publish(matrix_image_msg)

    xs = []
    ys = []
    zs = []
    intensities = []
    rs = []
    thetas = []
    phis = []

    for i in range(0, len(msg.data), 16):
        x_float = struct.unpack('f', msg.data[i : i+4])
        xs.append(x_float)

        y_float = struct.unpack('f', msg.data[i+4 : i+8])
        ys.append(y_float)

        z_float = struct.unpack('f', msg.data[i+8 : i+12])
        zs.append(z_float)

        intensity_float = struct.unpack('f', msg.data[i+12 : i+16])
        intensities.append(intensity_float[0])

        spherical_coords = cartesian_to_spherical(np.array([x_float, y_float, z_float]))
        rs.append(spherical_coords[0])
        thetas.append(spherical_coords[1])
        phis.append(spherical_coords[2])

    print(global_counter)
    list_of_line_scans[global_counter] = [xs, ys, zs, intensities, rs, thetas, phis]
    global_counter += 1

def listener():
    topic = rospy.get_param('~topic', '/yrl_pub/yrl_cloud')
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    listener()
    # outfile = TemporaryFile()
    # np.save(outfile, list_of_line_scans)

