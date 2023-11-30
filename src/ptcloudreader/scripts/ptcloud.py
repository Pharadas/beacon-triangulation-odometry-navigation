#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose2D
from matplotlib import pyplot as plt

# import tf_conversions

# import tf2_ros
# import geometry_msgs.msg

import global_settings

import numpy as np
import struct

import time
import math

class LowPass:
    def __init__(self, f0, fs, adaptive):
        self.order = 2  # You can change the order if needed
        self.a = [0.0] * self.order
        self.b = [0.0] * (self.order + 1)
        self.omega0 = 6.28318530718 * f0
        self.dt = 1.0 / fs
        self.adapt = adaptive
        self.tn1 = -self.dt
        self.x = [0.0] * (self.order + 1)
        self.y = [0.0] * (self.order + 1)
        self.set_coef()

    def set_coef(self):
        if self.adapt:
            t = time.time()
            self.dt = t - self.tn1
            self.tn1 = t

        alpha = self.omega0 * self.dt
        if self.order == 1:
            self.a[0] = -(alpha - 2.0) / (alpha + 2.0)
            self.b[0] = alpha / (alpha + 2.0)
            self.b[1] = alpha / (alpha + 2.0)
        elif self.order == 2:
            alpha_sq = alpha * alpha
            beta = [1, math.sqrt(2), 1]
            D = alpha_sq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2]
            self.b[0] = alpha_sq / D
            self.b[1] = 2 * self.b[0]
            self.b[2] = self.b[0]
            self.a[0] = -(2 * alpha_sq * beta[0] - 8 * beta[2]) / D
            self.a[1] = -(beta[0] * alpha_sq - 2 * beta[1]
                          * alpha + 4 * beta[2]) / D

    def filt(self, xn):
        if self.adapt:
            self.set_coef()

        self.y[0] = 0
        self.x[0] = xn

        for k in range(self.order):
            self.y[0] += self.a[k] * self.y[k + 1] + self.b[k] * self.x[k]

        self.y[0] += self.b[self.order] * self.x[self.order]

        for k in range(self.order, 0, -1):
            self.y[k] = self.y[k - 1]
            self.x[k] = self.x[k - 1]

        return self.y[0]

class Point:
    def __init__(self, x, y, z, intensity, ring, time):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.ring = ring
        self.time = time

fs, f0 = 20, 100
filterX = LowPass(fs, f0, True)
filterY = LowPass(fs, f0, True)
filterZ = LowPass(fs, f0, True)

pub = rospy.Publisher("/visualization_marker", PointStamped, queue_size = 2)
pub_relative = rospy.Publisher("/relative_visualization_marker", PointStamped, queue_size = 2)
pub_lidar = rospy.Publisher("/visualization_marker_lidar", PointStamped, queue_size = 2)
pub_position = rospy.Publisher("/odometry_position", Pose2D, queue_size = 2)
forp = 0

def rotate_vector_2d(vec, theta):
    vec = np.array([vec[0], vec[1]])
    c, s = np.cos(theta), np.sin(theta)
    return np.matmul(np.array(((c, -s), (s, c))), vec)

def transform_frame_pose(pos):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "velodyne"
    t.child_frame_id = "map"
    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def get_best_distance(retroreflector_centers):
    pass

def round_vector(vec):
    rounded_pos = (
        round(vec[0], 1),
        round(vec[1], 1),
        round(vec[2], 1)
    )

    return rounded_pos

def cartesian_to_spherical(vec):
    x = vec[0]
    y = vec[1]
    z = vec[2]

    r = np.linalg.norm(vec)

    return np.array([
        r,
        np.arctan2(y, x),
        np.arccos(z / r)
    ])

def odometry(beaconA_relative, beaconB_relative, beaconA_absolute, beaconB_absolute):
    beaconA_relative_spherical_coords = cartesian_to_spherical(beaconA_relative)
    beaconB_relative_spherical_coords = cartesian_to_spherical(beaconB_relative)

    _, theta1, phi1 = (beaconA_relative_spherical_coords[0], beaconA_relative_spherical_coords[1], beaconA_relative_spherical_coords[2])
    _, theta2, phi2 = (beaconB_relative_spherical_coords[0], beaconB_relative_spherical_coords[1], beaconB_relative_spherical_coords[2])

    Ax, _, Az = (beaconA_absolute[0], beaconA_absolute[1], beaconA_absolute[2])
    Bx, _, Bz = (beaconB_absolute[0], beaconB_absolute[1], beaconB_absolute[2])

    up = Ax + np.tan(phi1) * np.cos(theta1) * (Bz - Az) - Bx
    down = np.sin(phi2) * np.cos(theta2) - np.tan(phi1) * np.cos(phi2) * np.cos(theta1)

    new_r2 = up / down

    B_e = np.array([
        new_r2 * np.sin(phi2) * np.cos(theta2),
        new_r2 * np.sin(phi2) * np.sin(theta2),
        new_r2 * np.cos(phi2)
    ])

    real_lidar_pos = B_e + beaconB_absolute
    return np.array(real_lidar_pos)
    # print("real lidar position", [round(real_lidar_pos[0], 1), round(real_lidar_pos[1], 1), round(real_lidar_pos[2], 1)])

# dados todos los beacons, crea un mapa de distancias nuevo
def create_new_beacons_distance_map(beacons):
    new_beacon_distance_map = {}

    for first_beacon in range(len(beacons)):
        for second_beacon in range(first_beacon, len(beacons)):
            if first_beacon != second_beacon:
                a = np.array([beacons[first_beacon][0], beacons[first_beacon][1]])
                b = np.array([beacons[second_beacon][0], beacons[second_beacon][1]])
                beacons_distance = round(np.linalg.norm(a - b), global_settings.decimals_to_round_for_map)
                # beacons_distance = distance(beacons[first_beacon], beacons[second_beacon])

                distance_exists_in_map = beacons_distance not in new_beacon_distance_map
                distance_is_valid = beacons_distance > global_settings.min_valid_distance_between_beacons

                if distance_exists_in_map and distance_is_valid:
                    new_beacon_distance_map[beacons_distance] = [beacons[first_beacon], beacons[second_beacon]]

    return new_beacon_distance_map

def get_lidar_position(
    relative_beacons,
    absolute_beacons_map
):
    # we will choose 2 random beacons and get their matching absolute beacons
    relative_beacon_A, relative_beacon_B = (relative_beacons[0], relative_beacons[1])

    # absolute_beaconA could be either relative_beaconA or relative_beaconB,
    # since we don't know, we will have to choose a new distance to find out,
    # in this case we will choose relative_beacon_A and any other that isn't relative_beacon_B
    relative_beacon_C = relative_beacons[2]

    relative_A_B_distance = round(np.linalg.norm(relative_beacon_A - relative_beacon_B), global_settings.decimals_to_round_for_map)
    relative_A_C_distance = round(np.linalg.norm(relative_beacon_A - relative_beacon_C), global_settings.decimals_to_round_for_map)

    absolute_beaconA, absolute_beaconB = absolute_beacons_map[relative_A_B_distance]
    absolute_beacons_A_C = absolute_beacons_map[relative_A_C_distance]

    '''
    relative_A_B_distance (relative_beacons) -> RA, RB (RelativeA, RelativeB)
    relative_A_B_distance (absolute_beacons) -> AA, AB (AbsoluteA, AbsoluteB)

    relative_A_C_distance (relative_beacons) -> RA, RC
    relative_A_C_distance (absolute_beacons) -> AA, AC

    we can know which relative beacon matches which absolute beacon by seeing which one
    is repeated on both distances, in this case AA and RA, this may seem obvious but
    the beacons could be in any order so this step is necessary
    '''

    repeated_beacon_absolute = None

    pivot_new = relative_beacon_A
    other_new = relative_beacon_C

    pivot_old = None
    other_old = None

    # if beaconA is in our a-c beacons, then A is our pivot (it is repeated)
    if all(absolute_beaconA == absolute_beacons_A_C[0]) or all(absolute_beaconA == absolute_beacons_A_C[1]):
        pivot_old = absolute_beaconA
        if all(absolute_beaconA == absolute_beacons_A_C[0]):
            other_old = absolute_beacons_A_C[1]
        else:
            other_old = absolute_beacons_A_C[0]

    else:
        pivot_old = absolute_beaconB
        if all(absolute_beaconB == absolute_beacons_A_C[0]):
            other_old = absolute_beacons_A_C[1]
        else:
            other_old = absolute_beacons_A_C[0]

    a = other_old - pivot_old
    b = other_new - pivot_new

    cosa_dot = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))

    # get the angle between the two 
    cosTh = np.dot(a,b)
    sinTh = np.cross(a,b)
    estimated_theta = np.arctan2(sinTh,cosTh)

    # now we can say for sure that relative_beacon_A is repeated_beacon_absolute
    # just translated, now we only have to find this translation
    # print(repeated_beacon_absolute, relative_beacon_A)
    new_relative_beacon_A = rotate_vector_2d(pivot_new, -estimated_theta)
    if all(np.isclose(pivot_old - new_relative_beacon_A, np.array([0., 0.]))):
        return True, np.array([0., 0.]), round(np.rad2deg(estimated_theta), 5)

    return True, pivot_old - new_relative_beacon_A, round(np.rad2deg(estimated_theta), 5)

# the hash function truncates the position to an integer within a range
def hash(point, mins, lengths, spaces):
    hash_x = int(((point[0] - mins[0] + 0.001) / lengths[0]) * spaces[0])
    hash_y = int(((point[1] - mins[1] + 0.001) / lengths[1]) * spaces[1])
    hash_z = int(((point[2] - mins[2] + 0.001) / lengths[2]) * spaces[2])

    return (hash_x, hash_y, hash_z)

def distance(vec1, vec2):
    return np.linalg.norm(np.array(vec1) - np.array(vec2))

# use the hashed points to quickly get all neighbors within radius
def get_all_valid_neighbors(grid, point, hashed_point, max_distance):
    valid_neighbors = []

    # the number of cubes to search as a radius
    size = int(max_distance / global_settings.discretization_size) + 1
  
    # search neighbors in a cube around the point to search
    for x in range(-size, size):
        for y in range(-size, size):
            for z in range(-size, size):
                grid_cube = (x + hashed_point[0], y + hashed_point[1], z + hashed_point[2])

                if grid_cube in grid:
                    for neighbor in grid[grid_cube]:
                        if (distance(point, neighbor) < max_distance) and (neighbor != point):
                            valid_neighbors.append(neighbor)

    return valid_neighbors

# get points from the msg array PointCloud2 and put them into
# List[Point]
def get_points_from_data_array(msg_data):
    points = []

    # check the datatype of the message 
    for i in range(0, len(msg_data), 22):
        x = struct.unpack('f', msg_data[i : i+4])[0]
        y = struct.unpack('f', msg_data[i+4 : i+8])[0]
        z = struct.unpack('f', msg_data[i+8 : i+12])[0]
        intensity = struct.unpack('f', msg_data[i+12 : i+16])[0]
        ring = struct.unpack('H', msg_data[i+16 : i+18])[0]
        time = struct.unpack('f', msg_data[i+18 : i+22])[0]

        if intensity > global_settings.min_intensity:
            points.append([x, y, z])

    return np.array(points)

# get a few necessary values to hash all points
def get_hashmap_values(points):
    scan_points = len(points)
    min_x = 10000000
    max_x = -10000000
    min_y = 10000000
    max_y = -10000000
    min_z = 10000000
    max_z = -10000000

    # get min and max values of the x, y, z and intensities
    for i in range(scan_points):
        data = [points[i][0], points[i][1], points[i][2]]

        min_x = min(min_x, data[0])
        max_x = max(max_x, data[0])
        min_y = min(min_y, data[1])
        max_y = max(max_y, data[1])
        min_z = min(min_z, data[2])
        max_z = max(max_z, data[2])

    # get lengths for the hashing algorithm
    length_x = max_x - min_x
    length_y = max_y - min_y
    length_z = max_z - min_z

    spaces_x = int(length_x / global_settings.discretization_size) + 1
    spaces_y = int(length_y / global_settings.discretization_size) + 1
    spaces_z = int(length_z / global_settings.discretization_size) + 1

    mins = (min_x, min_y, min_z)
    lengths = (length_x, length_y, length_z)
    spaces = (spaces_x, spaces_y, spaces_z)

    return mins, lengths, spaces

def callback(msg):
    # print("diferencia de tiempo entre el mensaje que estamos procesando y nuestro codigo", time.time() - float(msg.header.stamp.to_sec()))
    global marker_pub
    global forp

    then_then = time.time()

    # print("processing new message")

    then = time.time()
    points = get_points_from_data_array(msg.data)
    # print("it took ", time.time() - then, "seconds to parse the points")
    mins, lengths, spaces = get_hashmap_values(points)

    spatial_hashmap = {}

    then = time.time()
    for i in points:
        grid_pos = hash(i, mins, lengths, spaces)

        if grid_pos not in spatial_hashmap:
            spatial_hashmap[grid_pos] = []
        spatial_hashmap[grid_pos].append((i[0], i[1], i[2]))
    # print("it took ", time.time() - then, "seconds to hash all points")

    # print("SEARCHING POINT CLOUD FOR RETROREFLECTORS")

    # save all points we haven't yet found for later
    points_yet_to_find = set()

    then = time.time()
    for i in points:
        points_yet_to_find.add(tuple(i))
    # print("it took ", time.time() - then, "seconds to add all points to a set")

    to_search = set()

    retroreflectors_points = []
    retroreflector_centers_temp = []
    retroreflector_centers = []

    found_points_set = set()
    found_points = np.zeros((len(points), 3))
    found_points[0, :] = points[0]
    found_points_counter = 0

    distinct_retroreflectors_found = 0

    then = time.time()
    # keep taking random points (which arent part of a retroreflector) until all have been explored
    while len(points_yet_to_find) != 0:
        retroreflectors_points.append(set())
        retroreflector_centers_temp = [0, 0, 0]

        curr_starting_point = points_yet_to_find.pop()
        distinct_retroreflectors_found += 1
        points_searched = 0

        # print("starting again from point", curr_starting_point)
        to_search.add(curr_starting_point)

        # depth first search based on distances using a hash map to accelerate neighbors search
        while len(to_search) != 0:
            curr_node_to_search = to_search.pop()

            # add position values for center calculation
            retroreflectors_points[-1].add(curr_node_to_search)
            retroreflector_centers_temp[0] += curr_node_to_search[0]
            retroreflector_centers_temp[1] += curr_node_to_search[1]
            retroreflector_centers_temp[2] += curr_node_to_search[2]

            points_searched += 1

            if curr_node_to_search in points_yet_to_find:
                points_yet_to_find.remove(curr_node_to_search)

            hash_pos = hash(curr_node_to_search, mins, lengths, spaces)

            valid_neighbors = get_all_valid_neighbors(spatial_hashmap, curr_node_to_search, hash_pos, global_settings.max_valid_distance_between_points_of_beacons)

            for neighbor in valid_neighbors:
                if (neighbor not in found_points_set) and (neighbor not in to_search):
                    to_search.add(neighbor)
                    found_points_set.add(neighbor)
                    found_points[found_points_counter] = neighbor
                    found_points_counter += 1

        # filter out all retroreflectors too small to be considered
        # print("points searched", points_searched)
        if points_searched < global_settings.min_points_to_find_to_consider_beacon:
            distinct_retroreflectors_found -= 1
        else:
            # calculate average of position for center of the shape
            retroreflector_centers.append((
                retroreflector_centers_temp[0] / points_searched,
                retroreflector_centers_temp[1] / points_searched,
                retroreflector_centers_temp[2] / points_searched
            ))

    # print("it took ", time.time() - then, "seconds to explore the point cloud")
    # print("DISTINCT RETROREFLECTORS FOUND:", distinct_retroreflectors_found)

    # get rid of extra, unused found retroreflectors
    retroreflector_centers = retroreflector_centers[:distinct_retroreflectors_found]

    for i in range(len(retroreflector_centers)):
        # global_settings.unique_beacons_positions_ever_found.add(i)

        retroreflector_centers[i] = np.array([retroreflector_centers[i][0], retroreflector_centers[i][1]])

    # this new map should be compared to a historic (absolute) map before extending the previous one
    if global_settings.should_update_global_distances_map:
        global_settings.global_beacons_map = create_new_beacons_distance_map(retroreflector_centers)
        print(global_settings.global_beacons_map)
        print("number of distances: ", len(global_settings.global_beacons_map), "should be", (len(retroreflector_centers) * (len(retroreflector_centers) - 1)) // 2)
        # for i in global_settings.global_beacons_map:
        #     print(i, global_settings.global_beacons_map[i])
        # print(global_settings.global_beacons_map)

    # publish all my relative retroreflector centers
    for center in retroreflector_centers:
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/velodyne"
        point.point.x = center[0]
        point.point.y = center[1]
        point.point.z = 0

        pub_relative.publish(point)

    for t in global_settings.global_beacons_map:
        c = global_settings.global_beacons_map[t]

        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/velodyne"
        point.point.x = c[0][0]
        point.point.y = c[0][1]
        point.point.z = 0

        pub.publish(point)

        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/velodyne"
        point.point.x = c[1][0]
        point.point.y = c[1][1]
        point.point.z = 0

        pub.publish(point)

    # print(len(retroreflector_centers))
    if len(retroreflector_centers) >= 3:
        ret, pos = None, None

        if not global_settings.should_update_global_distances_map:
            ordered_points = list(zip(retroreflector_centers, retroreflectors_points))
            ordered_points = sorted(ordered_points, key=lambda x: len(x[1]))

            ret, pos, rotation = get_lidar_position([i[0] for i in ordered_points], global_settings.global_beacons_map)
        else:
            ret, pos, rotation = get_lidar_position(retroreflector_centers, global_settings.global_beacons_map)

        if ret:
            # print(pos)
            if not np.isnan(pos[0]) and not np.isnan(pos[1]):
                x = filterX.filt(pos[0])
                y = filterY.filt(pos[1])
                # z = filterZ.filt(pos[2])

                x_list = [-5, 5, float(pos[0])]
                y_list = [-5, 5, float(pos[1])]
                colores = ['black', 'black', 'red']

                print(x, y, rotation)

                for t in global_settings.global_beacons_map:
                    for i in global_settings.global_beacons_map[t]:
                        x_list.append(i[0])
                        y_list.append(i[1])
                        colores.append('blue')

                # plt.cla()
                # plt.scatter(x_list, y_list, c=colores)

                # plt.grid()
                # major_ticks = np.arange(-6, 6, 0.6)
                # print(major_ticks)
                # plt.xticks(major_ticks)
                # plt.yticks(major_ticks)

                # plt.axis("equal")
                # plt.draw()
                # plt.pause(0.000000001)

                # print("estimated position", x, y, z)

                # print(np.linalg.norm(np.array([x, y, z])))

                point = PointStamped()
                point.header.stamp = rospy.Time.now()
                point.header.frame_id = "/velodyne"
                point.point.x = x
                point.point.y = y
                point.point.z = 0

                pub_lidar.publish(point)

    forp += 1

    # print("total time spent in algorithm", time.time() - then_then)
            # transform_frame_pose([x, y, z])

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    topic = rospy.get_param('~topic', '/velodyne_points')
    rospy.Subscriber(topic, PointCloud2, callback, queue_size=1)

    while True:
        position = Pose2D()
        position.x = 0
        position.y = 0
        position.theta = 0
        pub_position.publish(position)

    plt.ion()
    plt.show()
    rospy.spin()

