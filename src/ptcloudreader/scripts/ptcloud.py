#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

import numpy as np
import struct

discretization_size = 0.5
decimals_to_round_for_map = 3
global_beacons_map = {}

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
    pass

def get_lidar_position(beacons_found):
    global global_beacons_map

    # this beacon will contain both the current beacons positions and
    # the real beacons positions, it's important to mention that these
    # two beacons aren't the matching ones to the ones found in the
    # global_beacons_map
    available_beacons_to_perform_odometry = []

    # get all beacons with recognized distances
    for first_beacon in range(len(beacons_found)):
        for second_beacon in range(first_beacon, len(beacons_found)):
            rounded_beacons_distance = round(distance(beacons_found[first_beacon], beacons_found[second_beacon]), decimals_to_round_for_map)

            if rounded_beacons_distance in global_beacons_map:
                available_beacons_to_perform_odometry.append([
                    [beacons_found[first_beacon], beacons_found[second_beacon]], 
                ])

    # we need at least 3 beacons to be able to find our position
    if len(available_beacons_to_perform_odometry) < 3:
        raise Exception("Did not found at least 3 beacons to perform odometry")

    position_approximation = (0, 0, 0)

    # now we use these beacons to find our position
    # we will do an average of the found beacons to get
    # a better approximation of our position
    for beacon in range(2, len(available_beacons_to_perform_odometry)):
        beaconA = available_beacons_to_perform_odometry[beacon]
        beaconB = available_beacons_to_perform_odometry[beacon - 1]
        beaconC = available_beacons_to_perform_odometry[beacon - 2]

        A_B_rounded_beacons_distance = round(distance(beaconA, beaconB), decimals_to_round_for_map)
        A_C_rounded_beacons_distance = round(distance(beaconA, beaconC), decimals_to_round_for_map)

        real_beacons_A_B = global_beacons_map[A_B_rounded_beacons_distance]
        real_beacons_A_C = global_beacons_map[A_C_rounded_beacons_distance]

        beaconA_absolute = None
        beaconB_absolute = None
        beaconC_absolute = None

        # now we match the relative beacons to the absolute ones
        if real_beacons_A_B[0] in real_beacons_A_C: # first beacon is beaconA
            beaconA_absolute = real_beacons_A_B[0]
            beaconB_absolute = real_beacons_A_B[1]

            if real_beacons_A_B[0] == real_beacons_A_C[0]:
                beaconC_absolute = real_beacons_A_C[1]
            else:
                beaconC_absolute = real_beacons_A_C[0]

        else:
            beaconA_absolute = real_beacons_A_B[1]
            beaconB_absolute = real_beacons_A_B[0]

            if real_beacons_A_B[0] == real_beacons_A_C[0]:
                beaconC_absolute = real_beacons_A_C[1]
            else:
                beaconC_absolute = real_beacons_A_C[0]

        # ahora podemos hacer nuestra odometria varias veces y hacer un promedio
        odometry(beaconA, beaconB, beaconA_absolute, beaconB_absolute)
        odometry(beaconA, beaconC, beaconA_absolute, beaconC_absolute)
        odometry(beaconC, beaconB, beaconC_absolute, beaconB_absolute)

    return True, (0, 0, 0)

def hash(point, mins, lengths, spaces):
    hash_x = int(((point[0] - mins[0] + 0.001) / lengths[0]) * spaces[0])
    hash_y = int(((point[1] - mins[1] + 0.001) / lengths[1]) * spaces[1])
    hash_z = int(((point[2] - mins[2] + 0.001) / lengths[2]) * spaces[2])

    return (hash_x, hash_y, hash_z)

def distance(vec1, vec2):
    return np.linalg.norm(np.array(vec1) - np.array(vec2))

def get_all_valid_neighbors(grid, point, hashed_point, max_distance):
    valid_neighbors = []

    size = int(max_distance / discretization_size) + 1
    for x in range(-size, size):
        for y in range(-size, size):
            for z in range(-size, size):
                grid_cube = (x + hashed_point[0], y + hashed_point[1], z + hashed_point[2])

                if grid_cube in grid:
                    for neighbor in grid[grid_cube]:
                        if (distance(point, neighbor) < max_distance) and (neighbor != point):
                            valid_neighbors.append(neighbor)

    return valid_neighbors

class Point:
    def __init__(self, x, y, z, intensity, ring, time):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.ring = ring
        self.time = time

pub = rospy.Publisher("/visualization_marker", PointStamped, queue_size = 2)

def callback(msg):
    global marker_pub
    points = []

    for i in range(0, len(msg.data), 22):
        x = struct.unpack('f', msg.data[i : i+4])[0]
        y = struct.unpack('f', msg.data[i+4 : i+8])[0]
        z = struct.unpack('f', msg.data[i+8 : i+12])[0]
        intensity = struct.unpack('f', msg.data[i+12 : i+16])[0]
        ring = struct.unpack('H', msg.data[i+16 : i+18])[0]
        time = struct.unpack('f', msg.data[i+18 : i+22])[0]

        if intensity > 200:
            points.append(Point(x, y, z, intensity, ring, time))

    # print("Processing points")
    scan_points = len(points)

    all_points = np.zeros((scan_points, 3))
    all_points_colors = np.zeros(scan_points)

    spatial_hashmap = {}
    max_valid_distance = 0.1

    points_positions = []
    points_colors = []

    min_x = 10000000
    max_x = -10000000
    min_y = 10000000
    max_y = -10000000
    min_z = 10000000
    max_z = -10000000
    min_intensity = 10000000
    max_intensity = -10000000

    point_to_color = {}

    # filter points based on their intensity
    for i in range(scan_points):
        data = [0, 0, 0]
        data[0] = points[i].x
        data[1] = points[i].y
        data[2] = points[i].z

        # if (points[i].intensity[0]) > 200:

        min_x = min(min_x, data[0])
        max_x = max(max_x, data[0])
        min_y = min(min_y, data[1])
        max_y = max(max_y, data[1])
        min_z = min(min_z, data[2])
        max_z = max(max_z, data[2])

        min_intensity = min(min_intensity, points[i].intensity)
        max_intensity = max(max_intensity, points[i].intensity)

        points_colors.append(points[i].intensity)
        points_positions.append(data)

        point_to_color[tuple(data)] = points[i].intensity

        all_points[i][0] = data[0]
        all_points[i][1] = data[1]
        all_points[i][2] = data[2]
        all_points_colors[i] = points[i].intensity

# get lengths for the hashing algorithm
    length_x = max_x - min_x
    length_y = max_y - min_y
    length_z = max_z - min_z

    spaces_x = int(length_x / discretization_size) + 1
    spaces_y = int(length_y / discretization_size) + 1
    spaces_z = int(length_z / discretization_size) + 1

    mins = (min_x, min_y, min_z)
    lengths = (length_x, length_y, length_z)
    spaces = (spaces_x, spaces_y, spaces_z)

    for i in points_positions:
        hash_x = int(((i[0] - min_x + 0.001) / length_x) * spaces_x)
        hash_y = int(((i[1] - min_y + 0.001) / length_y) * spaces_y)
        hash_z = int(((i[2] - min_z + 0.001) / length_z) * spaces_z)

        grid_pos = (hash_x, hash_y, hash_z)
        if grid_pos in spatial_hashmap:
            spatial_hashmap[(hash_x, hash_y, hash_z)].append((i[0], i[1], i[2]))
        else:
            spatial_hashmap[(hash_x, hash_y, hash_z)] = [(i[0], i[1], i[2])]

    # print("SEARCHING POINT CLOUD FOR RETROREFLECTORS")

    # save all points we haven't yet found for later
    points_yet_to_find = set()
    for i in points_positions:
        points_yet_to_find.add(tuple(i))

    to_search = set()

    retroreflectors_points = []
    retroreflector_centers_temp = []
    retroreflector_centers = []

    found_points_set = set()
    found_points = np.zeros((len(points_positions), 3))
    found_points_colors = np.zeros(len(points_positions))
    found_points[0, :] = points_positions[0]
    found_points_counter = 0
    initial_points = np.zeros((10, 3))

    distinct_retroreflectors_found = 0

# keep taking random points (which arent part of a retroreflector) until all have been explored
    while len(points_yet_to_find) != 0:
        retroreflectors_points.append(set())
        retroreflector_centers_temp = [0, 0, 0]

        curr_starting_point = points_yet_to_find.pop()
        initial_points[distinct_retroreflectors_found] = curr_starting_point
        distinct_retroreflectors_found += 1
        points_searched = 0

        # print("starting again from point", curr_starting_point)
        to_search.add(curr_starting_point)

        # depth first search based on distances using a hash map to accelerate neighbors search
        while len(to_search) != 0:
            curr_node_to_search = to_search.pop()

            # add position values for average calculation
            retroreflectors_points[-1].add(curr_node_to_search)
            retroreflector_centers_temp[0] += curr_node_to_search[0]
            retroreflector_centers_temp[1] += curr_node_to_search[1]
            retroreflector_centers_temp[2] += curr_node_to_search[2]

            points_searched += 1

            if curr_node_to_search in points_yet_to_find:
                points_yet_to_find.remove(curr_node_to_search)

            hash_pos = hash(curr_node_to_search, mins, lengths, spaces)

            valid_neighbors = get_all_valid_neighbors(spatial_hashmap, curr_node_to_search, hash_pos, max_valid_distance)

            for neighbor in valid_neighbors:
                if (neighbor not in found_points_set) and (neighbor not in to_search):
                    to_search.add(neighbor)
                    found_points_set.add(neighbor)
                    found_points[found_points_counter] = neighbor
                    found_points_colors[found_points_counter] = point_to_color[neighbor] / 255
                    found_points_counter += 1

        # filter out all retroreflectors too small to be considered
        if points_searched < 4:
            distinct_retroreflectors_found -= 1

        # calculate average of position for center of the shape
        retroreflector_centers.append((
            retroreflector_centers_temp[0] / points_searched,
            retroreflector_centers_temp[1] / points_searched,
            retroreflector_centers_temp[2] / points_searched
        ))

    print("DISTINCT RETROREFLECTORS FOUND:", distinct_retroreflectors_found)

    # get rid of extra, unused found retroreflectors
    retroreflector_centers = retroreflector_centers[:distinct_retroreflectors_found]

    for i in retroreflector_centers:
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/velodyne"
        point.point.x = i[0]
        point.point.y = i[1]
        point.point.z = i[2]

        pub.publish(point)

    # for retroreflector in retroreflector_centers:

def listener():
    topic = rospy.get_param('~topic', '/velodyne_points')
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    listener()

    # print("saving")
