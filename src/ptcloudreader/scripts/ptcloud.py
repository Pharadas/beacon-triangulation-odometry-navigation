#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct

import pickle

discretization_size = 0.5

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

def callback(msg):
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

    print("Processing points")
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
    maxs = (max_x, max_y, max_z)
    lengths = (length_x, length_y, length_z)
    spaces = (spaces_x, spaces_y, spaces_z)

    print("min x:", min_x)
    print("max x:", max_x)
    print("min y:", min_y)
    print("max y:", max_y)
    print("min z:", min_z)
    print("max z:", max_z)

    print(spaces_x, spaces_y, spaces_z)

    for i in points_positions:
        hash_x = int(((i[0] - min_x + 0.001) / length_x) * spaces_x)
        hash_y = int(((i[1] - min_y + 0.001) / length_y) * spaces_y)
        hash_z = int(((i[2] - min_z + 0.001) / length_z) * spaces_z)

        grid_pos = (hash_x, hash_y, hash_z)
        if grid_pos in spatial_hashmap:
            spatial_hashmap[(hash_x, hash_y, hash_z)].append((i[0], i[1], i[2]))
        else:
            spatial_hashmap[(hash_x, hash_y, hash_z)] = [(i[0], i[1], i[2])]

    print("SEARCHING POINT CLOUD FOR RETROREFLECTORS")

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

        print("starting again from point", curr_starting_point)
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
    retroreflector_centers_np_arr = np.zeros((len(retroreflector_centers), 3))

    for i in range(len(retroreflector_centers)):
        retroreflector_centers_np_arr[i] = [retroreflector_centers[i][0], retroreflector_centers[i][1], retroreflector_centers[i][2]]

    retroreflectors_points = retroreflectors_points[:distinct_retroreflectors_found]

# find all beacons by searching up and down every retroreflector
    distinct_beacons_points = []
    beacons_centers = np.zeros((10, 3))
    current_searched_beacon = 0

# we will be consuming the retroreflector point sets, so we will just check if there's at least one left
    while len(retroreflectors_points) != 0:

        distinct_beacons_points.append(set())

        # search up above the retroreflector
        current_retroreflector_points = retroreflectors_points.pop()
        current_retroreflector_center = retroreflector_centers.pop()

        # print("retroreflector points", current_retroreflector_points)

        # we begin our beacon considering all retroreflector points
        distinct_beacons_points[-1].update(current_retroreflector_points)
        # print("beacon points", distinct_beacons_points[-1])

        # we will search a position above our current
        search_range = 0.8

        hash_pos = hash(current_retroreflector_center, mins, lengths, spaces)
        points_around = get_all_valid_neighbors(spatial_hashmap, current_retroreflector_center, hash_pos, search_range) # verify this distance is good

        last_position = current_retroreflector_center
        should_continue_searching = True

        # while we are still finding retroreflectors above the current points
        while should_continue_searching:
            point_to_compare = points_around[0]
            found_valid_continuation = False

            # check in which of our retroreflectors the points is
            i = 0
            while i < len(retroreflectors_points):
                if point_to_compare in retroreflectors_points[i]:
                    found_valid_continuation = True
                    # we found a set which contains the one of the points above our past retroreflector
                    # we should consume the whole retroreflector and make a single beacon
                    distinct_beacons_points[-1].union(retroreflectors_points.pop(i))
                    temp_retroreflector_center = retroreflector_centers.pop(i)
                    print(current_retroreflector_center, temp_retroreflector_center, distance(current_retroreflector_center, temp_retroreflector_center))

                    new_retroreflector_center = (
                        current_retroreflector_center[0] + temp_retroreflector_center[0],
                        current_retroreflector_center[1] + temp_retroreflector_center[1],
                        current_retroreflector_center[2] + temp_retroreflector_center[2]
                    )
                else:
                    i += 1

            should_continue_searching = found_valid_continuation
            last_position = point_to_compare

            hash_pos = hash(point_to_compare, mins, lengths, spaces)
            points_around = get_all_valid_neighbors(spatial_hashmap, point_to_compare, hash_pos, search_range) # TODO verify this distance is good

        beacons_centers[current_searched_beacon] = current_retroreflector_center
        current_searched_beacon += 1

    beacons_centers = beacons_centers[:current_searched_beacon]
    beacons_spherical_coords = np.zeros((len(beacons_centers), 3))

    print("BEACONS FOUND", len(beacons_centers))

def listener():
    topic = rospy.get_param('~topic', '/velodyne_points')
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    listener()

    print("saving")
