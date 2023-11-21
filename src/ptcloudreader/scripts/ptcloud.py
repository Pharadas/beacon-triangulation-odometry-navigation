#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PointStamped

import global_settings

import numpy as np
import struct

class Point:
    def __init__(self, x, y, z, intensity, ring, time):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.ring = ring
        self.time = time

pub = rospy.Publisher("/visualization_marker", PointStamped, queue_size = 2)
pub_lidar = rospy.Publisher("/visualization_marker_lidar", PointStamped, queue_size = 2)

def relative_to_absolute_beacons_translation(
    relative_beacons,
    absolute_beacons_map,
):
    # we will choose 2 random beacons and get their matching absolute beacons
    relative_beacon_A, relative_beacon_B  = (relative_beacons[0], relative_beacons[1])

    relative_A_B_distance = round(np.linalg.norm(relative_beacon_A - relative_beacon_B), global_settings.decimals_to_round_for_map)
    absolute_beaconA, absolute_beaconB = absolute_beacons_map[relative_A_B_distance]

    # absolute_beaconA could be either relative_beaconA or relative_beaconB,
    # since we don't know, we will have to choose a new distance to find out,
    # in this case we will choose relative_beacon_A and any other that isn't relative_beacon_B
    relative_beacon_C = relative_beacons[2]

    relative_A_C_distance = round(np.linalg.norm(relative_beacon_A - relative_beacon_C), global_settings.decimals_to_round_for_map)

    absolute_beacons_A_C = absolute_beacons_map[relative_A_C_distance]

    # print("19283102983")
    # print(relative_beacon_A)
    # print(absolute_beaconA, absolute_beacons_A_C)
    # print("19283102983")

    # relative_beacons_A_C = relative_beacons_map[relative_A_C_distance]

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

    # get the beacon that's repeated in from the absolute beacons
    if all(absolute_beaconA == absolute_beacons_A_C[0]) or all(absolute_beaconA == absolute_beacons_A_C[1]):
        repeated_beacon_absolute = absolute_beaconA
    else:
        repeated_beacon_absolute = absolute_beaconB

    # now we can say for sure that relative_beacon_A is repeated_beacon_absolute
    # just translated, now we only have to find this translation
    # print(repeated_beacon_absolute, relative_beacon_A)
    return repeated_beacon_absolute - relative_beacon_A

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
                beacons_distance = distance(beacons[first_beacon], beacons[second_beacon])

                if beacons_distance not in new_beacon_distance_map and beacons_distance > global_settings.min_valid_distance_between_beacons:
                    new_beacon_distance_map[beacons_distance] = [beacons[first_beacon], beacons[second_beacon]]

    return new_beacon_distance_map

def get_lidar_position(beacons_found):
    # this beacon will contain both the current beacons positions and
    # the real beacons positions, it's important to mention that these
    # two beacons aren't the matching onelv to the ones found in the
    # global_beacons_map
    available_beacons_to_perform_odometry = []

    # we need at least 3 beacons to be able to find our position
    if len(available_beacons_to_perform_odometry) < 2:
        print("Did not find at least 3 beacons to perform odometry")
        return False, (0, 0, 0)

    position_approximation = np.zeros(3)

    # now we use these beacons to find our position
    # we will do an average of the found beacons to get
    # a better approximation of our position
    t = 0
    for beacon in range(2, len(available_beacons_to_perform_odometry)):
        t += 3
        beaconA = available_beacons_to_perform_odometry[beacon]
        beaconB = available_beacons_to_perform_odometry[beacon - 1]
        beaconC = available_beacons_to_perform_odometry[beacon - 2]

        # print(beaconA, beaconB)

        A_B_rounded_beacons_distance = round(distance(beaconA, beaconB), global_settings.decimals_to_round_for_map)
        A_C_rounded_beacons_distance = round(distance(beaconA, beaconC), global_settings.decimals_to_round_for_map)

        if A_B_rounded_beacons_distance < global_settings.max_valid_distance_between_points_of_beacons or A_C_rounded_beacons_distance < global_settings.max_valid_distance_between_points_of_beacons:
            return False, (0, 0, 0)

        real_beacons_A_B = global_settings.global_beacons_map[A_B_rounded_beacons_distance]
        real_beacons_A_C = global_settings.global_beacons_map[A_C_rounded_beacons_distance]

        # print("absolute:", real_beacons_A_B)
        # print("relative:", [beaconA, beaconB])

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

        # point = PointStamped()
        # point.header.stamp = rospy.Time.now()
        # point.header.frame_id = "/velodyne"
        # point.point.x = round(beaconA_absolute[0], 1)
        # point.point.y = round(beaconA_absolute[1], 1)
        # point.point.z = round(beaconA_absolute[2], 1)

        # pub.publish(point)

        # point = PointStamped()
        # point.header.stamp = rospy.Time.now()
        # point.header.frame_id = "/velodyne"
        # point.point.x = round(beaconB_absolute[0], 1)
        # point.point.y = round(beaconB_absolute[1], 1)
        # point.point.z = round(beaconB_absolute[2], 1)

        # pub.publish(point)

        # point = PointStamped()
        # point.header.stamp = rospy.Time.now()
        # point.header.frame_id = "/velodyne"
        # point.point.x = round(beaconC_absolute[0], 1)
        # point.point.y = round(beaconC_absolute[1], 1)
        # point.point.z = round(beaconC_absolute[2], 1)

        # pub.publish(point)

        # ahora podemos hacer nuestra odometria varias veces y hacer un promedio
        position_approximation += odometry(beaconA, beaconB, beaconA_absolute, beaconB_absolute)
        # position_approximation += odometry(beaconA, beaconC, beaconA_absolute, beaconC_absolute)
        # position_approximation += odometry(beaconC, beaconB, beaconC_absolute, beaconB_absolute)

    # position_approximation /= t

    return True, position_approximation

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
    global marker_pub

    points = get_points_from_data_array(msg.data)
    mins, lengths, spaces = get_hashmap_values(points)

    spatial_hashmap = {}

    for i in points:
        grid_pos = hash(i, mins, lengths, spaces)

        if grid_pos in spatial_hashmap:
            spatial_hashmap[grid_pos].append((i[0], i[1], i[2]))
        else:
            spatial_hashmap[grid_pos] = [(i[0], i[1], i[2])]

    # print("SEARCHING POINT CLOUD FOR RETROREFLECTORS")

    # save all points we haven't yet found for later
    points_yet_to_find = set()
    for i in points:
        points_yet_to_find.add(tuple(i))

    to_search = set()

    retroreflectors_points = []
    retroreflector_centers_temp = []
    retroreflector_centers = []

    found_points_set = set()
    found_points = np.zeros((len(points), 3))
    found_points[0, :] = points[0]
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

            valid_neighbors = get_all_valid_neighbors(spatial_hashmap, curr_node_to_search, hash_pos, global_settings.max_valid_distance_between_points_of_beacons)

            for neighbor in valid_neighbors:
                if (neighbor not in found_points_set) and (neighbor not in to_search):
                    to_search.add(neighbor)
                    found_points_set.add(neighbor)
                    found_points[found_points_counter] = neighbor
                    found_points_counter += 1

        # filter out all retroreflectors too small to be considered
        if points_searched < global_settings.min_points_to_find_to_consider_beacon:
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

    for i in range(len(retroreflector_centers)):
        # global_settings.unique_beacons_positions_ever_found.add(i)

        retroreflector_centers[i] = round_vector(retroreflector_centers[i])

        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/velodyne"
        point.point.x = retroreflector_centers[i][0]
        point.point.y = retroreflector_centers[i][1]
        point.point.z = retroreflector_centers[i][2]

        pub.publish(point)

    global_distances_map = create_new_beacons_distance_map(retroreflector_centers)

    for i in global_distances_map:
        print(i, global_distances_map[i])

    # if global_settings.should_update_global_distances_map:
    #     for ai in global_settings.unique_beacons_positions_ever_found:
    #         for bi in global_settings.unique_beacons_positions_ever_found:
    #             if ai != bi:
    #                 # beaconA = (retroreflector_centers[ai][0], retroreflector_centers[ai][1])
    #                 # beaconB = (retroreflector_centers[bi][0], retroreflector_centers[bi][1])

    #                 rounded_beacons_distance = round(distance(ai, bi), global_settings.decimals_to_round_for_map)
    #                 distance_is_already_recorded = rounded_beacons_distance not in global_settings.global_beacons_map
    #                 distance_is_not_too_small = rounded_beacons_distance > global_settings.max_valid_distance_between_points_of_beacons

    #                 if distance_is_already_recorded and distance_is_not_too_small and (ai, bi) not in global_settings.compared_beacons and (bi, ai) not in global_settings.compared_beacons:
    #                     global_settings.global_beacons_map[rounded_beacons_distance] = [ai, bi]
    #                     global_settings.compared_beacons[(ai, bi)] = rounded_beacons_distance

    # # print(retroreflector_centers)
    # result, pos = get_lidar_position(retroreflector_centers)

    # print(
    #     round(pos[0], 1),
    #     round(pos[1], 1),
    #     round(pos[2], 1)
    # )

    # point = PointStamped()
    # point.header.stamp = rospy.Time.now()
    # point.header.frame_id = "/velodyne"
    # point.point.x = pos[0]
    # point.point.y = pos[1]
    # point.point.z = pos[2]

    # pub_lidar.publish(point)

def listener():
    topic = rospy.get_param('~topic', '/velodyne_points')
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    listener()

    # print("saving")
