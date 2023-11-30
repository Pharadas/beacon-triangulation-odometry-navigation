import numpy as np

discretization_size = 0.5
decimals_to_round_for_map = 0
max_valid_distance_between_points_of_beacons = 0.5
min_intensity = 180
min_points_to_find_to_consider_beacon = 2
min_valid_distance_between_beacons = 0.4

should_update_global_distances_map = False

global_beacons_map = {}
compared_beacons = {}

if not should_update_global_distances_map:
    global_beacons_map = {9.0: [np.array([ 1.60032623,  4.19188839]), np.array([ 0.23998955, -4.50587575])], 3.0: [np.array([ 2.54142899, -3.10625249]), np.array([ 0.23998955, -4.50587575])], 7.0: [np.array([ 1.60032623,  4.19188839]), np.array([ 2.54142899, -3.10625249])]}

unique_beacons_positions_ever_found = set()
