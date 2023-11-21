discretization_size = 0.5
decimals_to_round_for_map = 1
max_valid_distance_between_points_of_beacons = 0.2
min_intensity = 230
min_points_to_find_to_consider_beacon = 4
min_valid_distance_between_beacons = 0.1

should_update_global_distances_map = True

global_beacons_map = {}
compared_beacons = {}

if not should_update_global_distances_map:
    global_beacons_map = {1.5: [(2.6, 0.8, 0.4), (1.8, 2.0, 0.6000000000000001)], 1.3: [(2.6, 0.8, 0.4), (1.8, 1.8, 0.6000000000000001)], 3.4: [(2.2, -1.4, 0.4), (1.8, 2.0, 0.6000000000000001)], 3.2: [(2.2, -1.4, 0.4), (1.8, 1.8, 0.6000000000000001)], 2.2: [(2.2, -1.4, 0.4), (2.6, 0.8, 0.4)]}

unique_beacons_positions_ever_found = set()
