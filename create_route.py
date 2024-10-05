
import shapely
from shapely.ops import substring

import numpy as np
from shapely.geometry import LineString, Point

from enums import Direction
from osm_wrapper import OSMWrapper
from tree import Tree
from utils import convert_shapepoint_to_vehicle_coords, get_link_connecting_nodes, transform_to_vehicle_coordinates

def create_map(lat, lon, wrapper: OSMWrapper):

    side_rectangle_m = 400  #
    geo_rectangle = wrapper.rectangle_by_center_and_edges(
        lon,
        lat,
        side_rectangle_m,
        side_rectangle_m,
    )

    links_in_area = wrapper.get_links(geo_rectangle)
    links = []
    for link in links_in_area:
        links.append(link.get_ID())
    return links




def get_area_between_lines(line1: LineString, line2: LineString):
    """
    Calculate the area that is between two lines. Discretize the lines into points and calculate total distance between the points.
    """
    assert np.isclose(line1.length, line2.length, atol=200), "Lines must be 200 meters long"

    # Generate interpolation distances
    distances = np.arange(0, 200, 2)

    # Interpolate points on both lines
    line1_points = np.array([line1.interpolate(distance).coords[0] for distance in distances])
    line2_points = np.array([line2.interpolate(distance).coords[0] for distance in distances])

    # Calculate distances between corresponding points using NumPy for vectorized operations
    total_distance = np.sum(np.sqrt(np.sum((line1_points - line2_points) ** 2, axis=1)))

    return total_distance

def get_route_properties(tree: Tree, best_route_index, wrapper, vehicle_data):
    props = {}
    route_nodes, distance = tree.get_route_as_clean_nodes(
        best_route_index
    )  # nodes without inserted nodes, so OSM should have data on every link between them

    # first convert route nodes to link objects
    route_links = []
    reversed_list = []
    total_branches = 0
    has_bridge = False
    crossings = []
    total_distance = -distance
    for i, node in enumerate(route_nodes):
        if i == 0:
            continue
        link, reversed = get_link_connecting_nodes(route_nodes[i - 1], node, wrapper)
        link_in_local_coords = transform_to_vehicle_coordinates(vehicle_data, link)
        link_length = LineString(link_in_local_coords).length

        route_links.append(link)
        reversed_list.append(reversed)
        if link.is_bridge():
            has_bridge = True

        # record how far are the pedestrian crossings
        crossings_current_link = link.get_pedestrian_crossings()
        for crossing in crossings_current_link:
            geometry_connection = tree.find_clean_connection_from_map(route_nodes[i - 1], node)
            local_crossing_coords = convert_shapepoint_to_vehicle_coords(crossing, vehicle_data)
            distance_along_path = shapely.line_locate_point(geometry_connection, Point(local_crossing_coords))
            distance_from_ego = distance_along_path + total_distance
            if distance_from_ego > 0:
                crossings.append(float(distance_from_ego))

        # figure out how many branches there are, deliberately avoiding the first node
        for _, _ in node.get_connections().items():
            total_branches += 1
        total_distance += link_length

    link_we_are_on = route_links[0]
    are_we_reversed = reversed_list[0]
    props["road_class"] = link_we_are_on.get_road_class().name
    props["is_tunnel"] = link_we_are_on.is_tunnel()
    props["is_highway"] = link_we_are_on.is_highway()

    direction = Direction(are_we_reversed)
    props["num_lanes"] = link_we_are_on.get_lane_count(direction)
    speed_limit = link_we_are_on.get_speed_limit(direction)

    props["num_links"] = len(route_links)
    props["num_branches"] = total_branches
    props["has_bridge"] = has_bridge
    props["speed_limit"] = speed_limit
    props["crossings"] = crossings

    # get the number of branches
    return props

def create_route(output_dict, wrapper):
    # check if the length of the ground truth is less than 200 meters
    if (
        LineString(
            [[-lon, lat] for lat, lon in zip(output_dict["gt"]["local_lat"], output_dict["gt"]["local_lon"])]
        ).length
        < 200
    ):
        # TODO skip sample
        raise ValueError("Ground truth is less than 200 meters")

    map_links = create_map(output_dict["pred_time"]["lat"], output_dict["pred_time"]["lon"], wrapper)
    output_dict["map_data"] = map_links

    vehicle_data = {
        "ego_vehicle_lat": output_dict["pred_time"]["lat"],
        "ego_vehicle_lon": output_dict["pred_time"]["lon"],
        "ego_vehicle_yaw": output_dict["pred_time"]["heading"],
    }
    tree = Tree(map_links, wrapper, vehicle_data)
    tree.inspect_connections() # for debugging
    tree.insert_start_points(10, iter=0)
    tree.find_possible_routes()

    if not tree.routes:
        print(f"No routes found for sequence {output_dict['sequence_id']}")
        # TODO skip sample
        raise ValueError("No routes found")

    shortest_frechet_distance = float("inf")
    ground_truth_translated = [
        [lat, lon] for lat, lon in zip(output_dict["gt"]["local_lat"], output_dict["gt"]["local_lon"])
    ]
    gt_linestring = LineString(ground_truth_translated)
    gt_linestring = substring(gt_linestring, 0, 200)
    best_route_linestring = None
    best_route_index = None
    output_dict["all_route_coords"] = []
    # node_routes = tree.get_routes_as_nodes() #remove later, only for debugging
    linestrings = tree.get_routes_as_linestrings()
    if len(linestrings) == 0:
        print(f"No routes found for sequence {output_dict['sequence_id']}")
        raise ValueError("No routes found")
    for i, route_linestring in enumerate(linestrings):
        # output_dict["all_route_coords"].append(route_linestring.coords._coords.tolist())
        frechet_distance_value = get_area_between_lines(substring(route_linestring, 0, 200), gt_linestring)
        if frechet_distance_value < shortest_frechet_distance:
            shortest_frechet_distance = frechet_distance_value
            best_route_linestring = route_linestring
            best_route_index = i
    output_dict["route_coords"] = best_route_linestring.coords._coords.tolist()

    output_dict["route_properties"] = get_route_properties(tree, best_route_index, wrapper, vehicle_data)

    return output_dict