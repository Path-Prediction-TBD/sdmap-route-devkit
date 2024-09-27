import shapely
from shapely import reverse
from shapely.ops import split, linemerge

from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

from enums import MapObjectId
from osm_wrapper import OSMWrapper
from utils import get_link_connecting_nodes, transform_to_vehicle_coordinates


class Node:
    def __init__(self, node_id: str, start_point: bool = False):
        if type(node_id) == int:
            node_id = str(node_id)    
        assert isinstance(node_id, str), f"Node ID must be an integer, not {type(node_id)}"
        self.connected_nodes = {}
        self.node_id = node_id
        self.start_point = start_point

    def __str__(self):
        return f"Node {self.node_id}"

    def __repr__(self):
        return f"Node {self.node_id}"

    def add_connection_from_map(self, node, wrapper, vehicle_data):
        current_link_obj, reversed = get_link_connecting_nodes(self, node, wrapper)
        if current_link_obj is None:
            print(f"Could not find link connecting {self.node_id} and {node.node_id}")
            return
        link_in_local_coords = transform_to_vehicle_coordinates(vehicle_data, current_link_obj)
        geometry_connection = LineString(link_in_local_coords)
        if reversed:
            geometry_connection = reverse(geometry_connection)

        self.connected_nodes[node.node_id] = geometry_connection

    def add_custom_connection(self, node, geometry_connection, vehicle_data, wrapper):
        # local_node_coords = transform_to_vehicle_coordinates(vehicle_data, node)
        # o = self.get_relative_coords(vehicle_data, wrapper)
        self.connected_nodes[node.node_id] = geometry_connection

    def get_connections(self):
        return self.connected_nodes

    def remove_connection(self, node_id):
        self.connected_nodes.pop(node_id)

    def get_relative_coords(self):
        node_coords = []
        for node_id, connection in self.connected_nodes.items():
            # check that the first point of the connection is the same for all outgoing connections
            node_coords.append(connection.coords[0])
        assert (
            len(set(node_coords)) == 1
        ), f"Node {self.node_id} has different relative coordinates for its connections."
        return node_coords[0]


class Tree:
    def __init__(self, link_ids: list[MapObjectId], wrapper: OSMWrapper, vehicle_data: dict):
        self.link_ids = link_ids
        self.wrapper = wrapper
        self.nodes: list[Node] = []
        self.vehicle_data = vehicle_data
        for link_id in self.link_ids:
            if link_id.is_loop():
                print(f"Link {link_id} is a loop, skipping.")
                continue
            node_1 = self.get_node(link_id.node_id_a)
            node_2 = self.get_node(link_id.node_id_b)
            if not node_1:
                node_1 = Node(link_id.node_id_a)
                self.nodes.append(node_1)
            if not node_2:
                node_2 = Node(link_id.node_id_b)
                self.nodes.append(node_2)
            # change later to see if bidirectional
            node_1.add_connection_from_map(node_2, self.wrapper, self.vehicle_data)
            node_2.add_connection_from_map(node_1, self.wrapper, self.vehicle_data)

    def get_node(self, node_id: str) -> Node:
        if type(node_id) == int:
            node_id = str(node_id)
        node_list = [node for node in self.nodes if node.node_id == node_id]
        if not node_list:
            return None
        return node_list[0]

    def get_start_nodes(self):
        return [node for node in self.nodes if node.start_point == True]

    def insert_start_points(self, max_distance, iter):
        """
        Find connections (LineString) where any point is closer than max_distance to the ego vehicle, call it relevant_connections.
        Find the closest point on the link to the ego vehicle (so under max_distance), call it point X.
        For each relevant_connection, break the link at point X and insert a node at that point.
        Reassign the connections of the two broken links to the new node.

        """
        ego_in_local_coords = Point([0.0, 0.0])
        connections_to_remove = []
        connections_to_add: list[tuple[Node, Node, Node, LineString, LineString]] = []
        inserted_node_id = 0
        for node in self.nodes:
            for next_node_id, connection in node.get_connections().items():
                # connection is always from node to next_node
                if (
                    set([node.node_id, next_node_id]) not in connections_to_remove
                    and connection.distance(ego_in_local_coords) < max_distance
                ):  # not yet broken and relevant
                    closest_point = nearest_points(connection, ego_in_local_coords)[0]
                    buff = closest_point.buffer(
                        0.01
                    )  # https://stackoverflow.com/questions/50194077/shapely-unable-to-split-line-on-point-due-to-precision-issues
                    try:
                        first_seg, buff_seg, last_seg = split(connection, buff).geoms
                    except ValueError:
                        # continue
                        # print(f"Could not split connection {connection} at point {closest_point}")
                        # this means that the closest point is at the start or end of the line
                        connection_start = connection.interpolate(0)
                        connection_end = connection.interpolate(connection.length)
                        if connection_start.distance(closest_point) < connection_end.distance(closest_point):
                            # closest point is at the start
                            node.start_point = True
                        else:
                            # closest point is at the end
                            next_node = self.get_node(next_node_id)
                            next_node.start_point = True
                        continue
                    connections_to_remove.append(set([node.node_id, next_node_id]))
                    # make sure the linestring is connected by changing the last point of the first segment to the first point of the last segment
                    new_first_seg = list(first_seg.coords)[:-1]
                    new_first_seg.append(list(last_seg.coords)[0])
                    first_seg = LineString(new_first_seg)
                    # create new node
                    new_node = Node(f"inserted_node_{inserted_node_id}", start_point=True)
                    inserted_node_id += 1
                    # add new node to the tree
                    # add new connections
                    next_node = self.get_node(next_node_id)
                    connections_to_add.append([node, new_node, next_node, first_seg, last_seg])

        # remove the connections that were broken
        for connection in connections_to_remove:
            node_1 = self.get_node(list(connection)[0])
            node_2 = self.get_node(list(connection)[1])
            node_1.remove_connection(node_2.node_id)
            node_2.remove_connection(node_1.node_id)

        for node, new_node, next_node, first_seg, last_seg in connections_to_add:
            self.nodes.append(new_node)
            node.add_custom_connection(new_node, first_seg, self.vehicle_data, self.wrapper)
            new_node.add_custom_connection(node, reverse(first_seg), self.vehicle_data, self.wrapper)
            new_node.add_custom_connection(next_node, last_seg, self.vehicle_data, self.wrapper)
            next_node.add_custom_connection(new_node, reverse(last_seg), self.vehicle_data, self.wrapper)

        if len(self.get_start_nodes()) == 0 and iter < 100:  # try to get any route, even if it is bad
            iter += 1
            self.insert_start_points(max_distance + 10, iter)
            if iter % 10 == 0:
                print(f"Expanded search radius to {max_distance+10} meters.")

    def find_possible_routes(self):
        self.routes = []
        self.visited_nodes_per_route = []
        for start_node in self.get_start_nodes():
            visited = set([start_node.node_id])
            self.explore_routes([], start_node, visited, 0)
        # print("Routes found:", self.routes)

    def explore_routes(self, current_route, current_node, visited, total_route_length):
        if total_route_length >= 200:
            self.routes.append(current_route)
            self.visited_nodes_per_route.append(visited)
            return
        for next_node_id, connection in current_node.get_connections().items():
            if next_node_id not in visited:
                new_route = current_route.copy()
                new_route.append(connection)
                new_total_route_length = total_route_length + connection.length
                next_node = self.get_node(next_node_id)
                new_visited = visited.copy()
                new_visited.add(next_node_id)
                self.explore_routes(new_route, next_node, new_visited, new_total_route_length)

    def get_routes_as_linestrings_2(self) -> list[tuple[Node, LineString]]:
        linestrings = []
        for route in self.routes:
            linestring = LineString()
            for node in route:
                linestring = linestring.union(node)
            linestrings.append(linemerge(linestring))
        return linestrings

    def get_routes_as_linestrings(self) -> list[tuple[Node, LineString]]:
        linestrings = []
        for route in self.routes:
            coords = []
            for i, node in enumerate(route):
                if i != 0:
                    assert all(
                        node.coords._coords[0] == coords[-1]
                    ), f"Node {node.node_id} and {route[i-1].node_id} have different relative coordinates."
                    coords.extend(node.coords._coords[1:])
                else:
                    coords.extend(node.coords._coords)
            linestrings.append(LineString(coords))

        return linestrings

    def get_routes_as_nodes(self) -> list[tuple[Node]]:
        """Only for debugging purposes."""
        node_list = [[] for i in range(len(self.routes))]
        for i, route in enumerate(self.routes):
            for linestring in route:
                start_point = linestring.coords[0]
                node_list[i].append(self.find_node_by_coords_close_by(start_point))
        return node_list

    def get_route_as_clean_nodes(self, route_index) -> list[tuple[Node]]:
        """Used to get properties of the best route. Clean means without inserted nodes."""

        node_list = []
        for linestring in self.routes[route_index]:
            start_point = linestring.coords[0]
            node_list.append(self.find_node_by_coords_close_by(start_point))
        # append the end point
        end_point = self.routes[route_index][-1].coords[-1]
        node_list.append(self.find_node_by_coords_close_by(end_point))

        # any link with inserted node has been broken to contain a start point
        # cure the link by removing the start point and adding the point on the other side of the inserted node
        new_node_list = []
        for i, node in enumerate(node_list):
            if "inserted_node" in node.node_id:
                if i == 0 or i == len(node_list) - 1:
                    # find connections
                    con = node.get_connections()
                    # find the connection that is not in the node list
                    index_other = i + 1 if i == 0 else i - 1
                    con_other = [key for key in con.keys() if key != node_list[index_other].node_id]
                    assert (
                        len(con_other) == 1
                    ), f"Starting node {node.node_id} has more than two connections, investigate."
                    the_other_connection = con_other[0]
                    new_node_list.append(self.get_node(the_other_connection))
                # don't do anything if the node is in the middle, the connection will be cured by the next node
            else:
                new_node_list.append(node)
        # determine distance along the line to the starting point of the resulting node list
        start_point = Point(self.routes[route_index][0].coords[0])
        connection = self.find_clean_connection_from_map(new_node_list[0], new_node_list[1])
        distance = shapely.line_locate_point(connection, start_point)
        assert (
            distance < connection.length
        ), f"Distance {distance} is longer than the connection length {connection.length}"
        return new_node_list, distance

    def find_clean_connection_from_map(self, node_start, node_end):
        current_link_obj, reversed = get_link_connecting_nodes(node_start, node_end, self.wrapper)
        link_in_local_coords = transform_to_vehicle_coordinates(self.vehicle_data, current_link_obj)
        geometry_connection = LineString(link_in_local_coords)
        if reversed:
            geometry_connection = reverse(geometry_connection)
        return geometry_connection

    def find_node_by_coords_close_by(self, coords, max_distance=0.1):
        closest_node = None
        closest_distance = float("inf")
        for node in self.nodes:
            distance = Point(node.get_relative_coords()).distance(Point(coords))
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node
        if closest_distance < max_distance:
            return closest_node

    def inspect_connections(self):
        """
        Make sure that each connection goes both ways.

        """
        for node in self.nodes:
            for next_node_id, connection in node.get_connections().items():
                next_node = self.get_node(next_node_id)
                if node.node_id not in next_node.get_connections():
                    print(f"Node {node.node_id} is connected to {next_node.node_id} but not the other way around.")
                assert (
                    next_node.get_connections()[node.node_id].coords[0] == connection.coords[-1]
                ), f"Node {node.node_id} and {next_node.node_id} have different relative coordinates."
                assert (
                    next_node.get_connections()[node.node_id].coords[-1] == connection.coords[0]
                ), f"Node {node.node_id} and {next_node.node_id} have different relative coordinates."
            loc = node.get_relative_coords()  # this one also has an assert
