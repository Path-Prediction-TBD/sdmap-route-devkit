import osmnx as ox
from enums import Direction, RoadClass, MapObjectId
from shapely.geometry import LineString

from geopy.distance import distance
from geopy.point import Point

import warnings
from math import nan

    
class GeoRectangle:
    def __init__(self, center_of_rectangle=None, width_m=None, height_m=None):
        """
        Points are in geographic coordinates (latitude, longitude)
        :param center_of_rectangle: The center of the rectangle
        :param width_m: The width of the rectangle in m
        :param height_m: The height of the rectangle in m
        """
        if center_of_rectangle is None and width_m is None and height_m is None:
            self.center = None
            self.width_m = None
            self.height_m = None
            self.lower_left = None
            self.upper_right = None
        else:
            self.center = center_of_rectangle
            self.width_m = width_m
            self.height_m = height_m

            # Convert width and height from m to km
            width_km = width_m / 1000
            height_km = height_m / 1000

            # Calculate the lower-left corner by moving half the height south and half the width west
            lower_left_latitude = distance(kilometers=height_km / 2).destination(self.center, bearing=180).latitude
            lower_left_longitude = distance(kilometers=width_km / 2).destination(self.center, bearing=270).longitude
            self.lower_left = Point(lower_left_latitude, lower_left_longitude)

            # Calculate the upper-right corner by moving half the height north and half the width east
            upper_right_latitude = distance(kilometers=height_km / 2).destination(self.center, bearing=0).latitude
            upper_right_longitude = distance(kilometers=width_km / 2).destination(self.center, bearing=90).longitude
            self.upper_right = Point(upper_right_latitude, upper_right_longitude)

    def __repr__(self):
        return f"<GeoRectangle: lower_left={self.lower_left}, " f"upper_right={self.upper_right}>"

    # Methods to get center, lower_left, and upper_right
    def get_center(self) -> Point:
        if self.center is None:
            if self.upper_right is None or self.lower_left is None:
                raise ValueError("Center, lower_left, and upper_right are all None.")
            self.center = Point(
                (self.lower_left[0] + self.upper_right[0]) / 2, (self.lower_left[1] + self.upper_right[1]) / 2
            )
        return self.center

    def get_lower_left(self):
        return self.lower_left

    def get_upper_right(self):
        return self.upper_right

    # Methods to set lower_left and upper_right (if needed)
    def set_lower_left(self, position: Point):
        assert type(position) == Point
        self.lower_left = position

    def set_upper_right(self, position: Point):
        assert type(position) == Point
        self.upper_right = position


class Link:
    def __init__(self, id: MapObjectId, data: dict):
        assert "geometry" in data
        self.id = id
        self.geometry = data["geometry"]
        self.num_lanes = data.get("lanes", nan)
        self.length = data.get("length", nan)
        if "maxspeed" in data:
            if type(data["maxspeed"]) == list:
                max_speed = max([int(speed) for speed in data["maxspeed"]])
            else:
                max_speed = int(data["maxspeed"])
            if "mph" in data["maxspeed"]:
                max_speed = int(max_speed * 1.60934)
            self.speed_limit = max_speed
        else:
            self.speed_limit = nan
        self.highway = "highway" in data
        self.tunnel = "tunnel" in data
        self.bridge = "bridge" in data

        # Determine road class
        if "highway" in data:
            highway = data["highway"]
            if highway == "motorway":
                self.road_class = RoadClass.MOTORWAY
            elif highway == "trunk":
                self.road_class = RoadClass.TRUNK
            elif highway == "primary":
                self.road_class = RoadClass.PRIMARY
            elif highway == "secondary":
                self.road_class = RoadClass.SECONDARY
            elif highway == "tertiary":
                self.road_class = RoadClass.TERTIARY
            elif highway == "road":
                self.road_class = RoadClass.UNCLASSIFIED
            elif highway == "residential":
                self.road_class = RoadClass.RESIDENTIAL
            else:
                self.road_class = RoadClass.IGNORED

        else:
            self.road_class = RoadClass.IGNORED

    def get_ID(self) -> MapObjectId:
        return self.id

    def get_road_class(self):
        return self.road_class

    def get_pedestrian_crossings(self) -> list[Point]:
        # HERE WE NEED TO IMPLEMENT THE CROSSINGS
        return []

    def get_lane_count(self, direction: Direction) -> int:
        # HERE WE NEED TO IMPLEMENT THE DIRECTION
        return self.num_lanes

    def get_speed_limit(self, direction: Direction) -> float:
        # HERE WE NEED TO IMPLEMENT THE DIRECTION
        return self.speed_limit

    def is_highway(self) -> bool:
        return self.highway

    def is_tunnel(self) -> bool:
        return self.tunnel

    def is_bridge(self) -> bool:
        return self.bridge

    def get_length(self) -> float:
        return self.length

    def get_geometry(self) -> LineString:
        return self.geometry


class OSMWrapper:
    def __init__(self):
        pass

    def get_graph_from_point(self, lat, lon, dist=500, network_type="drive"):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", FutureWarning)
            # dist is in meters
            return ox.graph_from_point(
                (lat, lon), dist=dist, network_type=network_type, retain_all=True, truncate_by_edge=True, simplify=False
            )
            
    def project_graph(self, graph):
        return ox.project_graph(graph)

    def get_sd_object_by_id(self, link_id: MapObjectId) -> list[Link]:
        """Return a list with the only element being the Link with the matching link id"""
        if self.links is None:
            raise Exception("No links have been loaded yet")
        for link in self.links:
            if link.get_ID() == link_id:
                return [link]

    @staticmethod
    def rectangle_by_center_and_edges(
        lon_center: float, lat_center: float, width_m: float, height_m: float
    ) -> GeoRectangle:
        center_of_rectangle = Point(latitude=lat_center, longitude=lon_center)
        return GeoRectangle(center_of_rectangle, width_m, height_m)

    def get_links(self, geo_rectangle: GeoRectangle) -> list[Link]:
        graph = self.get_graph_from_point(
            geo_rectangle.get_center().latitude,
            geo_rectangle.get_center().longitude,
        )
        # projected_graph = self.project_graph(graph)
        links = []
        for u, v, data in graph.edges(data=True):
            if "geometry" not in data:
                # If there is no geometry, the edge is a straight line, but we still need to create a LineString object and add it to the data
                from_ = Point(longitude=graph.nodes[u]["x"], latitude=graph.nodes[u]["y"])
                to_ = Point(longitude=graph.nodes[v]["x"], latitude=graph.nodes[v]["y"])
                data["geometry"] = LineString([from_, to_])
            id = MapObjectId(u, v)
            link = Link(id, data)
            links.append(link)
        self.links = links
        return links
