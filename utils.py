import math
from typing import Tuple
import utm

import numpy as np
from shapely.geometry import LineString
from shapely.geometry import Point as ShapelyPoint


# these are classes, not instances
from enums import MapObjectId
from osm_wrapper import Link


def global_to_vehicle_coordinates(lcm_points, oxts_latlon, oxts_heading):
    origin_x, origin_y, utm_zone_num, utm_zone_letter = utm.from_latlon(oxts_latlon[0], oxts_latlon[1])
    origin = ShapelyPoint(origin_x, origin_y)
    aft_origin = ShapelyPoint(0, 0)
    rot_pts = []

    pts = np.array(
        [
            utm.from_latlon(
                shapepoint[0],
                shapepoint[1],
                utm_zone_num,
                utm_zone_letter,
            )[:2]
            for shapepoint in lcm_points
        ]
    )

    inverse_origin = [-origin.x, -origin.y]

    # Homogeneous transformation -> translate, rotate
    for pt in pts:
        point = translate(ShapelyPoint(pt), inverse_origin)
        rot_pt = rotate(aft_origin, oxts_heading, point)
        rot_pts.append([rot_pt.x, rot_pt.y])

    return np.array(rot_pts)


def transform_to_vehicle_coordinates(vehicle_data, link):
    """Transforms the link points from lat lon to the vehicle coordinate system."""
    ego_vehicle_lat = vehicle_data["ego_vehicle_lat"]
    ego_vehicle_lon = vehicle_data["ego_vehicle_lon"]
    ego_vehicle_yaw = vehicle_data["ego_vehicle_yaw"]
    origin_x, origin_y, utm_zone_num, utm_zone_letter = utm.from_latlon(ego_vehicle_lat, ego_vehicle_lon)
    origin = ShapelyPoint(origin_x, origin_y)
    aft_origin = ShapelyPoint(0, 0)
    rot_pts = []

    lats, lons = link.get_geometry().coords.xy
    pts = np.array(
        [
            utm.from_latlon(
                lat,
                lon,
                utm_zone_num,
                utm_zone_letter,
            )[:2]
            for lat, lon in zip(lats, lons)
        ]
    )

    inverse_origin = [-origin.x, -origin.y]

    # Homogeneous transformation -> translate, rotate
    for pt in pts:
        point = translate(ShapelyPoint(pt), inverse_origin)
        rot_pt = rotate(aft_origin, ego_vehicle_yaw, point)
        rot_pts.append([rot_pt.x, rot_pt.y])
    return rot_pts


def transform_to_origin_coords(link):
    vehicle_data = {
        "ego_vehicle_lat": 0,
        "ego_vehicle_lon": 0,
        "ego_vehicle_yaw": 0,
    }
    return transform_to_vehicle_coordinates(vehicle_data, link)


def rotate(origin, angle, point):
    """
    Rotate a point around a given origin in UTM coordinates (m), with angle (degrees) in the camera frame of reference (x-axis is forward-facing, y-axis is facing left).
    """
    ox, oy = origin.x, origin.y
    px, py = point.x, point.y
    qx = ox + math.cos(np.deg2rad(angle)) * (px - ox) - math.sin(np.deg2rad(angle)) * (py - oy)
    qy = oy + math.sin(np.deg2rad(angle)) * (px - ox) + math.cos(np.deg2rad(angle)) * (py - oy)
    return ShapelyPoint(qx, qy)


def translate(point, translation):
    """
    Translate a point to the reference system of the ego_vehicle.

    :param: point, to be translated point in UTM coordinates (m).
    :param: translation,  should equal the inverse of the ego_vehicle in UTM coordinates (m).
    """
    return ShapelyPoint(
        point.x + translation[0],
        point.y + translation[1],
    )


def link_id_object(string_id):
    return MapObjectId(int(string_id.split("-")[0]), int(string_id.split("-")[1]))


def get_link_by_id(link_id, wrapper):
    return wrapper.get_sd_object_by_id(link_id_object(link_id))[0]


def convert_shapepoint_to_vehicle_coords(shapepoint, vehicle_data):
    ego_vehicle_lat = vehicle_data["ego_vehicle_lat"]
    ego_vehicle_lon = vehicle_data["ego_vehicle_lon"]
    ego_vehicle_yaw = vehicle_data["ego_vehicle_yaw"]
    origin_x, origin_y, utm_zone_num, utm_zone_letter = utm.from_latlon(ego_vehicle_lat, ego_vehicle_lon)
    origin = ShapelyPoint(origin_x, origin_y)
    aft_origin = ShapelyPoint(0, 0)
    pt = utm.from_latlon(
        shapepoint.location.get_latitude().get_value_in_degrees().get_value(),
        shapepoint.location.get_longitude().get_value_in_degrees().get_value(),
        utm_zone_num,
        utm_zone_letter,
    )[:2]

    inverse_origin = [-origin.x, -origin.y]

    # Homogeneous transformation -> translate, rotate
    point = translate(ShapelyPoint(pt), inverse_origin)
    rot_pt = rotate(aft_origin, ego_vehicle_yaw, point)
    return [rot_pt.x, rot_pt.y]


def get_link_connecting_nodes(node_1, node_2, wrapper) -> Tuple[Link, bool]:
    """
    Returns the link connecting two nodes. if the link is stored as node_1-node_2, the second return value is False, otherwise True.
    """
    link_string_1 = f"{node_1.node_id}-{node_2.node_id}"
    link_string_2 = f"{node_2.node_id}-{node_1.node_id}"
    out = []
    reversed_list = []
    try:
        l1_out = wrapper.get_sd_object_by_id(link_id_object(link_string_1))
        out.extend(l1_out)
        reversed_list.extend([False for i in range(len(l1_out))])
    except:
        pass
    try:
        l2_out = wrapper.get_sd_object_by_id(link_id_object(link_string_2))
        out.extend(l2_out)
        reversed_list.extend([True for i in range(len(l2_out))])
    except:
        pass

    if len(out) == 1:
        return out[0], reversed_list[0]
    elif len(out) > 1:
        # find shortest link
        lengths = [LineString(transform_to_origin_coords(link)).length for link in out]
        best_index = np.argmin(lengths)
        return out[best_index], reversed_list[best_index]
    else:
        return None, None
