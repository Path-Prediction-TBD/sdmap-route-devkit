import math
import os
import pickle
from matplotlib import pyplot as plt
import numpy as np
from shapely.geometry import LineString
from shapely.ops import nearest_points

from osm_wrapper import GeoRectangle, Link, OSMWrapper
from shapely.geometry import Point as ShapelyPoint
from geopy.point import Point
import utm

from utils import link_id_object, transform_to_vehicle_coordinates



def get_links_around_egovehicle(vehicle_data, wrapper) -> list:
    """Retrieves the links around the vehicle location."""
    map_extent = [-150, -50, 150, 175]
    sorted_map_extent = sorted([abs(extent) for extent in map_extent])
    # to account for all possible vehicle yaws
    max_dist = math.sqrt(sorted_map_extent[-1]**2 + sorted_map_extent[-2]**2)

    origin_x, origin_y, utm_zone_num, utm_zone_letter = utm.from_latlon(vehicle_data["ego_vehicle_lat"], vehicle_data["ego_vehicle_lon"])
    bl = ShapelyPoint(origin_x - max_dist, origin_y - max_dist)
    tr = ShapelyPoint(origin_x + max_dist, origin_y + max_dist)

    bl_latlon = Point(utm.to_latlon(bl.x, bl.y, utm_zone_num, utm_zone_letter))
    tr_latlon = Point(utm.to_latlon(tr.x, tr.y, utm_zone_num, utm_zone_letter))

    geo_rectangle = GeoRectangle()
    geo_rectangle.set_lower_left(bl_latlon)
    geo_rectangle.set_upper_right(tr_latlon)
    links_in_enlarged_area = wrapper.get_links(geo_rectangle)
    link_ids = [link.get_ID() for link in links_in_enlarged_area]

    return link_ids

def plot_links(inputs, vehicle_data, wrapper: OSMWrapper, ax):
    
    links: list[Link] = []
    for link_id in inputs:
        if link_id:
            sd_object = wrapper.get_sd_object_by_id(link_id)
            if sd_object:  # Check if the list is not empty
                links.append(sd_object[0])

    for link in links:
        vectorized_points = transform_to_vehicle_coordinates(vehicle_data, link)

        line = LineString(vectorized_points)
        color, lw, zorder, alpha = ('lightgrey', 1, 99, 1)
        ax.plot(*line.coords.xy, c=color, lw=lw, zorder=zorder, alpha=alpha)

    return

if __name__ == "__main__":
    
    fig, ax = plt.subplots()
    
    wrapper = OSMWrapper()
    
    from_file = True
    if from_file:
        input_file_path = os.path.join("test", os.listdir("test")[0])
        input_data = pickle.load(open(input_file_path, "rb"))
        route = np.array([a for a in input_data["route_coords"]])
        gt = np.array([input_data["gt"]["local_lat"], input_data["gt"]["local_lon"]]).T
        
        vehicle_data = {
            "ego_vehicle_lat": input_data["pred_time"]["lat"],
            "ego_vehicle_lon": input_data["pred_time"]["lon"],
            "ego_vehicle_yaw": input_data["pred_time"]["heading"],
        }
        
    else:
        # compare arbitrary place with Google Maps
        vehicle_data = {
            "ego_vehicle_lat": 51.17937295328097,
            "ego_vehicle_lon": 17.045812123893963,
            "ego_vehicle_yaw": 0.0,
        }
        
        route = None
        gt = None
        
        
    links = get_links_around_egovehicle(vehicle_data, wrapper)
        
    plot_links(links, vehicle_data, wrapper, ax)
    if route is not None:
        ax.plot(route[:, 0], route[:, 1], label='Route', color='red')
    if gt is not None:
        ax.plot(gt[:, 0], gt[:, 1], label='GT', color='green')
        
    ax.set_xlabel('Y')
    ax.set_ylabel('X')
    ax.set_xlim(-150, 150)
    ax.set_ylim(-20, 175)
    ax.set_aspect('equal')
    ax.legend()
    
    plt.show()