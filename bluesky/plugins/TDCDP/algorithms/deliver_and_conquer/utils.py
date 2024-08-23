import random
import re
import osmnx as ox
import networkx as nx
import numpy as np
import geopandas as gpd
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from shapely.geometry import Polygon, Point
from warnings import warn
from osmnx.convert import graph_to_gdfs

def str_interpret(value):
    return value  # Ensure the value remains a string

def find_index_with_tolerance(latlon, lat_list, lon_list, tol=1e-6):
    """Find the index of a latitude/longitude pair in lists with tolerance."""
    for idx, (lat, lon) in enumerate(zip(lat_list, lon_list)):
        if abs(latlon[0] - lat) <= tol and abs(latlon[1] - lon) <= tol:
            return idx
    raise ValueError(f"Coordinates {latlon} not found in route within tolerance.")

def sample_points(G: nx.MultiGraph, n: int) -> gpd.GeoSeries:
    """
    ADAPTED FROM https://github.com/gboeing/osmnx/blob/main/osmnx/utils_geo.py
    This function returns points that are on the geometry instead of an
    interpolation between these points.

    Randomly sample points constrained to a spatial graph.

    This generates a graph-constrained uniform random sample of points. Unlike
    typical spatially uniform random sampling, this method accounts for the
    graph's geometry. And unlike equal-length edge segmenting, this method
    guarantees uniform randomness.

    Parameters
    ----------
    G
        Graph from which to sample points. Should be undirected (to avoid
        oversampling bidirectional edges) and projected (for accurate point
        interpolation).
    n
        How many points to sample.

    Returns
    -------
    point
        The sampled points, multi-indexed by `(u, v, key)` of the edge from
        which each point was sampled.
    """
    if nx.is_directed(G):  # pragma: no cover
        msg = "`G` should be undirected to avoid oversampling bidirectional edges."
        warn(msg, category=UserWarning, stacklevel=2)
    gdf_edges = graph_to_gdfs(G, nodes=False)[["geometry", "length"]]
    weights = gdf_edges["length"] / gdf_edges["length"].sum()

    # Initialize storage
    sample = []
    selected_points = set()
    N = n
    i = 0
    max_i = 3
    while N != 0 and not i > max_i:
        idx = np.random.default_rng().choice(gdf_edges.index, size=n, p=weights)
        lines = gdf_edges.loc[idx, "geometry"]
        sample, N = select_points_from_lines(lines, sample, selected_points, N)
        i += 1
        if i > max_i:
            print(f"There was an issue with selecting {n} points."
                   f"A selection of {n-N} points was constructed.")

    return pd.Series(sample)

def select_points_from_lines(geo_series, stored_points, selected_points, N):
    """
    Select points from a GeoSeries of linestrings while avoiding duplicates and considering weights.

    Parameters
    ----------
    geo_series : gpd.GeoSeries
        GeoSeries containing linestrings.
    num_samples : int
        Number of points to sample.

    Returns
    -------
    pd.DataFrame
        DataFrame with selected points and their associated lines.
    """
    for line in geo_series:
        points = list(line.coords)  # Extract points from the LineString
        
        available_points = [p for p in points if p not in selected_points]
        
        if available_points:
            chosen_point = random.choice(available_points)
            selected_points.add(chosen_point)
            stored_points.append(Point(chosen_point))
            N -= 1

            if N == 0:
                return stored_points, N

    return stored_points, N

def find_wp_indices(rte, lr_locs, max_wp, decimal_places=5):
    """
    Find waypoint indices of waypoints included in the truck's route.

    Parameters:
    - rte: object, route object containing waypoint data
    - lr_locs: list of Point, selected points on the route
    - max_wp: int, maximum number of waypoints to consider
    - decimal_places: int, number of decimal places to round coordinates for comparison

    Returns:
    - np.array, array of unique waypoint indices
    """
    # Create a dictionary that can store multiple indices for the same rounded coordinates
    rte_points_dict = {}
    
    # Populate the dictionary with lists of indices for each coordinate
    for i, (lat, lon) in enumerate(zip(rte.wplat[:max_wp + 1], 
                                                    rte.wplon[:max_wp + 1])):
        key = (round(lat, decimal_places), round(lon, decimal_places))
        if key in rte_points_dict:
            rte_points_dict[key].append(i)
        else:
            rte_points_dict[key] = [i]

    # Extract tuples from lr_locs, rounding coordinates
    lr_locs_tuples = [
        (round(point.y, decimal_places), round(point.x, decimal_places))
        for point in lr_locs
    ]

    # Find matching indices, gathering all potential indices for each matching coordinate
    wp_indices = []
    for point in lr_locs_tuples:
        indices = rte_points_dict.get(point, [])
        wp_indices.extend(indices)
    
    return np.unique(np.array(wp_indices))

def extract_arguments(data_string, ext):
    # Remove the prefix
    if data_string.startswith(ext):
        data_string = data_string[len(ext):]
    
    # Split by commas
    arguments = data_string.split(',')
    
    return arguments

def get_drone_id(drone):
    match = re.search(r'UAV(\d+)_', drone)
    if match:
        number = match.group(1)
    return number

def get_available_drone(truck_drones):
    drone_ids = [get_drone_id(drone_name) for drone_name in truck_drones]
    for i in range(1, len(truck_drones) + 2):
        if str(i) not in drone_ids:
            return str(i)
    
    raise ValueError("No available drone found")

def calculate_area(graph):
    # Extract node coordinates
    nodes = graph.nodes(data=True)
    lats = [data['y'] for _, data in nodes]
    lons = [data['x'] for _, data in nodes]

    # Determine the bounding box
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)

    # Create a polygon from the bounding box
    bbox_polygon = Polygon([
        (min_lon, min_lat),
        (min_lon, max_lat),
        (max_lon, max_lat),
        (max_lon, min_lat),
        (min_lon, min_lat)
    ])

    # Convert the polygon to a GeoDataFrame
    gdf = gpd.GeoDataFrame(index=[0], crs="EPSG:4326", geometry=[bbox_polygon])

    # Convert to a planar coordinate system (e.g., UTM)
    gdf_utm = gdf.to_crs(gdf.estimate_utm_crs())

    # Calculate the area in square meters
    area_sqm = gdf_utm['geometry'].area.iloc[0]

    return area_sqm

def plot_route(G, lats, lons, title=None, labels=None, point_lat=None, 
                                            point_lon=None, point_label=None):
    fig, ax = ox.plot_graph(G, show=False, close=False)
    # Check the validity of lats and lons
    if not len(lats) == len(lons) or len(lats) > len(mcolors.BASE_COLORS):
        return
    
    # Plot the routes
    colors = list(mcolors.BASE_COLORS.values()) 
    for i in np.arange(len(lats)):
        try:
            ax.plot(lons[i], lats[i], linewidth=2, color=colors[i], label=labels[i])
        except:
            ax.plot(lons[i], lats[i], linewidth=2, color=colors[i])
    
    # Plot the single point if provided
    if point_lat is not None and point_lon is not None:
        ax.plot(point_lon, point_lat, 'ro', markersize=8, label=point_label or 'Point')
    
    plt.legend()
    plt.title(title)
    plt.show()

class Customer:
    """
    Class to represent a customer instance.
    """
    def __init__(self, id, location, wt, drone_eligible=True):
        """Initialize a customer instance
        args: type, description:
        - id: int, identifyer of the customer
        - location: list of 2 floats, lat lon location of the customer
        - wt: float, weight of the customer's package in lbs
        """
        self.id = id
        self.location = location
        self.wt = wt
        self.drone_eligible = drone_eligible
        self.delivery_time = None

    def __repr__(self):
        """Get the data within the customer instance"""
        return (f"Customer(id={self.id}, location={self.location}, "
                f"drone_eligible={self.drone_eligible}, delivery_time={self.delivery_time}), "
                f"weight={self.wt}")
    
    def to_dict(self):
        """Convert the customer data to a dictionary"""
        return {
            'id': self.id,
            'location': self.location.tolist(),
            'served': self.served,
            'weight': self.wt,
            'drone_eligible': self.drone_eligible,
            'delivery_time': self.delivery_time
        }