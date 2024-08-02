import random
import osmnx as ox
import numpy as np
import geopandas as gpd
from shapely.geometry import Polygon, Point

def str_interpret(value):
    return value  # Ensure the value remains a string

# Function to select a random vertex on the edge geometry
def select_random_vertex(line):
    # Extract the coordinates from the LineString
    x, y = line.xy
    vertices = np.column_stack([x, y])
    
    # Select a random vertex
    random_vertex = random.choice(vertices)
    
    # Return the random vertex as a Point
    random_vertex_point = Point(random_vertex)
    return random_vertex_point

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