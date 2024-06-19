import numpy as np
import math
import osmnx as ox
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors
from k_means_constrained import KMeansConstrained
from bluesky.tools.geo import qdrdist
from shapely.geometry import LineString

def simplify_graph(G, tol=0.0001, gpkg_file=False):
    """
    Simplify the geometries of the edges in the GeoDataFrame.
    
    Args:
    - G: GeoDataFrame containing the edges to be simplified
    - tol: float, the simplification tolerance
    
    Returns:
    - GeoDataFrame with simplified geometries (edges)
    """
    # Convert the graph to GeoDataFrames
    gdf_nodes, gdf_edges = ox.graph_to_gdfs(G)
    

    # Identify and handle list-type fields
    list_type_columns = [col for col in gdf_edges.columns if gdf_edges[col].apply(lambda x: isinstance(x, list)).any()]
    # Convert list-type columns to strings
    for col in list_type_columns:
        gdf_edges[col] = gdf_edges[col].apply(lambda x: ','.join(map(str, x)) if isinstance(x, list) else x)

    gdf_edges_simplified = gdf_edges.copy()
    simplified_geometries = gdf_edges_simplified['geometry'].apply(lambda geom: geom.simplify(tol, \
                                                                                                preserve_topology=True))
    gdf_edges_simplified['geometry'] = simplified_geometries
    
    if gpkg_file:
        # Save the edges and nodes to a gpkg file for closer inspection on the simplifcation.
        # This has already been performed for tol=0.0001 (default), which gives accurate results but also faster 
        # processing of scenarios and simulation
        gdf_edges.to_file("graph_comparison.gpkg", layer='original_edges', driver="GPKG")
        gdf_edges_simplified.to_file("graph_comparison.gpkg", layer=f'edges {tol}', driver="GPKG")
        gdf_nodes.to_file("graph_comparison.gpkg", layer='nodes', driver="GPKG")

    # Convert back to an OSMNX graph
    G_mod = ox.graph_from_gdfs(gdf_nodes, gdf_edges_simplified)

    return G_mod

def get_map_lims(customer_locs, margin, unit='km'):
    """Function to get map limits where all customers fit in.
    Args: type, description
    margin: float or int, margin for borders of the map
    unit: string, unit for provided margin"""
    
    # Conversion factors
    unit_conversion = {
        'km': 1,
        'm': 1 / 1000,             # 1000 meters in a kilometer
        'mi': 1.60934,             # 1 mile is approximately 1.60934 kilometers
        'nm': 1.852                # 1 nautical mile is approximately 1.852 kilometers
    }

    # Convert margin to kilometers
    if unit in unit_conversion:
        margin_km = margin * unit_conversion[unit]
    else:
        raise ValueError(f"Unsupported unit: {unit}. Use 'km', 'm', 'mi', or 'nm'.")

    # Extract latitudes and longitudes into separate lists
    latitudes = [loc[0] for loc in customer_locs]
    longitudes = [loc[1] for loc in customer_locs]

    # Find the maximum and minimum values
    latmax = max(latitudes)
    latmin = min(latitudes)
    lonmax = max(longitudes)
    lonmin = min(longitudes)

    # Convert margin from km to degrees
    lat_margin_deg = margin_km / 111.32  # 1 degree latitude is approximately 111.32 km
    avg_lat = (latmax + latmin) / 2
    lon_margin_deg = margin_km / (111.32 * math.cos(math.radians(avg_lat)))  # Adjust longitude margin by latitude

    # Calculate the new limits
    box_latmax = latmax + lat_margin_deg
    box_latmin = latmin - lat_margin_deg
    box_lonmax = lonmax + lon_margin_deg
    box_lonmin = lonmin - lon_margin_deg

    # Return the coordinates as a tuple
    return (box_latmax, box_latmin, box_lonmax, box_lonmin)

def gen_clusters(M, custlocs):
    CCK = KMeansConstrained(size_min=1,size_max=M+1,random_state=0)
    return CCK.fit_predict(custlocs), CCK.cluster_centers_

def get_nearest(items, cur_pos, type):
    nearest_dist = np.inf
    idx = np.inf
    for item in items:
        if type == 'cluster':
            p1, p2 = item.centroid[0], item.centroid[1]
        elif type == 'customer':
            p1, p2 = item.location[0], item.location[1]
        else:
            raise ValueError(f"Type {type} not accepted, select from 'customer' or 'cluster'")
        _, dist = qdrdist(p1, p2, cur_pos[0], cur_pos[1])
        if dist == 0:
            # iif items equal eachother, continue
            continue
        elif dist < nearest_dist and not item.served:
            nearest_dist = dist
            idx = item.id
    # Nothing left, return None
    if idx == np.inf:
        return None
    return idx

def divide_and_order(customers, cur_pos, next_cluster_centroid):
    cust_copy = customers[:]
    TO_custs = [customer for customer in cust_copy if not customer.drone_eligible]
    truck_custs = []
    dronecusts = []

    if len(cust_copy) == 1:
        # CASE NUMBER 1: 
        # only 1 customer, serve with truck
        truck_custs = cust_copy
        return truck_custs, dronecusts
    
    elif len(cust_copy) == 2:
        # CASE NUMBER 2:
        # 2 customers, serve the closest one first
        # Both customers with truck
        C1_dist = euclidean_distance(cur_pos, cust_copy[0].location)
        C2_dist = euclidean_distance(cur_pos, cust_copy[1].location)

        truck_custs = cust_copy
        if C1_dist > C2_dist:
            # Closer to customer number 2 than to customer number 1
            # Go to customer 2 first, then to 1
            # This means swapping their positions around
            truck_custs.reverse()
        
        return truck_custs, dronecusts
    
    elif len(cust_copy) >= 3:
        if len(TO_custs) >= 3:
            # CASE NUMBER 3.1:
            # 3 or more customers, 3 or more truck only (rare)
            # Go to nearest one first
            entry_cust = min(cust_copy, key=lambda c: euclidean_distance(cur_pos, c.location))
            truck_custs.append(entry_cust)
            cust_copy.remove(entry_cust)
            # Then go to the one with max distance to next cluster, such we can exit close to nearest next cluster
            exit_cust = min(cust_copy, key=lambda c: euclidean_distance(next_cluster_centroid, c.location))
            cust_copy.remove(exit_cust)

            cust_i = entry_cust
            # Now append the final customers of the cluster, going to closest one continuously
            while len(cust_copy) > 0:
                next_cust = min(cust_copy, key=lambda c: euclidean_distance(cust_i, c.location))
                # update cust_i
                cust_i = next_cust
                cust_copy.remove(cust_i)
                truck_custs.append(cust_i)

            # append the exit customer such that it is the last one
            truck_custs.append(exit_cust)
            return truck_custs, dronecusts
        
        elif len(TO_custs) == 2:
            # CASE NUMBER 3.2:
            # 3 or more customers, of which 2 are truck only
            # Determine which one is exit and entry, the remaining cust is a drone cust
            truck_custs = TO_custs
            for cust in TO_custs:
                cust_copy.remove(cust)
            dronecusts = cust_copy

            # distance difference between current position dist and next cluster centroid dist
            dist_diff_c1 = euclidean_distance(cur_pos, truck_custs[0].location) - \
                            euclidean_distance(next_cluster_centroid, truck_custs[0].location)
            dist_diff_c2 = euclidean_distance(cur_pos, truck_custs[1].location) - \
                            euclidean_distance(next_cluster_centroid, truck_custs[1].location)

            # If the distance difference of first one is larger, this indicates it is wise to swap them
            if dist_diff_c1 > dist_diff_c2:
                truck_custs.reverse()

            return truck_custs, dronecusts

        elif len(TO_custs) == 1:
            # CASE NUMBER 3.3:
            # 3 or more customers, of which 1 is a mandatory truck only
            truck_custs = TO_custs

            exit_cust  = min(cust_copy, key=lambda c: euclidean_distance(next_cluster_centroid, c.location))
            entry_cust = min(cust_copy, key=lambda c: euclidean_distance(cur_pos, c.location))

            if exit_cust == entry_cust == truck_custs[0]:
                cust_copy.remove(exit_cust)
                additional_truck_cust = min(cust_copy, key=lambda c: 
                        euclidean_distance(exit_cust.location, c.location))
                truck_custs.append(additional_truck_cust)

            elif truck_custs[0] != exit_cust and truck_custs[0] != entry_cust:
                # Truck only delivery is neither the optimal entry or exit point
                # Replace the one that deviates less when compared to the original selected points for the truck.
                added_entry_dist = euclidean_distance(cur_pos, TO_custs[0].location) - \
                                    euclidean_distance(cur_pos, entry_cust.location) 
                added_exit_dist = euclidean_distance(next_cluster_centroid, TO_custs[0].location) - \
                                    euclidean_distance(next_cluster_centroid, exit_cust.location)

                if added_entry_dist > added_exit_dist:
                    # TO customer is the exit customer
                    truck_custs.append(entry_cust)
                    truck_custs.reverse()
                else:
                    # TO customer is the entry customer
                    truck_custs.append(exit_cust)

            elif truck_custs[0] == entry_cust:
                truck_custs.append(exit_cust)

            elif truck_custs[0] == exit_cust:
                truck_custs.append(entry_cust)
                truck_custs.reverse()

            for cust in truck_custs:
                try:
                    cust_copy.remove(cust)
                except ValueError:
                    continue

            # Remaining customer is drone customer
            dronecusts.append(cust_copy[0])

        elif len(TO_custs) == 0:
            # CASE NUMBER 3.4:
            # 3 or more customers, of which no truck only customers
            # Assign entry and exit customers regularly
            entry_cust = min(cust_copy, key=lambda c: euclidean_distance(cur_pos, c.location))
            truck_custs.append(entry_cust)
            cust_copy.remove(entry_cust)
            # Then go to the one with max distance to next cluster, such we can exit close to nearest next cluster
            exit_cust = min(cust_copy, key=lambda c: euclidean_distance(next_cluster_centroid, c.location))
            truck_custs.append(exit_cust)
            cust_copy.remove(exit_cust)
            # Now append the final customers to the dronecusts
            dronecusts = cust_copy
            
            return truck_custs, dronecusts

    return truck_custs, dronecusts
    
            
def euclidean_distance(p1, p2):
    # return np.sqrt(np.sum((np.array(p1) - np.array(p2))**2))
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2)

def reverse_linestring(line):
    """Helper function, reverses a Shapely LineString

    arg: type, description
    line: Shapely Linestring, line to reverse"""
    reversed_coords = LineString(list(line.coords)[::-1])
    return reversed_coords

def find_customer_by_location(customers, cur_cust_pos):
    for customer in customers:
        if np.allclose(customer.location, cur_cust_pos):
            return customer
    return None  # Return None if no customer matches the location

def plot_custlocs(custlocs, G, cluster = None):
    # Extract latitudes and longitudes
    latitudes = [location[0] for location in custlocs]
    longitudes = [location[1] for location in custlocs]

    # Plot the osmnx graph
    fig, ax = ox.plot_graph(G, show=False, close=False)

    # Plot the customer locations on the same graph
    if cluster is not None:
        unique_clusters = np.unique(cluster)
        cmap = cm.get_cmap('viridis', len(unique_clusters))
        norm = colors.Normalize(vmin=0, vmax=len(unique_clusters) - 1)

        for cluster_id in unique_clusters:
            indices = np.where(cluster == cluster_id)[0]
            ax.scatter([longitudes[i] for i in indices], 
                        [latitudes[i] for i in indices], 
                       c=np.array([cmap(norm(cluster_id))]), 
                       marker='o', s=50, label=f'Cluster {cluster_id}')
    else:
        ax.scatter(longitudes, latitudes, c='red', marker='o', s=50, label='Customer Locations')
    ax.legend()
    plt.show()

