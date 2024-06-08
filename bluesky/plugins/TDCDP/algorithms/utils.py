import numpy as np
import math
import osmnx as ox
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors
from k_means_constrained import KMeansConstrained
from bluesky.tools.geo import qdrdist

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
        _, dist = qdrdist(p1, p2, cur_pos[0], cur_pos[1])
        if dist == 0:
            # iif items equal eachother, continue
            continue
        elif dist < nearest_dist:
            nearest_dist = dist
            idx = item.id
    return idx

def divide_custs(customers, cur_pos, next_cluster_centroid, mandatory_truck_custs=2):
    cust_copy = customers[:]

    TO_custs = [customer for customer in cust_copy if not customer.drone_eligible]
    for cust in TO_custs:
        cust_copy.remove(cust)
    
    truckcusts = []
    dronecusts = []
    if len(TO_custs) > 0:
        truckcusts.extend(TO_custs)
    
    # Find the closest customer to the next cluster centroid for entry
    if len(cust_copy) > 0:
        entry_customer = min(cust_copy, key=lambda c: euclidean_distance(cur_pos, c.location))
        truckcusts.append(entry_customer)
        cust_copy.remove(entry_customer)
        
    # Find the closest customer to the next cluster centroid for exit
    if len(cust_copy) > 0:
        exit_customer = min(cust_copy, key=lambda c: euclidean_distance(next_cluster_centroid, c.location))
        truckcusts.append(exit_customer)
        cust_copy.remove(exit_customer)

    if len(truckcusts) == mandatory_truck_custs + 1:
        # select the one that deviates less when compared to the original selected points for the truck.
        added_entry_dist = euclidean_distance(cur_pos, TO_custs[0].location) - \
                            euclidean_distance(cur_pos, entry_customer.location) 
        added_exit_dist = euclidean_distance(next_cluster_centroid, TO_custs[0].location) - \
                            euclidean_distance(next_cluster_centroid, exit_customer.location)

        if added_entry_dist > added_exit_dist:
            # TO customer becomes the exit customer
            cust_copy.append(exit_customer)
            truckcusts.remove(exit_customer)
            # Required to reverse now, to get the right order of customers
            truckcusts.reverse()
        else:
            # TO customer becomes the entry customer
            cust_copy(entry_customer)
            truckcusts.remove(entry_customer)

    elif len(truckcusts) > mandatory_truck_custs + 1:
        for cust in truckcusts:
            if cust.drone_eligible:
                cust_copy.append(cust)
                truckcusts.remove(cust)

    dronecusts.extend(cust_copy)
    cust_copy.clear()


    return truckcusts, dronecusts 


def euclidean_distance(p1, p2):
    # return np.sqrt(np.sum((np.array(p1) - np.array(p2))**2))
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2)

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

