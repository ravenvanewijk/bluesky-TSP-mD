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

def get_nearest_cluster(clusters, curloc):
    nearest_dist = np.inf
    idx = np.inf
    for cluster in clusters:
        _, dist = qdrdist(cluster.centroid[0], cluster.centroid[1], curloc[0], curloc[1])
        if dist == 0:
            # iif clusters equal eachother, continue
            continue
        elif dist < nearest_dist:
            nearest_dist = dist
            idx = cluster.id
    return idx

def plot_custlocs(custlocs, G, cluster = None):
    # Extract latitudes and longitudes
    latitudes = custlocs[:, 0].astype(float)
    longitudes = custlocs[:, 1].astype(float)

    # Plot the osmnx graph
    fig, ax = ox.plot_graph(G, show=False, close=False)

    # Plot the customer locations on the same graph
    if cluster is not None:
        unique_clusters = np.unique(cluster)
        cmap = cm.get_cmap('viridis', len(unique_clusters))
        norm = colors.Normalize(vmin=0, vmax=len(unique_clusters) - 1)

        for cluster_id in unique_clusters:
            indices = np.where(cluster == cluster_id)
            ax.scatter(longitudes[indices], latitudes[indices], 
                       c=np.array([cmap(norm(cluster_id))]), 
                       marker='o', s=50, label=f'Cluster {cluster_id}')
    else:
        ax.scatter(longitudes, latitudes, c='red', marker='o', s=50, label='Customer Locations')
    ax.legend()
    plt.show()

