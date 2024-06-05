import bluesky as bs
import numpy as np
import osmnx as ox
import taxicab as tc
from bluesky import stack
from bluesky.core import Entity, timed_function
from bluesky.plugins.TDCDP.algorithms.utils import get_map_lims,\
                                                simplify_graph,\
                                                plot_custlocs,\
                                                gen_clusters,\
                                                get_nearest_cluster
from bluesky.plugins.TDCDP.algorithms.customer import Customer, Cluster
from geopy.distance import great_circle
# from bluesky.tools.geo import qdrdist

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'REACT',
        'plugin_type':     'sim'
    }
    return config

@stack.command
def react(vehicle_group, M, *args):
    """Call the reactive algorithm to solve the routing to a set of customers with a given number of drones.
    This will employ a clustering method to serve each cluster iteratively.
    
    args: type, description
        - vehicle_group: str (int), type of drone(s) to employ. Details are described in Murray's paper and Github
        https://doi.org/10.1016/j.trc.2019.11.003. Refers to specific drone type 10<X>.
        - M: str (int), number of drones
        - *args: str (floats), series of floats that describe the customer locations lat/lon and package weight [lbs]
        should be a multiple of 3
        """
    # call the reactiveroute class to react to the set of customers
    bs.traf.reactiveroute = ReactiveRoute(vehicle_group, M, args)

@stack.command(name='livereact')
def react2remaining():
    # implement function here that gets all existing locations of customers and react to these
    pass

class ReactiveRoute(Entity):
    def __init__(self, vehicle_group, M, args):
        """Initialize the reactive route on a set of customers that will be solved by the reactive algorithm."""
        if len(args)%3 !=0:
            bs.scr.echo('You missed a customer value, arguement number must be a multiple of 3.')
            return

        args = np.reshape(args, (int(len(args)/3), 3)).astype(float)
        self.custlocs = []
        parcelwt = []

        for custdata in args:
            self.custlocs.append([custdata[0], custdata[1]])
            parcelwt.append(custdata[2])

        # Store the depot location
        self.depotloc = self.custlocs[0]
        # Remove the depot
        self.custlocs = self.custlocs[1:]
        # Create customer instances
        self.customers = [Customer(id=i, location=data[:-1], wt=data[-1], 
                        drone_eligible=False if data[-1]==100 else True) 
                        for i, data in enumerate(args)]
        # 4 km border for the map is sufficient
        lims = get_map_lims(self.custlocs, 4)
        # Adjusted box sizes to include the entire map
        self.G = ox.graph_from_bbox(bbox=lims, network_type='drive')
        # Simplify the graph using osmnx
        self.G = simplify_graph(self.G)
        self.gen_distance_matrix()
        cluster_ids, cluster_centers = gen_clusters(3, self.custlocs)
        self.clusters = []
        for i in range(len(cluster_centers)):
            custids = np.where(cluster_ids==0)[0]
            clust_custs = [customer for customer in self.customers if customer.id in custids]
            self.clusters.append(Cluster(i, cluster_centers[i], clust_custs))
        # hdg, _ = qdrdist(float(data['lat_i']), float(data['lon_i']), float(data['lat_j']), float(data['lon_j']))
        bs.traf.cre('TRUCK', 'Truck', self.depotloc[0], self.depotloc[1], 0, 0, 0)

    @timed_function(dt = bs.sim.simdt)
    def determine_route(self):
        """Go to nearest cluster and route the truck and drones"""
        nearest_cluster = get_nearest_cluster(self.clusters, (bs.traf.lat[0], bs.traf.lon[0]))
        next_nearest_cluster = get_nearest_cluster(self.clusters, self.clusters[7].centroid)

    def gen_distance_matrix(self):
        """Get distance matrix from every i to every j"""
        num_locations = len(self.custlocs)
        self.distance_matrix = np.zeros((num_locations, num_locations))
        
        for i in range(num_locations):
            for j in range(num_locations):
                if i == j:
                    continue
                lat1, lon1 = self.custlocs[i]
                lat2, lon2 = self.custlocs[j]
                self.distance_matrix[i, j] = great_circle((lat1, lon1), 
                                                        (lat2, lon2)).meters
            
