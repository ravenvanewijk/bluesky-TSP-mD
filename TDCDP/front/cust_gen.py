import os 
import geopandas as gpd
import osmnx as ox
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString

class Streetgraph():

    def __init__(self, filename): 
        # save root path to ensure easily navigating back
        self.root_path = os.getcwd()
        try:
            # attempt to change folder to TDCDP working folder
            os.chdir(os.getcwd() + '/TDCDP')
            try:
                os.chdir(os.getcwd() + '/rotterdam')
            except:
                raise Exception('rotterdam folder not found')
        except:
            raise Exception('TDCDP folder not found')


        try:
            self.nodes = gpd.read_file(filename, layer='nodes')
            self.edges = gpd.read_file(filename, layer='edges')
        except:
            raise Exception(f'{filename} graph not found')

        self.locs = None

        # When loading a graph from a gpkg, geopandas will try to (incorrectly) guess the index. 
        # So we need to set the correct index.
        # Each node gets an "osmid", an OpenStreetMap ID. In this graph however, we set our own custom (simpler)
        # osmids in ascending order. However, this ID is needed for working with the OSMNX package.
        self.nodes.set_index(['osmid'], inplace=True)
        # Every edge is described by a starting node (u) and an end node (v). These also show the directionality
        # of the edge. You can travel from u to v but not from v to u. Key is always 0.
        self.edges.set_index(['u', 'v', 'key'], inplace=True)

        self.G = ox.graph_from_gdfs(self.nodes, self.edges)

        # get undirected graph
        self.G_undir = ox.get_undirected(self.G)
    
    def gen_customers(self, n):
        """
        Generates locations of customers on the graph.

        params:
        n = number of customers
        """
        # call function that randomly samples on roads. Can be anywhere inbetween
        self.locs = ox.utils_geo.sample_points(self.G, n)
        # Reset the index to a simple integer index
        self.locs.reset_index(drop=True, inplace=True)

    def gen_depots(self, n=1):
        """
        Generates locations of depots on the graph.

        params:
        n = number of depots
        """
        # call function that randomly samples on roads. Can be anywhere inbetween
        self.depots = ox.utils_geo.sample_points(self.G, n)

    def plot_graph(self, cust = False, depots = False):
        """Plots the graph of the selected gpkg file as well as customer locations"""
        # Plot city graph
        fig, ax = ox.plot_graph(self.G, show=False, close=False)
        # Plot the customers
        if cust and self.locs is not None:
            customer_scatter = ax.scatter([point.x for _, point in self.locs.items()],
                                            [point.y for _, point in self.locs.items()],
                                            c='red', s=50, zorder=5, label='Customers')
        if depots and self.depots is not None:
            plural = 's' if len(self.depots) > 1 else ''
            depot_scatter = ax.scatter([point.x for _, point in self.depots.items()],
                                        [point.y for _, point in self.depots.items()],
                                        c='blue', s=100, zorder=5, label='Depot' + plural)
        # Show the plot with a legend
        ax.legend(handles=[customer_scatter, depot_scatter])
        plt.show()

    def random_tsp(self):
        """Generate a semi random NN TSP route"""
        if self.depots is None:
            raise Exception("Impossible to generate solution without depot. Call gen_depots(n) first.")
        
        if len(self.depots) != 1:
            raise Exception("Method only callable for single depot graphs")

        if self.locs is None:
            raise Exception("Impossible to generate solution without customers. Call gen_customers(n) first.")

        lat_d, lon_d = self.depots.iloc[0].y, self.depots.iloc[0].x

        self.order = []
        visited = set()  # To keep track of visited customers
        current_index = -1  # Start with the depot
        TSP_len = len(self.locs)

        while len(self.order) < TSP_len:
            NN = {'dist': np.inf, 'index': -1}  # Initialize nearest neighbor with infinity distance
            for idx, customer in self.locs.items():
                if idx not in visited:
                    dist = ox.distance.euclidean_dist_vec(lat_d, lon_d, customer.y, customer.x)
                    if dist < NN['dist']:
                        NN['dist'] = dist
                        NN['index'] = idx

            # Update current position to the nearest neighbor found
            if NN['index'] == -1:
                raise Exception("No valid next customer found; possibly an error in distance calculation or logic.")
            
            visited.add(NN['index'])  # Mark this customer as visited
            self.order.append(NN['index'])  # Add to the tour order
            # Update current coordinates to the nearest neighbor's coordinates
            lat_d, lon_d = self.locs.loc[NN['index']].y, self.locs.loc[NN['index']].x

    def plot_route(self):
        """WIP: plot route of the semi random TSP"""
        # Check if a route order exists
        if not hasattr(self, 'order') or self.order is None:
            raise Exception("No route order found. Please generate the route first.")

        # Extract node coordinates in the order of the route
        route_geom = []
        for idx in self.order:
            node = self.locs.loc[idx]
            route_geom.append((node.x, node.y))

        # Create a LineString from the coordinates
        route_line = LineString(route_geom)

        # Create a GeoDataFrame to store the line
        route_gdf = gpd.GeoDataFrame(index=[0], crs=self.locs.crs, geometry=[route_line])

        # Plotting the graph
        fig, ax = ox.plot_graph(self.G, show=False, close=False)
        self.locs.plot(ax=ax, marker='o', color='red', zorder=3)  # plot customer locations
        self.depots.plot(ax=ax, marker='x', color='blue', zorder=3)  # plot depot location
        route_gdf.plot(ax=ax, linewidth=2, color='green', zorder=2)  # plot the route

        # Add a legend
        plt.scatter([], [], c='red', label='Customers')
        plt.scatter([], [], c='blue', label='Depot')
        plt.plot([], [], color='green', label='Route')
        plt.legend()

        # Show the plot
        plt.show()

# sm = Streetgraph('rotterdam.gpkg')
# sm.gen_customers(50)
# sm.gen_depots()
# sm.plot_graph(cust=True, depots=True)
# sm.random_tsp()
# sm.plot_route()