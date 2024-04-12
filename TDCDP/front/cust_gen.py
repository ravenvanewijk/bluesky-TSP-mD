import os 
import geopandas as gpd
import osmnx as ox
import matplotlib.pyplot as plt

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

    def plot_graph(self):
        """Plots the graph of the selected gpkg file"""
        ox.plot_graph(self.G)

    def plot_cust(self):
        """Plots the graph of the selected gpkg file as well as customer locations"""
        if self.locs is None:
            raise Exception("Customer locations should first be generated. \
                            Use 'gen_customers(n)' to generate n customer locations")
        else:
            # Plot city graph
            fig, ax = ox.plot_graph(self.G, show=False, close=False)
            # Plot the customers
            for _, point in self.locs.items():
                ax.scatter(point.x, point.y, c='red', s=100, zorder=5)  # Adjust color and size as needed

            # Show the plot
            plt.show()

# sm = Streetgraph('rotterdam.gpkg')
# sm.gen_customers(50)
# sm.plot_cust()