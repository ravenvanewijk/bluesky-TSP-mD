import bluesky as bs
import numpy as np
import osmnx as ox
import taxicab as tc
import pandas as pd 
import random
from shapely.ops import linemerge
from bluesky import stack
from bluesky.traffic import Route
from bluesky.core import Entity, timed_function
from bluesky.tools.geo import kwikqdrdist
from bluesky.plugins.TDCDP.algorithms.utils import get_map_lims,\
                                                simplify_graph,\
                                                plot_custlocs,\
                                                gen_clusters,\
                                                get_nearest,\
                                                divide_and_order,\
                                                reverse_linestring,\
                                                find_customer_by_location
from bluesky.plugins.TDCDP.algorithms.customer import Customer, Cluster
from geopy.distance import great_circle
from bluesky.plugins.TDCDP.algorithms.set_partitioning.set_partitioning import\
                                                            SP_GA

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'REACT',
        'plugin_type':     'sim'
    }
    bs.traf.reactiveroute = ReactiveRoute()
    return config

class ReactiveRoute(Entity):
    def __init__(self):
        self.called = False
        self.timing_added = False
        # Keeps track of the time that has been added to the RTA to account for operations
        self.op_time_added = 0
    
    @stack.command
    def react(self, vehicle_group, M, *args):
        """Call the reactive algorithm to solve the routing to a set of customers with a given number of drones.
        This will employ a clustering method to serve each cluster iteratively.
        
        args: type, description
            - vehicle_group: str (int), type of drone(s) to employ. Details are described in Murray's paper and Github
            https://doi.org/10.1016/j.trc.2019.11.003. Refers to specific drone type 10<X>.
            - M: str (int), number of drones
            - *args: str (floats), series of floats that describe the customer locations lat/lon and package weight [lbs]
            should be a multiple of 3
            """

        self.load_custs(args)
        self.load_graph()
        self.gen_clusters(M)
        self.spawn_truck('TRUCK')
        plot_custlocs(self.custlocs, self.G, self.cluster_ids)

        #_____________PARAMS_____________
        self.delivery_time = 30
        self.sortie_time = 60
        self.rendezvous_time = 60
        self.vehicle_group = vehicle_group

        # set called to True such that the routing begins
        self.called = True

    @stack.command(name='livereact')
    def react2remaining(self):
        # implement function here that gets all existing locations of customers and react to these
        pass

    def spawn_truck(self, truckname):
        """Spawn the truck at the depot location.
        
        Args:
            - truckname: str, the name of the truck
        """
        self.truckname = truckname
        bs.traf.cre(self.truckname, 'Truck', self.depotloc[0], self.depotloc[1], 0, 0, 0)
        stack.stack(f'COLOUR {self.truckname} RED')
        stack.stack(f'PAN {self.truckname}')
        stack.stack('ZOOM 50')

    def load_custs(self, args):
        """Load customer data from arguments and create customer instances.
        
        Args:
            - args: list of floats, series of floats that describe the customer locations lat/lon and package weight [lbs]
        """
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
        # Remove the depot
        self.customers = self.customers[1:] 

    def gen_clusters(self, M):
        """Generate clusters for the customers using the set partitioning genetic algorithm.
        
        Args:
            - M: int, the number of drones plus two (truck entry and exit points)
        """
        # self.cluster_ids, cluster_centers = gen_clusters(3, self.custlocs)
        self.cluster_ids, cluster_centers, _ = SP_GA(self.custlocs, int(M)+2, 100)
        self.clusters = []
        for i in range(len(cluster_centers)):
            custids = np.where(np.array(self.cluster_ids)==i)[0]
            # adjustment is required, customer.id starts counting from 1 instead of 0
            clust_custs = [customer for customer in self.customers if customer.id - 1 in custids]
            self.clusters.append(Cluster(i, cluster_centers[i], clust_custs, False))

    def load_graph(self):
        """Load and simplify the road graph using OSMnx."""
        # 4 km border for the map is sufficient
        lims = get_map_lims(self.custlocs, 4)
        # Adjusted box sizes to include the entire map
        self.G = ox.graph_from_bbox(bbox=lims, network_type='drive')
        # Simplify the graph using osmnx
        self.G = simplify_graph(self.G)

    @stack.command(name='loadtiming')
    def load_truck_travel_times(self, input_dir, file_name):
        self.timing_added = True
        self.trucktiming = pd.read_csv(input_dir + '/' + file_name)
        self.C2Ctime = 0

    @timed_function(dt = bs.sim.simdt * 5)
    def determine_route(self):
        """Determine the route for the truck to the nearest cluster."""
        if not self.called:
            return
        acrte = Route._routes[self.truckname]
        first_clust = True if acrte.iactwp == -1 else False
        cond = True if len(acrte.wplat) <= acrte.iactwp+1 or acrte.iactwp == -1 else False
        if cond: 
            """Go to nearest cluster and route the truck and drones"""
            stack.stack('HOLD')
            cur_pos = (bs.traf.lat[0], bs.traf.lon[0]) if acrte.iactwp == -1 else (acrte.wplat[-1], acrte.wplon[-1])
            nearest_cluster = get_nearest(self.clusters, cur_pos, 'cluster')

            # route back to the start if all clusters are done
            if nearest_cluster is None:
                # When it is the last routepart, return
                # Routing is done so we can stop adding route parts
                road_back = self.roadroute((acrte.wplat[-1], acrte.wplon[-1]), self.depotloc)
                wp_commands = self.construct_scenario(road_back)
                stack.stack(wp_commands)
                destination_tolerance = 3/1852
                stack.stack(f"{self.truckname} ATDIST {road_back.xy[1][-1]} {road_back.xy[0][-1]} \
                                {destination_tolerance} TRKDEL {self.truckname}")
                # Add delivery point such that the truck stops at the end
                stack.stack(f"ADDOPERATIONPOINTS {self.truckname} {road_back.xy[1][-1]}/{road_back.xy[0][-1]} DELIVERY 1")
                timing_commands = self.truck_RTA((acrte.wplat[-1], acrte.wplon[-1]), 0, self.depotloc)
                stack.stack('OP')
                stack.stack('FF')
                self.called = False
                return

            next_nearest_cluster = get_nearest(self.clusters, self.clusters[nearest_cluster].centroid, 'cluster')

            # End of route reached, back to start is the way to go
            if next_nearest_cluster is None:
                next_target_loc = self.depotloc
            else:
                next_target_loc = self.clusters[next_nearest_cluster].centroid

            truck_custs, drone_custs = divide_and_order(self.clusters[nearest_cluster].customers, cur_pos, 
                                                        next_target_loc)
            # Mark cluster as served such that it is not selected again
            self.clusters[nearest_cluster].mark_served()
            # print(f'Now serving cluster {nearest_cluster}...')
            self.addroute(cur_pos, truck_custs, drone_custs)
            stack.stack('OP')
            stack.stack('FF')
            if first_clust:
                stack.stack(f"SPDAP {self.truckname} 5")
                stack.stack(f"{self.truckname} ATSPD 2 LNAV {self.truckname} ON")
                stack.stack(f"{self.truckname} ATSPD 2 VNAV {self.truckname} ON")
            else:
                pass

    def addroute(self, cur_pos, truck_custs, drone_custs):
        """Add route waypoints and operations for the truck and drones.
        
        Args:
            - cur_pos: tuple/list of floats, the current position of the truck (lat, lon)
            - truck_custs: list of Customer objects, the customers to be served by the truck
            - drone_custs: list of Customer objects, the customers to be served by the drones
        """
        for truckcust in truck_custs:
            road_route = self.roadroute(cur_pos, truckcust.location)
            wp_commands = self.construct_scenario(road_route)
            stack.stack(wp_commands)
            if self.timing_added:
                timing_commands = self.truck_RTA(cur_pos, truckcust.id, truckcust.location)
                stack.stack(timing_commands)
            # update current position for next customer part
            cur_pos = truckcust.location
        operation_commands = self.construct_operations(truck_custs, drone_custs)
        for operation in operation_commands:
            stack.stack(operation)

    def roadroute(self, A, B):
        """Compute the road route from point A to point B using the taxicab distance.
        
        Args:
            - A: tuple/ list of floats, the starting point (lat, lon)
            - B: tuple/ list of floats, the destination point (lat, lon)
        """
        route = []
        routepart = tc.distance.shortest_path(self.G, [A[0], A[1]], 
                                                    [B[0], B[1]])
    
        # Use the nodes to extract all edges u, v of graph G that the vehicle completely traverses
        routepart_edges = zip(routepart[1][:-1], routepart[1][1:])

        # routepart at beginning
        route.append(routepart[2])
        try:
            # For every pair of edges, append the route with the Shapely LineStrings
            for u, v in routepart_edges:
                # Some edges have this attribute embedded, when geometry is curved
                if 'geometry' in self.G.edges[(u, v, 0)]:
                    route.append(self.G.edges[(u, v, 0)]['geometry'])
                # Other edges don't have this attribute. These are straight lines between their two nodes.
                else:
                    # So, get a straight line between the nodes and append that line piece
                    route.append(LineString([(self.G.nodes[u]['x'], self.G.nodes[u]['y']), 
                                            (self.G.nodes[v]['x'], self.G.nodes[v]['y'])]))
        except IndexError:
            pass
        
        try:
            # Additional check for first linepart directionality. Sometimes it might be facing the wrong way.
            # The end of the beginning (incomplete) linestring should match
            try:
                if not route[1].coords[0] == routepart[2].coords[-1]:
                    # Check if flipped version does align
                    if route[1].coords[0] == routepart[2].coords[0]:
                        route[0] = reverse_linestring(route[0])
                    else:
                        raise Exception('Taxicab alignment Error: Coordinates of beginning LineString does not align')
            except IndexError:
                pass
        except AttributeError:
            pass

        try:
            # Check whether final incomplete linestring is in proper direction, similar check
            try:
                if not route[-1].coords[-1] == routepart[3].coords[0]:
                    # Check if flipped version does align
                    if route[-1].coords[-1] == routepart[3].coords[-1]:
                        route.append(reverse_linestring(routepart[3]))
                    else:
                        raise Exception('Taxicab alignment Error: Coordinates of final LineString does not align')
                else:
                    route.append(routepart[3])
            except IndexError:
                pass
        except AttributeError or IndexError:
            pass

        return linemerge(route)

    def construct_scenario(self, road_route):
        """Construct the scenario text for the waypoints of the road route.
        
        Args:
            - road_route: LineString, the road route as a LineString
        """
        route_waypoints = list(zip(road_route.xy[1], road_route.xy[0]))
        route_lats = road_route.xy[1]
        route_lons = road_route.xy[0]
        i = 1 # Start at second waypoint
        turns = ['turn'] # Doesn't matter what the first waypoint is designated as, so just have it as true.
        for lat_cur, lon_cur in route_waypoints[1:-1]:
            # Get the previous and the next waypoint
            lat_prev, lon_prev = route_waypoints[i-1]
            lat_next, lon_next = route_waypoints[i+1]
            # Get the angle
            a1, _ = kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
            a2, _ = kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
            angle=abs(a2-a1)
            if angle>180:
                angle=360-angle
            # In general, we noticed that we don't need to slow down if the turn is smaller than 25 degrees
            # If the angle is larger, then a more severe slowdown is required
            #  However, this will depend on the cruise speed of the vehicle.
            if angle > 35:
                turns.append('sharpturn')
            elif angle > 25:
                turns.append('turn')
            else:
                turns.append('straight')
            i += 1

        # Let the vehicle slow down for the depot
        turns.append(True)
        # Add some commands to pan to the correct location and zoom in, and use the modified active wp package.
        scen_text = ""

        # After creating it, we want to add all the waypoints. We can do that using the ADDTDWAYPOINTS command.
        # ADDTDWAYPOINTS can chain waypoint data in the following way:
        # ADDTDWAYPOINTS ACID LAT LON ALT SPD Turn? TurnSpeed
        # SPD here can be set as the cruise speed so the vehicle knows how fast to go
        # cruise_spd = 25 #kts
        cruise_alt = 0 # Keep it constant throughout the flight
        # Turn speed of 5 kts usually works well
        turn_spd = 10 #kts
        sharpturn_spd = 5 #kts
        # Initiate adddtwaypoints command
        scen_text += f'ADDTDWAYPOINTS {self.truckname}' # First add the root of the command
        # Loop through waypoints
        for wplat, wplon, turn in zip(route_lats, route_lons, turns):
            # Check if this waypoint is a turn
            if turn == 'turn' or turn == 'sharpturn':
                wptype = 'TURNSPD'
                wp_turnspd = turn_spd if turn == 'turn' else sharpturn_spd
            else:
                wptype = 'FLYBY'
                # Doesn't matter what we pick here, as long as it is assigned. 
                # Will be ignored
                wp_turnspd = turn_spd
            # Add the text for this waypoint. It doesn't matter if we always add a turn speed, as BlueSky will
            # ignore it if the wptype is set as FLYBY
            # we have to give a speed if we dont specify RTAs, so set the default to 25
            cruisespd = '' if self.timing_added else 25
            scen_text += f',{wplat},{wplon},{cruise_alt},{cruisespd},{wptype},{wp_turnspd}'

        return scen_text

    def construct_operations(self, truck_custs, drone_custs):
        """Construct the operation commands for the truck and drones.
        
        Args:
            - truck_custs: list of Customer objects, the customers to be served by the truck
            - drone_custs: list of Customer objects, the customers to be served by the drones
        """
        operation_text = []
        for truck_cust in truck_custs:
            operation_text.append(f"ADDOPERATIONPOINTS {self.truckname} {truck_cust.location[0]}/{truck_cust.location[1]} \
                                DELIVERY {self.delivery_time}")
        
        for i, drone_cust in enumerate(drone_custs):
            operation_text.append(f"ADDOPERATIONPOINTS {self.truckname}, \
                                {truck_custs[0].location[0]}/{truck_custs[0].location[1]}, \
                                SORTIE, {self.sortie_time}, M{self.vehicle_group}, {i + 1}, \
                                {drone_cust.location[0]}, {drone_cust.location[1]}, \
                                {truck_custs[-1].location[0]}/{truck_custs[-1].location[1]}, \
                                164.04199475065616, 60.828336856672 60, 30")

        return operation_text

    def truck_RTA(self, cur_cust_pos, truckcust_id, truckcust_location):
        if list(cur_cust_pos) == self.depotloc:
            cur_cust_id = 0
        else:
            cur_cust = find_customer_by_location(self.customers, cur_cust_pos)
            cur_cust_id = cur_cust.id

        # look up position of the time that is required
        # self.trucktiming is a flattened matrix, so take sqrt of len and multiply with cur cust id
        # Then you arrive at the cur_cust. Add truck cust id to get to correct pos
        pos = int(cur_cust_id * np.sqrt(len(self.trucktiming)) + truckcust_id)
        data = self.trucktiming.iloc[pos]
        t_req = data[' time [sec]']
        self.C2Ctime += t_req
        RTA = self.C2Ctime + self.op_time_added

        return f"TDRTA {self.truckname}, {truckcust_location[0]}/{truckcust_location[1]}, {RTA}"

    @timed_function
    def mod_truck_RTA(self):
        """Modifies the truck required time of arrival to account for the operations it has performed so far"""
        if self.called and self.timing_added:
            diff = bs.traf.Operations.trk_op_time - self.op_time_added
            if diff > 0:
                # add time lost to RTAs to account for the delay due to operations
                Route._routes[self.truckname].wprta = \
                                        [value + diff if value > 0 else value 
                                        for value in Route._routes['TRUCK'].wprta]
                # Update the added time such that it will not be added again
                self.op_time_added += diff
            # print([value for value in Route._routes[self.truckname].wprta if value > 0])
            # print(bs.sim.simt)
