import bluesky as bs
import osmnx as ox
import numpy as np
import pandas as pd
import geopandas as gdp
import random
import matplotlib.pyplot as plt
from roadroute_lib import roadroute
from shapely.ops import linemerge
from shapely import Point
from bluesky.traffic.route import Route
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.utils import (
    str_interpret,
    calculate_area,
    Customer,
    find_index_with_tolerance,
    sample_points,
    find_wp_indices,
    extract_arguments
)
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.opt_inputs import (
    calc_inputs,
    raw_data
)
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.PACO import PACO
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.op_opt import (
    LR_Optimizer,
    LR_PuLP
)
from bluesky.plugins.TDCDP.TDoperations import delivery_dist
from bluesky.plugins.TDCDP.TDdronemanager import get_wpname
from bluesky.tools.aero import kts, ft                                                                                                   
from bluesky.tools.geo import kwikqdrdist
from bluesky import stack                                                       
from bluesky.core import Entity, timed_function

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'DC',
        'plugin_type':     'sim'
    }
    bs.traf.deliver_conquer = DeliverConquer()
    return config

class DeliverConquer(Entity):
    def __init__(self):
        self.called = False

    def reset_added_cmds(self):
        self.added_route = []
        self.added_deliveries = []
    
    @stack.command
    def deliver(self, vehicle_group, M, problem_name, *args):
        """Call the deliver and conquer algorithm to solve the routing to a \
        set of customers with a given number of drones.
        This will employ a clustering method to serve each cluster iteratively.
        
        args: type, description
            - vehicle_group: str (int), type of drone(s) to employ. 
            Details are described in Murray's paper and Github
            https://doi.org/10.1016/j.trc.2019.11.003
            Refers to specific drone type 10<X>.
            - M: str (int), number of drones
            - problem_name: str, name of the problem used to look up timing 
            table of the truck
            - *args: str (floats), series of floats that describe the customer 
            locations lat/lon and package weight [lbs]
            should be a multiple of 3
            """
        d_data = raw_data['M'+vehicle_group]['envelop']
        self.load_custs(args)
        self.spawn_truck('TRUCK')

        #_____________PARAMS_____________
        self.M = M
        self.delivery_time = 60
        self.sortie_time = 60
        self.rendezvous_time = 30
        self.truck_delivery_time = 30
        self.vehicle_group = vehicle_group
        self.cruise_alt = d_data['h_max']
        self.cruise_spd = d_data['v_max']
        self.vspd_up = d_data['vs_max']
        self.vspd_down = np.abs(d_data['vs_min'])
        #________________________________

        # Dictionaries to store the text, will be used later
        self.routing_cmds = {}
        self.delivery_cmds = {}
        self.calc_basic_tsp(problem_name)
        self.route_basic_tsp()
        self.gen_lr_locs()

        # Create a copy of the tour
        self.current_route = self.model.P[0].tour[:]
        self.added_route = []
        self.added_deliveries = []
        # set called to True such that the drone routing begins
        self.called = True

    def spawn_truck(self, truckname):
        """Spawn the truck at the depot location.
        
        Args:
            - truckname: str, the name of the truck
        """
        self.truckname = truckname
        bs.traf.cre(self.truckname, 'Truck', self.customers[0].location[0], \
                    self.customers[0].location[1], 0, 0, 0)
        stack.stack(f'COLOUR {self.truckname} RED')
        stack.stack(f'PAN {self.truckname}')
        stack.stack('ZOOM 50')

    def load_custs(self, args):
        """Load customer data from arguments and create customer instances.
        
        Args:
            - args: list of floats, series of floats that describe the customer
                    locations lat/lon and package weight [lbs]
        """
        if len(args)%3 !=0:
            bs.scr.echo('You missed a customer value, argument number must be a multiple of 3.')
            return

        args = np.reshape(args, (int(len(args)/3), 3)).astype(float)
        self.custlocs = []
        parcelwt = []

        for custdata in args:
            self.custlocs.append([custdata[0], custdata[1]])
            parcelwt.append(custdata[2])

        # Create customer instances
        self.customers = [Customer(id=i, location=data[:-1], wt=data[-1], 
                        drone_eligible=False if data[-1]==100 else True) 
                        for i, data in enumerate(args)]

    @stack.command(name='LOADGRAPH')
    def load_graph(self, path):
        """Load and simplify the road graph using OSMnx."""
        self.G = ox.load_graphml(filepath=path,
                                edge_dtypes={'osmid': str_interpret,
                                            'reversed': str_interpret})
        
    def gen_lr_locs(self, cust_only = False):
        # Extract locations
        locations = [Point(customer.location[1], customer.location[0]) 
                        for customer in self.customers]

        # Create a pandas Series from the locations
        self.lr_locs = pd.Series(locations)

        if not cust_only:
            sites_per_km2 = 10
            sites_per_m = sites_per_km2 / 10**6
            n = int(calculate_area(self.G) * sites_per_m)
            n -= len(self.customers)
            self.lr_locs = pd.concat(
                            [sample_points(self.G, n), self.lr_locs],
                            ignore_index=True
                                        )
        
        # self.plot_graph(True)

    def calc_basic_tsp(self, problem_name):
        """Calculate TSP path without any drones, only 1 truck
        Uses Ant Colony Optimization to determine its route"""

        folder = 'bluesky/plugins/TDCDP/algorithms/deliver_and_conquer/timing_tables'
        ST = pd.read_csv(f'{folder}/tbl_truck_travel_data_{problem_name}.csv')
        ST['1/time'] = ST['time [sec]'].apply(lambda x: 1/x if x != 0 else np.nan)

        cities = int(np.sqrt(ST.shape[0]))
        eta = np.full((cities, cities), np.nan)
        dist_mat = np.full((cities, cities), np.nan)
        # Populate the matrix
        for _, row in ST.iterrows():
            i = int(row['% from location i'])
            j = int(row['to location j'])
            
            # Ensure indices are within bounds
            if 0 <= i < cities and 0 <= j < cities:
                eta[i, j] = row['1/time']
                dist_mat[i, j] = row['time [sec]']
            else:
                raise Exception(f"Warning: Index out of bounds (i={i}, j={j})")

        self.model = PACO(1, 10, 0.3, 3, 10, 200, eta, 5)
        self.model.simulate()

    def route_basic_tsp(self):
        """
        ________________________________PHASE 1________________________________
        Routes the truck according to the TSP obtained by PACO."""
        stack.stack("HOLD")
        delivery_cmds = []
        for u, v in zip(self.model.P[0].tour, self.model.P[0].tour[1:]):
            road_route, spd_lims = roadroute(self.G, 
                                            self.customers[u].location,
                                            self.customers[v].location)
            road_route_merged = linemerge(road_route)
            # Update customer location with Bluesky location on route
            self.customers[u].location = \
                np.around([road_route_merged.xy[1][0], 
                            road_route_merged.xy[0][0]], 6)
            wp_commands = self.construct_scenario(road_route_merged, spd_lims)
            self.routing_cmds[f'{u}-{v}'] = wp_commands
            stack.stack(wp_commands)
            if v == 0:
                continue
            op_text = (f"ADDOPERATIONPOINTS {self.truckname},"
                f"{self.customers[v].location[0]}/"
                f"{self.customers[v].location[1]},"
                f"DELIVERY,{self.delivery_time}")
            delivery_cmds.append(op_text)
            self.delivery_cmds[f'{v}'] = op_text

        for delivery_cmd in delivery_cmds:
            stack.stack(delivery_cmd)

        # stack.stack(f"VNAV {self.truckname} ON")
        # stack.stack(f"LNAV {self.truckname} ON")
        stack.stack("TRAIL ON")

        stack.stack("OP")

    def construct_scenario(self, road_route, spd_lims):
        """Construct the scenario text for the waypoints of the road route.
        
        Args:
            - road_route: LineString, the road route as a LineString
            - spd_lims: list, speed limits of the road route's LineStrings
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
            # In general, we noticed that we don't need to slow down if 
            # the turn is smaller than 25 degrees
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
        scen_text = ""

        # We can do that using the ADDTDWAYPOINTS command.
        # ADDTDWAYPOINTS can chain waypoint data in the following way:
        # ADDTDWAYPOINTS ACID LAT LON ALT SPD Turn? TurnSpeed
        # cruise_spd = 25 #kts
        cruise_alt = 0 # Keep it constant throughout the flight
        # Turn speed of 5 kts usually works well
        turn_spd = 10 #kts
        sharpturn_spd = 5 #kts
        # Initiate addtdwaypoints command
        scen_text += f'ADDTDWAYPOINTS {self.truckname}'
        # Loop through waypoints
        for wplat, wplon, turn, spdlim in zip(route_lats, route_lons, turns, spd_lims):
            # Check if this waypoint is a turn
            if turn == 'turn' or turn == 'sharpturn':
                wptype = 'TURNSPD'
                wp_turnspd = turn_spd if turn == 'turn' else sharpturn_spd
            else:
                wptype = 'FLYBY'
                # Doesn't matter what we pick here, as long as it is assigned. 
                # Will be ignored
                wp_turnspd = turn_spd
            # Add the text for this waypoint. 
            # It doesn't matter if we always add a turn speed, as BlueSky will
            # ignore it if the wptype is set as FLYBY
            cruisespd = spdlim
            scen_text += f',{wplat},{wplon},{cruise_alt},{cruisespd},{wptype},{wp_turnspd}'

        return scen_text

    def plot_graph(self,lines=[], cust=False):
        """Plots the graph of the selected gpkg file as well as customer 
        locations"""
        # Plot city graph
        fig, ax = ox.plot_graph(self.G, show=False, close=False)
        # Plot the customers
        if cust and self.lr_locs is not None:
            locs_scatter = ax.scatter([point.x for _, point in self.lr_locs.items()],
                                            [point.y for _, point in self.lr_locs.items()],
                                            c='red', s=8, zorder=5, label='L&R locations')

        for line in lines:
            x, y = line.xy
            ax.plot(x, y, marker='o')  # Plot the line with markers at vertices
            ax.plot(x[-1],y[-1],'rs') 

        # Show the plot with a legend
        ax.legend(handles=[locs_scatter])
        plt.show()

    def remove_commands(self, dcustid, ncustid):
        """Removed commands from the dictionaries where commands are stored.
        This must be done when customers are served, such that these commands
        will not be parsed again.
        """
        # Alter route and remove commands from dictionaries
        try:
            # Only necessary first time
            self.routing_cmds.pop(f'0-{dcustid}')
            self.current_route.remove(0)
        except KeyError:
            pass 
        self.routing_cmds.pop(f'{dcustid}-{ncustid}')
        self.delivery_cmds.pop(f'{dcustid}')
        # Alter current route of the truck
        while not self.current_route.index(dcustid) == 0:
            # Account for the case where truck has made one or more deliveries
            # During the delivery of the drone(s)
            to_del = self.current_route[0]
            self.current_route.remove(to_del)
            self.delivery_cmds.pop(f'{to_del}')

        self.current_route.remove(dcustid)

    @timed_function(dt=bs.sim.simdt)
    def calc_savings(self):
        """Calculates the potential savings of serving next customer by drone
        If time can be saved, a drone will be launched. Else, nothing is done
        """
        if not self.called:
            return

        dronecount = len(bs.traf.Operations.drone_manager.active_drones)

        if dronecount >= int(self.M):
            return

        # Stop the simulation, we need to determine next drone's launch point
        stack.stack('HOLD')
        self.reroute()

    def reroute(self):
        """Reroutes the truck when the next customer can be served by drone"""
        try:
            truckidx = bs.traf.id.index(self.truckname)
        except ValueError:
            return

        rte = bs.traf.ap.route[truckidx]
        # self.copy = rte.wplat[:]

        wplats = list(map(lambda x: round(x, ndigits=6), rte.wplat))
        wplons = list(map(lambda x: round(x, ndigits=6), rte.wplon))

        # Get customers served from operations module
        # Drones that are otw to their customers should also be counted
        custidx = 1 + bs.traf.Operations.custs_served + \
                    sum(1 for drone in 
                    bs.traf.Operations.drone_manager.active_drones.values() 
                    if drone['del_done'])
        dcustid = self.model.P[0].tour[custidx]
        if dcustid == 0:
            # Depot node, do not serve with drone
            stack.stack("OP")
            stack.stack("FF")
            return

        ncustid = self.model.P[0].tour[custidx + 1]
        # if not self.customers[custid].drone_eligible:
        #     return
        # Terminology:
        # - dcustid: drone customer id (to be handled by a drone)
        # - ncustid: next customer id (after the drone customer)
        # - ocustid: the origin customer that is being routed from
        dcust_latlon = self.customers[dcustid].location
        ncust_latlon = self.customers[ncustid].location
        try:
            dronecustwpidx = find_index_with_tolerance(dcust_latlon, wplats, wplons)
            # print(f'Arriving at customer with id {custidx} in {calc_eta(truckidx, nextcustwpidx)} seconds')
        except ValueError:
            raise ValueError(f"Customer {dcustid} with coordinates {dcust_latlon} not found in route")
        try:
            nextcustwpidx = find_index_with_tolerance(ncust_latlon, wplats, wplons)
        except ValueError:
            raise ValueError(f"Customer {ncustid} with coordinates {ncust_latlon} not found in route")

        self.remove_commands(dcustid, ncustid)

        # Delete old truck route
        print("Removing old route...")
        bs.traf.ap.route[truckidx].delrte(truckidx)
        self.reset_added_cmds()
        print("Route removal done.")
        
        road_route, spd_lims = roadroute(self.G, (bs.traf.lat[truckidx], bs.traf.lon[truckidx]), \
                                                self.customers[ncustid].location)

        road_route_merged = linemerge(road_route)
        wp_commands = self.construct_scenario(road_route_merged, spd_lims)
        # self.plot_graph(road_route, True)
        newcmds = extract_arguments(wp_commands, 
                                    f'ADDTDWAYPOINTS {self.truckname},')

        bs.traf.ap.route[truckidx].addtdwaypoints(truckidx, *newcmds)

        self.add_routepiece()
        self.deploy(truckidx, rte, dcustid)

    def deploy(self, truckidx, rte, dcustid):
        """
        ________________________________PHASE 3________________________________

        Deploys a drone by considering the customer that is to be served by a
        drone. A PuLP model is activated to determine the optimal launch and
        retrieval location of the drone.

        The routing commands are directly inserted to the bs stack.
        """
        # Define look ahead window: drone can be picked up a maximum of M + 1 
        # customers later than the current drone customer
        cust_max = self.model.P[0].tour\
                            [min(self.model.P[0].tour.index(dcustid) + \
                                int(self.M) + 1, 
                                len(self.model.P[0].tour) - 1)]
        cust_max_loc = self.customers[cust_max].location
        max_wp = find_index_with_tolerance(cust_max_loc,
                                            rte.wplat,
                                            rte.wplon)

        # Get matching LR locations on route
        wp_indices = find_wp_indices(rte, self.lr_locs, max_wp)

        L, P, t_ij, t_jk, T_i, T_k, T_ik, B = calc_inputs(
                                truckidx, rte, wp_indices, 
                                self.customers[dcustid].location[0],
                                self.customers[dcustid].location[1],
                                self.cruise_alt, self.cruise_spd, 
                                self.vspd_up, self.vspd_down,
                                self.delivery_time, self.truck_delivery_time)

        # m = LR_Optimizer(L, P, t_ij, t_jk, T_i, T_k, T_ik, B)
        # m.create_model()
        # m.solve()

        mp = LR_PuLP(L, P, t_ij, t_jk, T_i, T_k, T_ik, 15, 10, B)
        mp.create_model()
        mp.solve()

        bs.traf.ap.route[truckidx].addoperationpoints(truckidx, 
            f'{rte.wplat[mp.launch_location]}/{rte.wplon[mp.launch_location]}', 
            'SORTIE', self.sortie_time, f'M{self.vehicle_group}', '1',
            self.customers[dcustid].location[0], 
            self.customers[dcustid].location[1],
            f'{rte.wplat[mp.pickup_location]}/{rte.wplon[mp.pickup_location]}',
            self.cruise_alt / ft, self.cruise_spd / kts,
            self.delivery_time, self.rendezvous_time)

        # if mp.launch_location == 0 or mp.launch_location == 1:
        #     _, dist = kwikqdrdist(bs.traf.lat[0], bs.traf.lon[0], rte.wplat[mp.launch_location], rte.wplon[mp.launch_location])
        #     print(dist)

        # if not kwikqdrdist(bs.traf.lat[0], bs.traf.lon[0], rte.wplat[mp.launch_location], rte.wplon[mp.launch_location])[1] < delivery_dist:
        stack.stack(f'VNAV {self.truckname} ON')
        stack.stack(f'LNAV {self.truckname} ON')
        
        # else:
        #     pass

        stack.stack('OP')
        stack.stack('FF')
    
    @timed_function(dt = bs.sim.simdt * 10)
    def add_routepiece(self):
        if not self.called:
            return

        try:
            truckidx = bs.traf.id.index(self.truckname)
        except ValueError:
            return
        window = int(self.M) + 1
        for u,v in zip(self.current_route[:window], 
            self.current_route[1:1 + window]):
            if (u, v) in self.added_route:
                continue
            args = extract_arguments(self.routing_cmds[f'{u}-{v}'], 
                                    f'ADDTDWAYPOINTS {self.truckname},')
            bs.traf.ap.route[truckidx].addtdwaypoints(truckidx, *args)
            self.added_route.append((u, v))

        for delivery_cmd in self.delivery_cmds:
            if delivery_cmd in self.added_deliveries:
                continue
            if int(delivery_cmd) in self.current_route[:1 + window]:
                args = extract_arguments(self.delivery_cmds[delivery_cmd],
                                    f'ADDOPERATIONPOINTS {self.truckname},')
                bs.traf.ap.route[truckidx].addoperationpoints(truckidx, *args)
                self.added_deliveries.append(delivery_cmd)