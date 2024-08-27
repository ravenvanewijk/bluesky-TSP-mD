"""
This script is the core of the DC (delivery and conquer) plugin

Its goal is to perform live routing of a truck with multiple drones to its 
disposal. Instead of regular routing (which is all determined beforehand), this
script determines launch and retrieval locations of drones on the go.

It requires the following plugins:

--> 'SPDAP', 'OPERATIONS', 'TDACTWP', 'TDAUTOPILOT', 'TDRoute'

Add these to the settings.cfg along with the name of this plugin itself ('DC')

CREATED BY RAVEN VAN EWIJK AS A PART OF THE AEROSPACE ENGINEERING MASTER'S 
THESIS.
"""

import bluesky as bs
import osmnx as ox
import numpy as np
import pandas as pd
import geopandas as gdp
import random
import matplotlib.pyplot as plt
from roadroute_lib import roadroute, construct_scenario
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
    extract_arguments,
    plot_route,
    get_drone_id,
    get_available_drone
)
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.opt_inputs import (
    calc_inputs,
    raw_data,
    calc_truck_ETA,
    calc_truck_ETA2,
    calc_drone_ETA
)
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.PACO import PACO
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.op_opt import (
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
        self.seed = 11 # None for random seed, enter a number to add a seed

    def reset_added_cmds(self):
        self.added_route = []
        self.added_deliveries = []
        self.trucketa = []
        self.reconeta = []
    
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

        #_____________PHYSICAL_PARAMS_____________
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

        # Dictionaries to store the text and etas, will be used later
        self.routing_cmds = {}
        self.delivery_cmds = {}
        self.routing_etas = {}

        # Lists of current routing etas and timeout counter
        self.timeout = 0
        self.trucketa = []
        self.reconeta = []

        self.calc_basic_tsp(problem_name)
        self.route_basic_tsp()
        self.gen_lr_locs()

        # Create a copy of the tour
        self.current_route = self.model.P[0].tour[:]
        self.added_route = []
        self.added_deliveries = []
        # define window of customers that are being added to the route.
        # Larger window makes bs slower, but gives more opportunities for 
        # L&R operations
        # Measured in number of customers
        self.window = int(self.M) + 2

        # PuLP model parameters
        self.alpha = 15
        self.beta = 10
        self.bigM = 10000

        # set called to True such that the routing begins
        self.called = True

    def spawn_truck(self, truckname):
        """Spawn the truck at the depot location.
        
        Args:
            - truckname: str, the name of the truck
        """
        self.truckname = truckname
        self.recon_name = "RECON"
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
            bs.scr.echo('You missed a customer value,'
                        'argument number must be a multiple of 3.')
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
        """Generate locations where an operation, i.e. a sortie or rendezvous
        are possible on the graph that has been loaded
        
        args: type, description
            - cust_only: bool, whether or not customer locations are the 
                only locations where an operation can be conducted"""

        if not self.G:
            raise Exception("Graph G should first be loaded. Run the command\
                            load_graph first with a correct filepath.")
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
                            [sample_points(self.G, n, self.seed), 
                            self.lr_locs], ignore_index=True
                                        )
        
        # self.plot_graph(True)

    def calc_basic_tsp(self, problem_name):
        """Calculate TSP path without any drones, only 1 truck
        Uses Ant Colony Optimization to determine its route"""

        folder = 'bluesky/plugins/TDCDP/algorithms/deliver_and_conquer/timing_tables'
        ST = pd.read_csv(f'{folder}/tbl_truck_travel_data_{problem_name}.csv')
        ST['1/time'] = ST['time [sec]'].apply(lambda x: 1/x if x != 0 
                                                            else np.nan)

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

        self.model = PACO(1, 10, 0.3, 3, 10, 200, eta, 5, self.seed)
        self.model.simulate()
        # Temporary hardcoding so get get the same tour every time
        self.model.P[0].tour = [0, 12, 19, 1, 11, 4, 6, 3, 21, 22, 13, 23, 17, 16, 20, 9, 2, 10, 25, 5, 8, 7, 15, 18, 14, 24, 0]

    def route_basic_tsp(self):
        """
        ________________________________PHASE 1________________________________
        Routes the truck according to the TSP obtained by PACO."""
        stack.stack("HOLD")
        delivery_cmds = []
        for u, v in zip(self.model.P[0].tour, self.model.P[0].tour[1:]):
            road_route, spd_lims, etas = roadroute(self.G, 
                                            self.customers[u].location,
                                            self.customers[v].location)
            road_route_merged = linemerge(road_route)
            # Update customer location with Bluesky location on route
            self.customers[u].location = \
                np.around([road_route_merged.xy[1][0], 
                            road_route_merged.xy[0][0]], 6)
            wp_commands = construct_scenario(self.truckname, road_route_merged, 
                                                                    spd_lims)
            self.routing_cmds[f'{u}-{v}'] = wp_commands
            self.routing_etas[f'{u}-{v}'] = etas
            self.trucketa.extend(etas)
            stack.stack(wp_commands)
            if v == 0:
                continue
            op_text = (f"ADDOPERATIONPOINTS {self.truckname},"
                f"{self.customers[v].location[0]}/"
                f"{self.customers[v].location[1]},"
                f"DELIVERY,{self.truck_delivery_time}")
            delivery_cmds.append(op_text)
            self.delivery_cmds[f'{v}'] = op_text

        for delivery_cmd in delivery_cmds:
            stack.stack(delivery_cmd)

        stack.stack("TRAIL ON")
        stack.stack("OP")

    def plot_graph(self, lines=[], cust=False):
        """Plots the graph of the selected gpkg file as well as customer 
        locations"""
        # Plot city graph
        fig, ax = ox.plot_graph(self.G, show=False, close=False)
        # Plot the customers
        if cust and self.lr_locs is not None:
            locs_scatter = ax.scatter([point.x for _, point in 
                            self.lr_locs.items()],
                            [point.y for _, point in self.lr_locs.items()],
                            c='red', s=8, zorder=5, label='L&R locations')
            # Show the plot with a legend
            ax.legend(handles=[locs_scatter])

        for line in lines:
            x, y = line.xy
            ax.plot(x, y, marker='o')  # Plot the line with markers at vertices
            ax.plot(x[-1],y[-1],'rs') 

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
        try:
            self.routing_cmds.pop(f'{dcustid}-{ncustid}')
        except KeyError:
            pass
        try:
            self.delivery_cmds.pop(f'{dcustid}')
        except KeyError:
            pass
        # Alter current route of the truck
        if dcustid in self.current_route:
            while not self.current_route.index(dcustid) == 0:
                # Account for the case where truck has made one or more 
                # deliveries during the delivery of the drone(s)
                to_del = self.current_route[0]
                self.current_route.remove(to_del)
                self.delivery_cmds.pop(f'{to_del}')
        try:
            self.current_route.remove(dcustid)
        except ValueError:
            pass

    @timed_function(dt=bs.sim.simdt)
    def calc_savings(self):
        """Calculates the potential savings of serving next customer by drone
        If time can be saved, a drone will be launched. Else, nothing is done
        """
        if not self.called:
            return

        # Drones that are otw to their customers should also be counted
        # Get customers served from operations module
        custidx = 1 + bs.traf.Operations.custs_served + \
            sum(1 for drone in 
            bs.traf.Operations.drone_manager.active_drones.values()
            if not drone['del_done'])

        dcustid = self.model.P[0].tour[custidx]
        ncustid = self.model.P[0].tour[min(custidx + 1,     
                                        len(self.model.P[0].tour) - 1)]

        # if dcustid != self.current_route[0] and self.current_route[0] != 0 \
        #     and dcustid in self.current_route:
        # if dcustid != self.current_route[0] \
        #     and dcustid in self.current_route:
        if len(self.current_route) + custidx > len(self.model.P[0].tour):
            # Remove commands if truck has made deliveries without launching
            # a drone
            self.remove_commands(dcustid, ncustid)
        
        if self.timeout > 0:
            self.timeout -= 1
            return

        dronecount = len(bs.traf.Operations.drone_manager.active_drones)

        if dronecount >= int(self.M):
            return
            
        # Enter phase 2 if we have a drone at our disposal
        # Phase 2 is the reconaissance phase to scout if we can save time
        #_______________________________PHASE 2_______________________________
        try:
            truckidx = bs.traf.id.index(self.truckname)
        except ValueError:
            # Truck is deleted, stop the routing
            self.called = False
            return 
        rte = bs.traf.ap.route[truckidx]
        wplats = list(map(lambda x: round(x, ndigits=6), rte.wplat))
        wplons = list(map(lambda x: round(x, ndigits=6), rte.wplon))

        if dcustid == 0 or not self.customers[dcustid].drone_eligible:
            # Depot node, do not serve with drone
            # self.resume()
            return

        # Terminology:
        # - dcustid: drone customer id (to be handled by a drone)
        # - ncustid: next customer id (after the drone customer)

        dcust_latlon = self.customers[dcustid].location
        ncust_latlon = self.customers[ncustid].location
        try:
            dronecustwpidx = find_index_with_tolerance(dcust_latlon, wplats, 
                                                                        wplons)
        except ValueError:
            raise ValueError(f"Customer {dcustid} with coordinates {dcust_latlon} not found in route")
        try:
            nextcustwpidx = find_index_with_tolerance(ncust_latlon, wplats, 
                                                                        wplons)
        except ValueError:
            raise ValueError(f"Customer {ncustid} with coordinates {ncust_latlon} not found in route")

        # Create second truck to perform calculations
        bs.traf.cre(self.recon_name, "Truck", bs.traf.lat[truckidx], 
                                        bs.traf.lon[truckidx],
                                        bs.traf.hdg[truckidx],
                                        bs.traf.alt[truckidx],
                                        bs.traf.tas[truckidx])
        reconidx = bs.traf.id.index(self.recon_name)

        max_cust, uav_lat, uav_lon, _ = self.reroute_truck(reconidx, dcustid, 
                                                                ncustid)

        rte_recon = bs.traf.ap.route[reconidx]

        penalty = self.reschedule_existing_drones(reconidx, rte_recon, dcustid)

        # Find out what wp should be looked up to match the next customer
        # in both occasions
        reconwpidx = find_index_with_tolerance(
                    self.customers[max_cust].location, 
                    rte_recon.wplat, rte_recon.wplon)

        truckwpidx = find_index_with_tolerance(
                    self.customers[max_cust].location, rte.wplat, rte.wplon)
        
        # plot_route(self.G, [rte.wplat[:truckwpidx + 1]], 
        #                     [rte.wplon[:truckwpidx + 1]],
        #                     f'Route with serving customer {dcustid} by truck',
        #                     ['Truck'],
        #                     dcust_latlon[0],
        #                     dcust_latlon[1],
        #                     f'Customer {dcustid}')
        # plot_route(self.G, [rte_recon.wplat[:reconwpidx + 1], uav_lat], 
        #                     [rte_recon.wplon[:reconwpidx + 1], uav_lon],
        #                     f'Route with serving customer {dcustid} by drone',
        #                     ['Truck', 'Drone'])

        # Calculate the eta to next customer for both cases
        eta_r = calc_truck_ETA2(self.trucketa[:truckwpidx + 1], 
                                rte.operation_duration[:truckwpidx + 1])
        eta_m = calc_truck_ETA2(self.reconeta[:reconwpidx + 1], 
                                rte_recon.operation_duration[:reconwpidx + 1]) 

        bs.traf.Operations.recondelete(self.recon_name)

        success = 'sufficient savings' if eta_m + penalty < eta_r else \
                    'insufficient savings'
        print(f'Reconaissance successfull, yielded in {success}')
        if eta_r < eta_m + penalty:
            # We asserted the potential of serving next customer by drone
            # This however adds time to the makespan
            # Therefore we dont change plans
            # Set timeout such that we don't immediately try again
            # timeout in seconds
            self.timeout = 180 / bs.sim.simdt
            return
        # The modified time reduces the makespan
        # Therefore we change the plans and serve the next customer by drone
        print(f'Serving customer {dcustid} by drone')

        # Reroute the 'real' truck
        # This is phase 3 starting here below...
        max_cust, uav_lat, uav_lon, child = self.reroute_truck(truckidx, 
                                                            dcustid, ncustid)
        self.reschedule_existing_drones(truckidx, rte, dcustid, child)

        # plot_route(self.G, [rte.wplat[:truckwpidx + 1], uav_lat], 
        #                     [rte.wplon[:truckwpidx + 1], uav_lon],
        #                     f'Route with serving customer {dcustid} by drone non recon',
        #                     ['Truck', 'Drone'],
        #                     dcust_latlon[0],
        #                     dcust_latlon[1],
        #                     f'Customer {dcustid}')
        # print(len(bs.traf.Operations.drone_manager.active_drones))

    def reroute_truck(self, truckidx, dcustid, ncustid):
        """Reroutes the truck when the next customer can be served by drone
       
        Args: type, description
            - truckidx: int, bs identifyer of the truck 
            - dcustid: int, id of the drone customer being served
            - ncustid: int, id of next customer 
            - recon: bool, whether or not the deployment is using a 
                reconaissance truck"""

        recon = True if bs.traf.id[truckidx] == self.recon_name else False
        if not recon:
             # Delete old truck route
            bs.traf.ap.route[truckidx].delrte(truckidx)
            self.reset_added_cmds()
        

        # Current position is position A
        A = (bs.traf.lat[truckidx], bs.traf.lon[truckidx])
        # A serves as input for the rerouting to launches
        # If there aren't any launches to reroute to, then A itself will be 
        # returned in the form of B
        # Else the last launch location that is added to the route is returned
        B = self.reroute_to_operations(truckidx, A, recon)

        road_route, spd_lims, etas = roadroute(self.G, 
                            B,
                            self.customers[ncustid].location)

        road_route_merged = linemerge(road_route)
        wp_commands = construct_scenario(self.truckname, road_route_merged, 
                                                                    spd_lims)

        # self.plot_graph(road_route, True)
        newcmds = extract_arguments(wp_commands, 
                                    f'ADDTDWAYPOINTS {self.truckname},')
        try:
            rte = bs.traf.ap.route[truckidx]
            last_lat = rte.wplat[-1]
            last_lon = rte.wplon[-1]
            if last_lat == float(newcmds[0]) and last_lon == float(newcmds[1]):
                # skip the first wp if we already added that one
                startidx = 6
            else:
                startidx = 0
        except IndexError:
            startidx = 0
        bs.traf.ap.route[truckidx].addtdwaypoints(truckidx,*newcmds[startidx:])
        if recon:
            self.reconeta.extend(etas[startidx//6:])
            self.add_recon_routepiece()
        else:
            self.remove_commands(dcustid, ncustid)
            self.trucketa.extend(etas[startidx//6:])
            self.add_routepiece()

        max_cust_considered, uav_lat, uav_lon, child = self.deploy_new_drone(
                                                truckidx, dcustid, ncustid)

        return max_cust_considered, uav_lat, uav_lon, child

    def reroute_to_operations(self, truckidx, A, recon):

        # Filter the active drones
        # Should not be set to status == True, since then i (the launch) has 
        # already been passed. It should also have the real truck as its parent
        # We also want to add drones that are performing their rendezvous,
        # since we do not want to reroute these
        op_drones = {
            drone_id: drone_info for drone_id, drone_info in 
                bs.traf.Operations.drone_manager.active_drones.items()
            if (
                # drone_info['truck'] must be equal to self.truckname 
                drone_info['truck'] == self.truckname 
                and (
                    # Case 1: The drone is not active
                    # Reroute to its sortie if this is the case
                    not drone_info['status']
                    # Case 2: The drone's 'op_type' is 'RENDEZVOUS'
                    # Reroute to where it is performing the rendezvous
                    or bs.traf.Operations.operational_states.get
                            (drone_id, {}).get('op_type') == ['RENDEZVOUS']
                    )
                )
                    }
        
        # In case we dont have any drones to launch, we want the end (B)
        # Location to be equal to the start (A) location, since we do not add
        # any additional linepieces here
        B = A

        for drone in op_drones:
            # TODO implement small TSP here of up to 4 drones such that we
            # also get inbetween optimality
            op_loc = 'i' if drone not in bs.traf.Operations.operational_states\
                            else 'k'
            B = (op_drones[drone][f'lat_{op_loc}'], 
                        op_drones[drone][f'lon_{op_loc}'])
            if A == B:
                if op_loc == 'k':
                    # SPECIAL CASE: we're rerouting to a drone that is already
                    # preparing to be picked up.
                    bs.traf.ap.route[truckidx].addoperationpoints(truckidx, 
                        f"{A[0]}/{A[1]}", 'RENDEZVOUS', self.rendezvous_time,
                                                                    drone)
                continue
            road_route, spd_lims, etas = roadroute(self.G, A, B)

            road_route_merged = linemerge(road_route)
            wp_commands = construct_scenario(self.truckname, road_route_merged, 
                                                                    spd_lims)
            newcmds = extract_arguments(wp_commands, 
                                        f'ADDTDWAYPOINTS {self.truckname},')

            bs.traf.ap.route[truckidx].addtdwaypoints(truckidx, *newcmds)

            if op_loc == 'k':
                # SPECIAL CASE: we're rerouting to a drone that is already
                # preparing to be picked up.
                bs.traf.ap.route[truckidx].addoperationpoints(truckidx, 
                    f"{B[0]}/{B[1]}", 'RENDEZVOUS', self.rendezvous_time,drone)

            if recon:
                self.reconeta.extend(etas)

            else:
                self.trucketa.extend(etas)

            A = B
        
        return B

    def deploy_new_drone(self, truckidx, dcustid, ncustid):
        """
        ________________________________PHASE 3________________________________

        Deploys a NEW drone by considering the customer that is to be served by
        a drone. A PuLP model is activated to determine the optimal launch and
        retrieval location of the drone.

        The routing commands are directly inserted to bs,
        such that these can be used for calculations as well as to ensure 
        these commands can directly be obeyed.

        Args: type, description
            - truckidx: int, bs identifyer of the truck or recon truck
            - dcustid: int, number of the drone customer being served
            - recon: bool, whether or not the deployment is using a 
                reconaissance truck
        """
        rte = bs.traf.ap.route[truckidx]
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

        # conditional parameters
        if bs.traf.id[truckidx] == self.recon_name:
            eta = self.reconeta
            recon = True
        else:
            eta = self.trucketa
            recon = False
        
        L, P, t_ij, t_jk, T_i, T_k, T_ik, B = calc_inputs(
                                truckidx, rte, eta, rte.operation_duration, 
                                wp_indices, 
                                self.customers[dcustid].location[0],
                                self.customers[dcustid].location[1],
                                self.cruise_alt, self.cruise_spd, 
                                self.vspd_up, self.vspd_down,
                                self.delivery_time, self.truck_delivery_time)

        # Launch before we arrive at next customer
        next_idx = find_index_with_tolerance(self.customers[ncustid].location,
                                            rte.wplat,
                                            rte.wplon)
        L_id = list(L).index(next_idx)
        # Slice such that we can only get up until that idx for launching                                   
        L = L[:L_id + 1]

        # Call the linear programming module to solve the problem
        # This gives us an optimal launch and retrieval location
        mp = LR_PuLP(L, P, t_ij, t_jk, T_i, T_k, T_ik, self.alpha, self.beta,
                                                self.bigM)
        mp.create_model()
        mp.solve()
        
        truck_name = bs.traf.id[truckidx]
        truck_drones = {drone_id: drone_info for drone_id, drone_info in
                        bs.traf.Operations.drone_manager.active_drones.items() 
                        if drone_info['truck']==truck_name}
        drone_id = get_available_drone(truck_drones)
        # Pass the optimized operation and its location to the routing module
        # This module will take care of the entire operation
        child = bs.traf.ap.route[truckidx].addoperationpoints(truckidx, 
            f'{rte.wplat[mp.launch_location]}/{rte.wplon[mp.launch_location]}', 
            'SORTIE', self.sortie_time, f'M{self.vehicle_group}', drone_id,
            self.customers[dcustid].location[0], 
            self.customers[dcustid].location[1],
            f'{rte.wplat[mp.pickup_location]}/{rte.wplon[mp.pickup_location]}',
            self.cruise_alt / ft, self.cruise_spd / kts,
            self.delivery_time, self.rendezvous_time)

        uav_lat = [rte.wplat[mp.launch_location], 
                self.customers[dcustid].location[0], 
                rte.wplat[mp.pickup_location]]
        uav_lon = [rte.wplon[mp.launch_location], 
                self.customers[dcustid].location[1], 
                rte.wplon[mp.pickup_location]]
        # Continue the simulation
        if not recon:
            # Add information on the current truck waiting time
            bs.traf.Operations.drone_manager.active_drones[child]['w_t'] = \
                                        mp.truck_waiting_time

            self.resume()
        
        return cust_max, uav_lat, uav_lon, child
    
    @timed_function(dt = bs.sim.simdt * 10)
    def add_routepiece(self, recon=None):
        """Function to add routepiece(s) and delivery commands.
        Keeps into account the commands that have already been added.
        If it is a reconaissance run, then we dont want to keep these into 
        account, and just add the route.
        If its not a reconaissance run but a real run, we also want to save
        the commands that are given such that they are not added again.
        
        Args: type, description
            - recon: str (if used), reconaissance truck name"""

        if not self.called:
            return

        try:
            truckidx = bs.traf.id.index(self.truckname)
        except ValueError:
            return

        for u,v in zip(self.current_route[:self.window], 
            self.current_route[1:1 + self.window]):
            if (u, v) in self.added_route and recon is None:
                # If the route already has been added, skip the routepiece
                continue
            # Else continue and add the route
            # Fetch argument from the string
            args = extract_arguments(self.routing_cmds[f'{u}-{v}'], 
                                    f'ADDTDWAYPOINTS {self.truckname},')
            self.trucketa.extend(self.routing_etas[f'{u}-{v}'])
            # Parse the command directly, this way it is added without a 
            # bs timetick
            bs.traf.ap.route[truckidx].addtdwaypoints(truckidx, *args)

            # Store the route such that it will not be added again
            self.added_route.append((u, v))

        for delivery_cmd in self.delivery_cmds:
            if delivery_cmd in self.added_deliveries and recon is None:
                # If the delivery already has been added, skip the routepiece
                continue
            # Else continue and add the delivery
            # ...But first check whether the wp of that delivery has been
            # given to the truck already (should be in the window)
            if int(delivery_cmd) in self.current_route[:1 + self.window]:
                # Fetch arguments from the string
                args = extract_arguments(self.delivery_cmds[delivery_cmd],
                                    f'ADDOPERATIONPOINTS {self.truckname},')
                # Parse the command directly, this way it is added without a 
                # bs timetick
                bs.traf.ap.route[truckidx].addoperationpoints(truckidx, *args)

                # Store the delivery such that it will not be added again
                self.added_deliveries.append(delivery_cmd)

    def add_recon_routepiece(self):
        try:
            truckidx = bs.traf.id.index(self.recon_name)
        except ValueError:
            return

        if len(self.current_route) <= 2:
            # correction for dronecust not needed for last piece
            cor = 0
        else:
            # Add all routepieces in window, except for dronecust
            cor = 1
            
        for u,v in zip(self.current_route[cor:cor+self.window], 
            self.current_route[1+cor:1 + cor + self.window]):
            # The drone customer is the first one in line
            # Fetch argument from the string
            args = extract_arguments(self.routing_cmds[f'{u}-{v}'], 
                                    f'ADDTDWAYPOINTS {self.truckname},')

            self.reconeta.extend(self.routing_etas[f'{u}-{v}'])
            # Parse the command directly, this way it is added without a 
            # bs timetick
            bs.traf.ap.route[truckidx].addtdwaypoints(truckidx, *args)


        for delivery_cmd in self.delivery_cmds:
            # Add delivery commands in window
            if int(delivery_cmd) in self.current_route[1:2 + self.window]:
                # Fetch arguments from the string
                args = extract_arguments(self.delivery_cmds[delivery_cmd],
                                    f'ADDOPERATIONPOINTS {self.truckname},')
                # Parse the command directly, this way it is added without a 
                # bs timetick
                bs.traf.ap.route[truckidx].addoperationpoints(truckidx, *args)

    def reschedule_existing_drones(self, truckidx, rte, dcustid, child=None):

        recon = True if bs.traf.id[truckidx] == self.recon_name else False
        penalty = 0
        # Filter drones that actually exist (i.e. have truck as parent)
        truck_drones = {drone_id: drone_info for drone_id, drone_info in
                        bs.traf.Operations.drone_manager.active_drones.items() 
                        if drone_info['truck']==self.truckname}
        for drone in truck_drones:
            # If the drone is the child that we just spawned, we do not have to
            # do anything
            if drone == child or bs.traf.Operations.operational_states.get \
                            (drone, {}).get('op_type') == ['RENDEZVOUS']:
                continue

            # Find new optimum of retrieval local that we can achieve on the 
            # truck route, along with the penalty that we get from this 
            # rerouting w.r.t the originally planned route
            # In case we are rescheduling the real drones we actually do not 
            # care about the penalty but we obtain it anyway
            wpid_k, penalty_i = self.recalculate_k(
                                    truckidx, rte, drone, dcustid
                                            )
            penalty += penalty_i
            lat_k = rte.wplat[wpid_k]
            lon_k = rte.wplon[wpid_k]
            wpname_k = rte.wpname[wpid_k]

            if not recon:
                print(f"Changed pickup point of {drone} to {wpname_k} " +
                        f"during spawning drone {child}")
                # Modify parameters of drone manager first
                bs.traf.Operations.drone_manager.active_drones[drone]\
                                            ['lat_k'] = lat_k
                bs.traf.Operations.drone_manager.active_drones[drone]\
                                            ['lon_k'] = lon_k
                bs.traf.Operations.drone_manager.active_drones[drone]\
                                            ['wpname_k'] = wpname_k

            if bs.traf.Operations.drone_manager.active_drones[drone]\
                                    ['status'] != False:
                # This indicates whether the drone is in air or still in 
                # the back of the truck. In this case it is in the air
                # So we just have to re-add the rendezvous

                # Add the new RENDEZVOUS point of the route
                bs.traf.ap.route[truckidx].addoperationpoints(truckidx, 
                    wpname_k, 'RENDEZVOUS', self.rendezvous_time, drone)

                # Only if the drone is in air we want to modify the last wp
                # If it is not spawned yet the routing commands are not 
                # given yet so we can omit these. But in this case we do have 
                # to modify it manually
                # steps:
                # 1. Remove last wp (wp_k)
                # 2. add new wpk
                # 3. re-add operation point point
                droneidx = bs.traf.id.index(drone)
                dronerte = bs.traf.ap.route[droneidx]
                wp_to_del = get_wpname(
                    f"{truck_drones[drone]['lat_k']}" +
                    f"/{truck_drones[drone]['lon_k']}",
                    dronerte)
                cur_wp = dronerte.iactwp
                # Delete old wp k
                bs.traf.ap.route[droneidx].deltdwpt(droneidx, wp_to_del)
                # Add new wp k
                bs.traf.ap.route[droneidx].addtdwaypoints(droneidx,
                                lat_k, lon_k, self.cruise_alt, 
                                self.cruise_spd, 'TURNSPD', 3)
                # Add the rendezvous to new k 
                bs.traf.ap.route[droneidx].addoperationpoints(droneidx, 
                    f'{lat_k}/{lon_k}', 'RENDEZVOUS', self.rendezvous_time)
                dronerte.direct(droneidx, dronerte.wpname[cur_wp])
            else:
                # Here we want to re-add the sortie, because we know i is 
                # in the route (we design for that)
                # We have the new wp k as well, and we know the drone has not 
                # been launched yet. So, add entire sortie
                args = (
                    truckidx, f"{truck_drones[drone]['lat_i']}" +
                                f"/{truck_drones[drone]['lon_i']}",
                    'SORTIE', self.sortie_time, f'M{self.vehicle_group}', 
                    get_drone_id(drone), truck_drones[drone]['lat_j'],
                    truck_drones[drone]['lon_j'],
                    f"{truck_drones[drone]['lat_k']}" +
                        f"/{truck_drones[drone]['lon_k']}", 
                    self.cruise_alt / ft, self.cruise_spd / kts,
                    self.delivery_time, self.rendezvous_time
                )

                if not recon:
                    # Delete the old UAV, will be immediately replaced by 
                    # adding the new sortie
                    old_w_t = bs.traf.Operations.drone_manager.active_drones\
                                    [drone]['w_t']
                    del bs.traf.Operations.drone_manager.active_drones[drone]

                bs.traf.ap.route[truckidx].addoperationpoints(*args)

                if not recon:
                    bs.traf.Operations.drone_manager.active_drones[drone]\
                        ['w_t'] = old_w_t + penalty_i

        return penalty

    def recalculate_k(self, truckidx, rte, drone, dcustid):

        active_drones = bs.traf.Operations.drone_manager.active_drones
        # Define look ahead window: drone can be picked up a maximum of M + 1 
        # customers later than the current drone customer
        cust_max = self.model.P[0].tour\
                            [min(self.model.P[0].tour.index(dcustid) + \
                                int(self.M), 
                                len(self.model.P[0].tour) - 1)]
        cust_max_loc = self.customers[cust_max].location
        max_wp = find_index_with_tolerance(cust_max_loc,
                                            rte.wplat,
                                            rte.wplon)

        eta = self.reconeta if bs.traf.id[truckidx] == self.recon_name else \
                                                            self.trucketa

        # Get matching LR locations on route
        wp_indices = find_wp_indices(rte, self.lr_locs, max_wp)

        best_obj = np.inf
        best_wp = None

        # find launch wp
        try:
            wpid_i = find_index_with_tolerance(
                (active_drones[drone]['lat_i'], active_drones[drone]['lon_i']), 
                rte.wplat, rte.wplon)
        except ValueError:
            # wpid_i has been passed already, because the drone has been 
            # launched. Any wp can be selected now
            # Set wpid_i to negative so we dont catch the next if statement
            # Also ensures we get empty lists for the truck eta calcs,
            # such that the truck eta is 0
            wpid_i = -len(eta)
        for wp in wp_indices:
            # We want to calculate the total distance the drone has to traverse
            # And at what time the it could be at a certain potential wp k 
            if wpid_i >= wp:
                # We don't want to pick up the drone at a point we have already
                # passed for the launch
                continue
            
            loc_A = (active_drones[drone]['lat_i'], active_drones[drone]\
                        ['lon_i']) if not active_drones[drone]['status'] \
                    else (bs.traf.lat[bs.traf.id.index(drone)], 
                            bs.traf.lon[bs.traf.id.index(drone)])
            #TODO later: streamline this because we dont have to recalculate 
            # the ij a ton of times. only once is good in most cases
            if active_drones[drone]['del_done'] and active_drones[drone]\
                                                            ['status']!= False:
                droneidx = bs.traf.id.index(drone) 
                # delivery done so we dont have to account for time to customer
                # logic for otw back
                drone_t_ij = 0
                _, dist_jk = kwikqdrdist(loc_A[0], loc_A[1],
                    rte.wplat[wp],
                    rte.wplon[wp])
                drone_t_jk = calc_drone_ETA(dist_jk, self.cruise_spd, 
                        self.vspd_up, self.vspd_down, self.cruise_alt, 3.5,
                        bs.traf.tas[droneidx], bs.traf.vs[droneidx])

            else:
                # Drone has either not been spawned yet or is on its way to 
                # the customer
                if active_drones[drone]['status'] != False:
                    droneidx = bs.traf.id.index(drone) 
                    # logic for otw to wp j
                    # we need to add the drone speed to the calculations
                    _, dist_ij = kwikqdrdist(loc_A[0], loc_A[1],
                        active_drones[drone]['lat_j'], 
                        active_drones[drone]['lon_j'])

                    drone_t_ij = calc_drone_ETA(dist_ij, self.cruise_spd, 
                        self.vspd_up, self.vspd_down, self.cruise_alt, 3.5,
                            bs.traf.tas[droneidx], bs.traf.vs[droneidx]) + \
                                    self.delivery_time
                else:
                    # drone has not been spawned, we can use stationary 
                    # conditions
                    _, dist_ij = kwikqdrdist(loc_A[0], loc_A[1],
                        active_drones[drone]['lat_j'], 
                        active_drones[drone]['lon_j'])

                    drone_t_ij = calc_drone_ETA(dist_ij, self.cruise_spd, 
                        self.vspd_up, self.vspd_down, self.cruise_alt, 3.5) +\
                            self.delivery_time
                
                _, dist_jk = kwikqdrdist(
                    active_drones[drone]['lat_j'], 
                    active_drones[drone]['lon_j'],
                    rte.wplat[wp],
                    rte.wplon[wp])
                drone_t_jk = calc_drone_ETA(dist_jk, self.cruise_spd, 
                        self.vspd_up, self.vspd_down, self.cruise_alt, 3.5)

            drone_t = drone_t_ij + drone_t_jk
            # Truck t from curloc to new potential k 
            truck_t = calc_truck_ETA2(eta[:wp], 
                                        rte.operation_duration[:wp]) 
            drone_eta = drone_t + \
                calc_truck_ETA2(eta[:wpid_i], rte.operation_duration[:wpid_i])

            waiting_time_truck = max(drone_eta - truck_t, 0)
            waiting_time_drone = max(truck_t - drone_eta, 0)
            obj = self.bigM * waiting_time_truck + drone_t + \
                                                    waiting_time_drone
            if obj < best_obj:
                best_wp, best_truck_waiting_time, best_obj = \
                                wp, waiting_time_truck, obj
                w = (waiting_time_truck, drone_t)
                
        
        old_wait = active_drones[drone]['w_t']

        penalty = best_truck_waiting_time - old_wait
        return best_wp, penalty

    def resume(self):
        """Method to resume the simulation"""
        if self.truckname not in bs.traf.Operations.operational_states:
            stack.stack(f'VNAV {self.truckname} ON')
            stack.stack(f'LNAV {self.truckname} ON')
        # uncomment for fast sim
        # debug --> slower so no ff
        # stack.stack("FF")