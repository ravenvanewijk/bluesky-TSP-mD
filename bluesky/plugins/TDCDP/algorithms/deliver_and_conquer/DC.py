import bluesky as bs
import osmnx as ox
import numpy as np
import pandas as pd
from roadroute_lib import roadroute
from shapely.ops import linemerge
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.utils import \
                                                        str_interpret, \
                                                        Customer
from bluesky.plugins.TDCDP.algorithms.deliver_and_conquer.PACO import \
                                                        PACO
from bluesky.tools.geo import kwikqdrdist
from bluesky import stack                                                       
from bluesky.core import Entity


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
    
    @stack.command
    def deliver(self, vehicle_group, M, cruise_alt, cruise_spd, \
                problem_name, *args):
        """Call the deliver and conquer algorithm to solve the routing to a \
        set of customers with a given number of drones.
        This will employ a clustering method to serve each cluster iteratively.
        
        args: type, description
            - vehicle_group: str (int), type of drone(s) to employ. Details are described in Murray's paper and Github
            https://doi.org/10.1016/j.trc.2019.11.003. Refers to specific drone type 10<X>.
            - M: str (int), number of drones
            - cruise_alt: float, cruise altitude of the drone in XXX
            - cruise_spd: float, cruise speed of the drone in XXX
            - problem_name: str, name of the problem used to look up timing table of the truck
            - *args: str (floats), series of floats that describe the customer locations lat/lon and package weight [lbs]
            should be a multiple of 3
            """
        self.load_custs(args)
        self.spawn_truck('TRUCK')


        #_____________PARAMS_____________
        self.delivery_time = 30
        self.sortie_time = 60
        self.rendezvous_time = 60
        self.vehicle_group = vehicle_group
        self.cruise_alt = cruise_alt
        self.cruise_spd = cruise_spd
        #________________________________


        self.calc_basic_tsp(problem_name)
        self.route_basic_tsp()
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
        """Routes the truck according to the TSP obtained by PACO."""
        stack.stack("HOLD")
        delivery_cmds = []
        for u, v in zip(self.model.P[0].tour, self.model.P[0].tour[1:]):
            road_route, spdlims = roadroute(self.G, self.customers[u].location,\
                                                    self.customers[v].location)
            road_route_merged = linemerge(road_route)
            wp_commands = self.construct_scenario(road_route_merged, spdlims)
            stack.stack(wp_commands)
            delivery_cmds.append(
                            f"ADDOPERATIONPOINTS {self.truckname} "
                            f"{self.customers[v].location[0]}/"
                            f"{self.customers[v].location[1]} "
                            f"DELIVERY {self.delivery_time}"
                            )

        for delivery_cmd in delivery_cmds:
            stack.stack(delivery_cmd)

        stack.stack(f"VNAV {self.truckname} ON")
        stack.stack(f"LNAV {self.truckname} ON")
        stack.stack("TRAIL ON")

        stack.stack("OP")


    def construct_scenario(self, road_route, spd_lims):
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
            # Add the text for this waypoint. It doesn't matter if we always add a turn speed, as BlueSky will
            # ignore it if the wptype is set as FLYBY
            # we have to give a speed if we dont specify RTAs, so set the default to 25
            cruisespd = spdlim
            scen_text += f',{wplat},{wplon},{cruise_alt},{cruisespd},{wptype},{wp_turnspd}'

        return scen_text