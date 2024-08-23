import bluesky as bs 
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.tools.geo import qdrdist
from bluesky.traffic import Route
from math import isclose, sqrt

class DroneManager(Entity):
    def __init__(self):
        super().__init__()
        self.active_drones = {}
        self.completed_ops = {}

    def reset(self):
        self.active_drones = {}
        self.completed_ops = {}

    def add_drone(self, truck, type, UAVnumber, lat_i, lon_i, lat_j, lon_j, lat_k, lon_k, wpname_k, alt, spd, 
                    service_t, recovery_t):
        """Adds a drone to the dictionary of available drones. Available drones is used to keep track of drones in the
        memory of the simulation. Keeps track of (intermediate) destination, spd, alt, service/ recovery time etc.
        args: type, description
        - truck: str, parent truck that spawns the drone
        - type: str, drone type
        - UAVnumber: str, physical UAV number 
        - lat_i: str, lat of spawn location
        - lon_i: str, lon of spawn location
        - lat_j: str, lat of customer location
        - lon_j: str, lon of customer location
        - lat_k: str, lat of rendezvous location
        - lon_k: str, lon of rendezvous location
        - wpname_k: str, waypoint name of rendezvous location
        - alt: str, altitude of the UAV
        - spd: str, speed of the UAV
        - service_t: str, delivery time required for UAV
        - recovery_t: str, rendezvous time required for UAV"""

        available_name = self.get_available_name(UAVnumber, truck)
        self.active_drones[available_name] = {
            'status': False, 'truck': truck, 'type': type, 'lat_i': lat_i, 
            'lon_i': lon_i, 'lat_j': lat_j, 'lon_j': lon_j, 'lat_k': lat_k,
            'lon_k': lon_k, 'wpname_k': wpname_k, 'alt': alt, 'spd': spd,
            'service_time': service_t, 'recovery_time': recovery_t,
            'del_done': False
            }
        return available_name

    def get_available_name(self, UAVnumber, truck):
        """Get available name for drone in memory. Keeps track of number of operations 
        performed by drone with drone number UAVnumber for a specific truck.
        
        Args:
        - UAVnumber: str, physical UAV number
        - truck: str, the name of the truck to which the UAV belongs

        Naming convention:
        --> <Truck single key identifyer>_UAV<UAV physical number>_<launch #>

        e.g. T_UAV_3_1

        Truck single key identifyer = 'T'
        Physical UAV number = '3'
        Launch number = '1' (being launched for the 1st time)
        """
        # Collect all the existing drone names for the given UAVnumber and truck
        existing_names = [
            key.split('_')[-1] for key in self.active_drones.keys() 
            if key.startswith(f'{truck[0]}_UAV{UAVnumber}_')
        ] + [
            key.split('_')[-1] for key in self.completed_ops.keys() 
            if key.startswith(f'{truck[0]}_UAV{UAVnumber}_')
        ]
        
        # Convert the names to integers to find the available number
        used_numbers = sorted(int(num) for num in existing_names)
        
        # Find the smallest available number by checking for gaps in the used numbers
        for i in range(1, len(used_numbers) + 1):
            if i not in used_numbers:
                return f'{truck[0]}_UAV{UAVnumber}_{i}'
        
        # If no gaps were found, return the next consecutive number
        # TODO make names shorter such that BS can display them
        return f'{truck[0]}_UAV{UAVnumber}_{len(used_numbers) + 1}'

    def spawn_drone(self, drone_name):
        """Spawn (create) a drone with drone name.
        args: type, description
        - drone_name: str, name of the drone to be created"""
        data = self.active_drones[drone_name]
        hdg, _ = qdrdist(float(data['lat_i']), float(data['lon_i']), float(data['lat_j']), float(data['lon_j']))
        bs.traf.cre(drone_name, data['type'], data['lat_i'], data['lon_i'], hdg, 8, 0)
        self.active_drones[drone_name]['status'] = 'OTW'

    def route_drone(self, drone_name):
        """Route a drone with drone name.
        args: type, description
        - drone_name: str, name of the drone to be created"""
        data = self.active_drones[drone_name]
        stack.stack(f"ALT {drone_name} {data['alt']}")
        scen_text = f"ADDTDWAYPOINTS {drone_name} "
        for wp in ['i', 'j', 'k']:
            if data['lon_k'] is None and wp == 'k':
                # This means coordinates are given but embedded in the wpname
                data['lat_k'], data['lon_k'] = data['wpname_k'].split('/')
            scen_text += f"{data['lat_' + wp]} {data['lon_' + wp]} {data['alt']} {data['spd']} TURNSPD 3 "

        stack.stack(scen_text)

        drone_id = bs.traf.id.index(drone_name)
        stack.stack(f'ATALT {drone_name} {data["alt"]} LNAV {drone_name} ON')
        stack.stack(f'ATALT {drone_name} {data["alt"]} VNAV {drone_name} ON')
        stack.stack(f"ATALT {drone_name} {data['alt']} SPDAP {drone_name} {data['spd']}")
        
        # Add operation for the delivery
        stack.stack(f"ADDOPERATIONPOINTS {drone_name} {data['lat_j']}/{data['lon_j']} DELIVERY {data['service_time']}")
        # Add operation for the rendezvous
        recovery_t = data['recovery_time']
        stack.stack(f"ADDOPERATIONPOINTS {drone_name} {data['lat_k']}/{data['lon_k']} RENDEZVOUS {recovery_t}")
        # stack.stack(f"ADDOPERATIONPOINTS {data['truck']} {data['wpname_k']} RENDEZVOUS {recovery_t} {drone_name}")

    def complete_sortie(self, drone_name):
        """Ready up a drone rendezvous with drone name, such that it can be picked up by its truck.
        args: type, description
        - drone_name: str, name of the drone to be created"""
        if drone_name in self.active_drones:
            self.active_drones[drone_name]['status'] = 'HOVERING'

    def check_rendezvous(self, drone_name):
        """Spawn (create) a drone with drone name.
        args: type, description
        - drone_name: str, name of the drone to be created"""
        return True if (drone_name in self.active_drones and self.active_drones[drone_name]['status'] == 'HOVERING') \
            else False
    
    def retrieve_drone(self, drone_name):
        """Retrieve (delete) a drone with drone name.
        args: type, description
        - drone_name: str, name of the drone to be created"""
        stack.stack(f"DEL {drone_name}")
        self.active_drones[drone_name]['status'] = True
        self.completed_ops[drone_name] = self.active_drones[drone_name]
        self.active_drones.pop(drone_name, None)

    def drone_available(self, UAVnumber):
        """Check whether a drone is available to be spawned, i.e. whether this drone is already performing a route.
        args: type, description
        - UAVnumber: str, physical UAV number"""
        for drone_id, drone_data in self.active_drones.items():
            if (drone_data['status'] == 'OTW' or drone_data['status'] == 'HOVERING') and\
                    drone_id.split('_')[1] == UAVnumber.split('_')[1]:
                return False
        return True

def get_wpname(wpname, rte, tol=1e-6, prefer_later=True):
    """
    Gets the closest wpname of a wp in a vehicle's route by looking for its coordinates.
    
    Args: type - description
    wpname: str - coordinates in 'lat/lon' format to look up
    rte: route object - route of the vehicle containing waypoint data
    tol: float - tolerance for considering two coordinates as close
    prefer_later: bool - prefer equal waypoints later in the vehicle's route
    
    Returns:
    str: the name of the closest waypoint or None if not found
    """
    acc = int(tol**-1)
    # Parse the input coordinates and round them
    lat, lon = map(float, wpname.split('/'))
    lat = round(lat, acc)
    lon = round(lon, acc)

    closest_wpname = None
    # Initialize min_distance with infinity
    min_distance = float('inf')  

    # Iterate over all waypoints to find the closest one
    for i, (plat, plon) in enumerate(zip(rte.wplat, rte.wplon)):
        # Calculate the Euclidean distance (for simplicity)
        distance = sqrt((plat - lat) ** 2 + (plon - lon) ** 2)

        # Update the closest waypoint if a new minimum distance is found
        if prefer_later:
            condition = distance <= min_distance
        else:
            condition = distance < min_distance
        if condition:
            min_distance = distance
            closest_wpname = rte.wpname[i]

    return closest_wpname