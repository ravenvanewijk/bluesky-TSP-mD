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
        self.routes_to_modify = {}

    def reset(self):
        self.active_drones = {}
        self.routes_to_modify = {}

    def add_drone(self, truck, type, lat_i, lon_i, lat_j, lon_j, lat_k, lon_k, wpname_k, alt, spd):
        available_name = self.get_available_name()
        self.active_drones[available_name] = {'status': False, 'truck': truck, 'type': type, 'lat_i': lat_i, 
                                            'lon_i': lon_i, 'lat_j': lat_j, 'lon_j': lon_j, 'lat_k': lat_k,
                                            'lon_k': lon_k, 'wpname_k': wpname_k, 'alt': alt, 'spd': spd}
        return available_name

    def get_available_name(self):
        nr = len(self.active_drones)
        return 'SP'+str(nr + 1)

    def spawn_drone(self, drone_name):
        # add new get available name here____
        data = self.active_drones[drone_name]
        hdg, _ = qdrdist(float(data['lat_i']), float(data['lon_i']), float(data['lat_j']), float(data['lon_j']))
        bs.traf.cre(drone_name, data['type'], data['lat_i'], data['lon_i'], hdg, 8, 0)
        self.active_drones[drone_name]['status'] = 'OTW'

    def route_drone(self, drone_name):
        data = self.active_drones[drone_name]
        stack.stack(f"ALT {drone_name} {data['alt']}")
        stack.stack(f"{drone_name} ATALT {data['alt']} SPDAP {drone_name} {data['spd']}")
        scen_text = f"ADDTDWAYPOINTS {drone_name} "
        for wp in ['i', 'j', 'k']:
            if data['lon_k'] is None and wp == 'k':
                # This means coordinates are given but embedded in the wpname
                # Add the waypoint but do not add the drone to the routes to modify, since the truck does not have
                # this waypoint yet
                data['lat_k'], data['lon_k'] = data['wpname_k'].split('/')
            elif wp == 'k':
                self.routes_to_modify[drone_name] = {'wpname_k': data['wpname_k'], 'truck': data['truck']}
            
            scen_text += f"{data['lat_' + wp]} {data['lon_' + wp]} {data['alt']} {data['spd']} TURNSPD 5 "

        stack.stack(scen_text)
        stack.stack(f"LNAV {drone_name} ON")
        stack.stack(f"VNAV {drone_name} ON")
        # Add operation for the delivery
        stack.stack(f"ADDOPERATIONPOINTS {drone_name} {data['lat_j']}/{data['lon_j']} DELIVERY 5")

    def complete_sortie(self, drone_name):
        if drone_name in self.active_drones:
            self.active_drones[drone_name]['status'] = 'HOVERING'

    def check_rendezvous(self, drone_name):
        return True if (drone_name in self.active_drones and self.active_drones[drone_name]['status'] == 'HOVERING') \
            else False
    
    def retrieve_drone(self, drone_name):
        stack.stack(f"DEL {drone_name}")

    @timed_function(dt = 10)
    def modify_wp(self):
        to_remove = []
        for drone_name in self.routes_to_modify:
            acrte = Route._routes[drone_name]
            stack.stack(f"ADDOPERATIONPOINTS {drone_name} {acrte.wpname[-1]} RENDEZVOUS 1")
            stack.stack(f"ADDOPERATIONPOINTS {self.routes_to_modify[drone_name]['truck']} \
                    {self.routes_to_modify[drone_name]['wpname_k']} RENDEZVOUS 1 {drone_name}")
            to_remove.append(drone_name)
        
        for key in to_remove:
            self.routes_to_modify.pop(key)

def get_wpname(wpname, truckrte, tol=1e-6):
    """
    Gets the closest wpname of a wp in a truck's route by looking for its coordinates.
    
    Args: type - description
    wpname: str - coordinates in 'lat/lon' format to look up
    acrte: route object - route of the truck containing waypoint data
    acc: int - number of decimal places to consider for rounding coordinates
    tol: float - tolerance for considering two coordinates as close
    
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
    for i, (plat, plon) in enumerate(zip(truckrte.wplat, truckrte.wplon)):
        # Calculate the Euclidean distance (for simplicity)
        distance = sqrt((plat - lat) ** 2 + (plon - lon) ** 2)

        # Update the closest waypoint if a new minimum distance is found
        if distance < min_distance:
            min_distance = distance
            closest_wpname = truckrte.wpname[i]

    return closest_wpname