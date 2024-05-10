import bluesky as bs 
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.tools.geo import qdrdist
from bluesky.traffic import Route
from math import isclose

def init_plugin():
 
    # Addtional initilisation code
 
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'DRONEMANAGER',
 
        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config


class DroneManager(Entity):
    def __init__(self):
        super().__init__()
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

    @timed_function(dt = 10)
    def add_rendezvous(self):
        for drone_name in self.active_drones:
            if self.active_drones[drone_name]['status'] == 'OTW' and \
                    len(self.active_drones[drone_name]['wpname_k'].split('/')) == 2:
                truckrte = Route._routes[self.active_drones[drone_name]['truck']]
                wpname_k = get_wpname(self.active_drones[drone_name]['wpname_k'], truckrte, acc=11)
                if wpname_k is None:
                    continue
                print(f'Adding operation point k for {drone_name}')
                wpid_k = truckrte.wpname.index(wpname_k.upper())
                self.active_drones[drone_name]['lat_k'] = truckrte.wplat[wpid_k]
                self.active_drones[drone_name]['lon_k'] = truckrte.wplon[wpid_k]
                self.active_drones[drone_name]['wpname_k'] = wpname_k
                data = self.active_drones[drone_name]
                # Adding the drone to this dictionary ensures that its waypoint (and the truck) at k will be operations
                self.routes_to_modify[drone_name] = {'wpname_k': data['wpname_k'], 'truck': data['truck']}

def get_wpname(wpname, acrte, acc=6, tol=1e-6):
    """Gets the wpname of a wp by looking for its coords.
    args: type, description
    wpname: string, coordinates lat lon separated by /, wp to look up
    acrte: route object, route of the AC
    tol: float, tolerance for coordinate matching
    """
    lat, lon = map(float, wpname.split('/'))
    lat = round(lat, acc)
    lon = round(lon, acc)
    
    for i, (plat, plon) in enumerate(zip(acrte.wplat, acrte.wplon)):
        if isclose(plat, lat, abs_tol=tol) and isclose(plon, lon, abs_tol=tol):
            return acrte.wpname[i]