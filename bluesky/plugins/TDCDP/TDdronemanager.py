import bluesky as bs 
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.tools.geo import qdrdist

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

    def add_drone(self, truck, type, lat_i, lon_i, lat_j, lon_j, lat_k, lon_k, wpname_k, alt, spd):
        available_name = self.get_available_name()
        self.active_drones[available_name] = {'status': False, 'truck': truck, 'type': type, 'lat_i': lat_i, 
                                            'lon_i': lon_i, 'lat_j': lat_j, 'lon_j': lon_j, 'lat_k': lat_k,
                                            'lon_k': lon_k, 'wpname_k': wpname_k, 'alt': alt, 'spd': spd}
        return available_name

    def get_available_name(self):
        nr = sum('SP' in x for x in bs.traf.id)
        return 'SP'+str(nr + 1)

    def spawn_drone(self, drone_name):
        data = self.active_drones[drone_name]
        hdg, _ = qdrdist(float(data['lat_i']), float(data['lon_i']), float(data['lat_j']), float(data['lon_j']))
        stack.stack(f"CRE {drone_name} {data['type']} {data['lat_i']} {data['lon_i']} {hdg} 8 0")
        self.active_drones[drone_name]['status'] = 'OTW'

    def route_drone(self, drone_name):
        data = self.active_drones[drone_name]
        stack.stack(f"ALT {drone_name} {data['alt']}")
        stack.stack(f"{drone_name} ATALT {data['alt']} SPDAP {drone_name} {data['spd']}")
        scen_text = f"ADDTDWAYPOINTS {drone_name} "
        for wp in ['i', 'j', 'k']:
            scen_text += f"{data['lat_' + wp]} {data['lon_' + wp]} {data['alt']} {data['spd']} TURNSPD 5 "

        stack.stack(scen_text)
        stack.stack(f"LNAV {drone_name} ON")
        stack.stack(f"VNAV {drone_name} ON")
        stack.stack(f"{drone_name} ATSPD 1 ADDOPERATIONPOINTS {drone_name} SP1002 DELIVERY 5")
        stack.stack(f"{drone_name} ATSPD 1 ADDOPERATIONPOINTS {drone_name} SP1003 RENDEZVOUS 1")
        stack.stack(f"{drone_name} ATSPD 1 ADDOPERATIONPOINTS {data['truck']} {data['wpname_k']} RENDEZVOUS 1")

    def complete_sortie(self, drone_name):
        if drone_name in self.active_drones:
            self.active_drones[drone_name]['status'] = 'HOVERING'

    def check_rendezvous(self, drone_name):
        return True if (drone_name in self.active_drones and self.active_drones[drone_name]['status'] == 'HOVERING') \
            else False
    
    def retrieve_drone(self, drone_name):
        stack.stack(f"DEL {drone_name}")