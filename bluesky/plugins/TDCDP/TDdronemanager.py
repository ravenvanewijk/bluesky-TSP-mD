 
from bluesky.core import Entity, timed_function


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

