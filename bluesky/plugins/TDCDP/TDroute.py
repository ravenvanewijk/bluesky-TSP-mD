from bluesky.traffic.route import Route

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'TDRoute',
        'plugin_type':     'sim'
    }
    return config

class TDRoute(Route):
    def __init__(self, acid):
        super().__init__(acid)
        