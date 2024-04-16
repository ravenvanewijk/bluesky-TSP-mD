"""Plugin to enable modified delivery waypoints."""
import bluesky as bs
import numpy as np
import random

from bluesky.tools.geo import kwikdist_matrix
from bluesky.tools.misc import txt2tim
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.traffic import Route
 
def init_plugin():
 
    # Addtional initilisation code
 
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'DELIVER',
 
        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }
    bs.traf.delivery = Delivery()

    return config
 
class Delivery(Entity):
    def __init__(self):
        super().__init__()
        
    @stack.command
    def adddeliverypoints(self, acidx: 'acid', *args):
        # Args come in this order: lat, lon, alt, spd, TURNSPD/TURNRAD/FLYBY, turnspeed or turnrad value and WPT type
        # (extended) WPT type can be DELIVER only for now. Will implement package delivery logic
        # If turn is '0', then ignore turnspeed

        if len(args)%7 !=0:
            bs.scr.echo('You missed a waypoint value, argument number must be a multiple of 7.')
            return

        acid = bs.traf.id[acidx]
        acrte = Route._routes[acid]
 
        # Extract additional parsed argument here. Used on top of regular addwaypoints function
        args = np.reshape(args, (int(len(args)/7), 7))
        
        acrte.delivery_wp = args[:,6]
        
        # Extract regular waypoint arguments. These are the first 6 of the arguments (and multiple)
        reg_args = args[:,:6]
        
        # Call regular addwaypoints function
        Route.addwaypoints(acidx, *reg_args.flatten())

    @timed_function(dt = bs.sim.simdt)
    def check_delivery(self):
        for acid in bs.traf.id:
            acrte = Route._routes[acid]
            iactwp = acrte.iactwp

            # check whether or not the attribute exists. Will not exist if regular addwaypoints is called
            if hasattr(acrte, 'delivery_wp'):
                if acrte.delivery_wp[iactwp] == 'DELIVERY':
                    # bs.scr.echo(f'delivery point ~ {acrte.delivery_wp[iactwp]}')
                    pass
                    # implement logic to deliver package
                    # stack.stack(f'SPD {acid} 0')
                else:
                    # do nothing
                    pass