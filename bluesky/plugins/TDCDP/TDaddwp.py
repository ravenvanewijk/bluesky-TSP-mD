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

        for wp in args:
            if int(wp[5]) != 0 and wp[6] == 'DELIVERY':
                bs.scr.echo('Cannot deliver a package with a turnspeed. \
                            Turnspeed for waypoint(s) marked as delivery automatically adjusted to 0.')
                # Modify TURNSPD to 0 for all delivery waypoints
                wp[5] = '0'
        
        # Check if attribute already exists. If not: create it (first cmds); iff exists: append it (later cmds).
        if hasattr(acrte, 'delivery_wp'):
            acrte.delivery_wp = np.append(acrte.delivery_wp, args[:,6])
        else:        
            acrte.delivery_wp = args[:,6]
        
        # Extract regular waypoint arguments. These are the first 6 of the arguments (and multiple)
        reg_args = args[:,:6]
        
        # Call regular addwaypoints function
        Route.addwaypoints(acidx, *reg_args.flatten())

    @timed_function(dt = bs.sim.simdt)
    def check_delivery(self):
        delivery_duration = 5
        for acid in bs.traf.id:
            acrte = Route._routes[acid]
            iactwp = acrte.iactwp
            # check whether or not the attribute exists. Will not exist if regular addwaypoints is called
            if hasattr(acrte, 'delivery_wp'):
                if acrte.delivery_wp[iactwp] == 'DELIVERY':
                    # AC has a delivery WP. When arrived at WP, decline to ALT and deliver package
                    stack.stack(f'{acid} ATDIST {acrte.wplat[iactwp]} {acrte.wplon[iactwp]} 0.0025 {acid} ATSPD 0 \
                                {acid} ALT 0')
                    # once the sequence of commands starts, remove the delivery tag
                    acrte.delivery_wp[iactwp] = 'DELIVERING'
                    
                if acrte.delivery_wp[iactwp] == 'DELIVERING':
                    # AC is out making a delivery. When it reaches the delivery altitude, wait the delivery time
                    # After the waiting time, ascend to cruise altitude again
                    if bs.traf.alt[iactwp] == 0:
                        acrte.delivery_wp[iactwp] = 'DELDONE'
                        self.t_delivery = bs.sim.simt

                elif acrte.delivery_wp[iactwp] == 'DELDONE':
                    # When delivery has occured (after the waiting time), fly back to cruise altitude and speed
                    if bs.sim.simt - self.t_delivery > delivery_duration:
                        stack.stack(f'{acid} ALT 100')
                        stack.stack(f'{acid} ATALT 100 SPDAP {acid} 25')
                        # QUESTION::: Why can this only be initiated at SPD 5 ?
                        stack.stack(f'{acid} ATALT 100 ATSPD {acid} 5 LNAV SP1 ON')
                        stack.stack(f'{acid} ATALT 100 ATSPD {acid} 5 VNAV SP1 ON')
                        acrte.delivery_wp[iactwp] = 'Nondelivery'
