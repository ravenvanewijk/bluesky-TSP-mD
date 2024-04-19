"""Plugin to enable modified delivery waypoints."""
import bluesky as bs
import numpy as np
import random

from bluesky.tools.geo import kwikdist_matrix, qdrdist
from bluesky.tools.misc import txt2tim
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.traffic import Route
from bluesky.plugins.TDCDP.TDdronemanager import DroneManager
 
def init_plugin():
 
    # Addtional initilisation code
 
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'OPERATIONS',
 
        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }
    bs.traf.operations = Operations()

    return config
 
class Operations(Entity):
    def __init__(self):
        super().__init__()
        self.operational_states = {}
        self.operation_durations = {}
        self.cruise_speeds = {}
        self.cruise_alts = {}
        self.drone_manager = DroneManager()
        self.operation_types = {
            'DELIVERY': 'Delivery',
            'SPAWN': 'Sortie',
            'RETRIEVE': 'Rendezvous'
            }
    
    @stack.command
    def addoperationpoints(self, acidx: 'acid', *args):
        # Args come in this order: lat, lon, alt, spd, TURNSPD/TURNRAD/FLYBY, turnspeed or turnrad value, WPT type and 
        # operational time duration.
        # (extended) WPT type can be DELIVER only for now. Will implement package delivery logic
        # If turn is '0', then ignore turnspeed

        if len(args)%8 !=0:
            bs.scr.echo('You missed a waypoint value, argument number must be a multiple of 8.')
            return

        acid = bs.traf.id[acidx]
        acrte = Route._routes[acid]
 
        # Extract additional parsed argument here. Used on top of regular addwaypoints function
        args = np.reshape(args, (int(len(args)/8), 8))

        for wp in args:
            if wp[6] == 'DELIVERY':
                if wp[4] != 'TURNSPD':
                    bs.scr.echo('TURNSPD waypoint required for delivery. \
                                Turnspeed for waypoint(s) marked as delivery automatically adjusted to 0.')
                # Modify TURNSPD to 5 for all delivery waypoints
                wp[5] = '5'
                wp[4] = 'TURNSPD'
        
        # Check if attribute already exists. If not: create it (first cmds); if exists: append it (later cmds).
        if hasattr(acrte, 'operation_wp') and 'acid' in self.operation_durations \
            and 'acid' in self.cruise_speeds and 'acid' in self.cruise_alts:
            acrte.operation_wp = np.append(acrte.operation_wp, args[:,6])
            self.operation_durations.append(args[:,7])
            self.cruise_alts.append(args[:,2])
            self.cruise_speeds.append(args[:,3])
        else:        
            acrte.operation_wp = args[:,6]
            self.operation_durations[acid] = args[:,7]
            self.cruise_alts[acid] = args[:,2]
            self.cruise_speeds[acid] = args[:,3]

        # Extract regular waypoint arguments. These are the first 6 of the arguments (and multiple)
        reg_args = args[:,:6]
        
        # Call regular addwaypoints function
        Route.addwaypoints(acidx, *reg_args.flatten())

    @timed_function(dt = bs.sim.simdt)
    def check_operation(self):
        for acid in bs.traf.id:
            acidx = bs.traf.id.index(acid)
            acrte = Route._routes[acid]
            iactwp = acrte.iactwp
            # check whether or not the attribute exists. Will not exist if regular addwaypoints is called
            if hasattr(acrte, 'operation_wp'):
                _, actdist = qdrdist(bs.traf.lat[acidx], bs.traf.lon[acidx], acrte.wplat[iactwp], acrte.wplon[iactwp])
                # when distance is neglible, set SPD to 0 manually and start delivery process
                if acrte.operation_wp[iactwp] == 'DELIVERY' and actdist < 0.0025:
                    self.commence_operation(acrte, acidx, acid, iactwp)
                    
                # check whether an operation is active, or start timer when vehicle is stationary at ALT and SPD 0
                if acid in self.operational_states:
                    # AC is marked to make a delivery. Start the timer for the first time the altitude is 0
                    # At altitude 0 the delivery starts
                    if bs.traf.alt[acidx] == 0 and bs.traf.cas[acidx] == 0 \
                         and 'operation_start' not in self.operational_states[acid]:
                        self.operational_states[acid]['operation_start'] = bs.sim.simt

                    # check whether delivery has started
                    elif 'operation_start' in self.operational_states[acid]:
                        # When delivery has occured (after the waiting time), fly back to cruise altitude and speed
                        if bs.sim.simt - self.operational_states[acid]['operation_start'] >= \
                                        int(self.operation_durations[acid][self.operational_states[acid]['wp_index']]):
                            if self.operational_states[acid]['op_type'] == 'Delivery':
                                pass
                            elif self.operational_states[acid]['op_type'] == 'Sortie':
                                pass
                            elif self.operational_states[acid]['op_type'] == 'Rendezvous':
                                pass

                            self.complete_operation(acidx, acid)


# and self.operational_states[acid]['op_type'] == 'Delivery'

    def commence_operation(self, acrte, acidx, acid, iactwp):
        # AC has a delivery WP. When arrived at WP, decline to ALT and deliver package
        bs.traf.selspd[acidx] = 0
        bs.traf.swvnavspd[acidx]   = False
        stack.stack(f'{acid} ALT 0')
        op_type = self.operation_types.get(acrte.operation_wp[iactwp].upper())
        # once the sequence of commands starts, remove the delivery tag
        acrte.operation_wp[iactwp] = 'DELIVERING'
        # log delivery start to delivery state dict. This will trigger the delivery process.
        self.operational_states[acid] = {'start_time': bs.sim.simt, 'wp_index': iactwp, 'op_type': op_type}

    def complete_operation(self, acidx, acid):
        wp_index = self.operational_states[acid]['wp_index']
        # only if the vehicle is not a truck, continue with ascend. Otherwise, directly accelerate
        # After the waiting time, ascend to cruise altitude again
        if bs.traf.type[acidx] != 'TRUCK':
            stack.stack(f'{acid} ALT {self.cruise_alts[acid][wp_index]}')
            stack.stack(f'{acid} ATALT {self.cruise_alts[acid][wp_index]} \
                SPDAP {acid} {self.cruise_speeds[acid][wp_index]}')
        else:
            stack.stack(f'SPDAP {acid} {self.cruise_speeds[acid][wp_index]}')
        # QUESTION::: Why can this only be initiated at SPD lower than 5 ?
        stack.stack(f'ATSPD {acid} 5 LNAV {acid} ON')
        stack.stack(f'ATSPD {acid} 5 VNAV {acid} ON')
        # delivery is done, remove from the delivery states dict
        self.operational_states.pop(acid, None)