"""Plugin to enable modified delivery waypoints."""
import bluesky as bs
import numpy as np
import random
from math import isclose
from bluesky.tools.geo import kwikdist_matrix, qdrdist
from bluesky.tools.misc import txt2tim
from bluesky.tools.aero import kts, ft
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.traffic import Route
from bluesky.plugins.TDCDP.TDdronemanager import DroneManager, get_wpname
 
def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'OPERATIONS',
        'plugin_type':     'sim'
    }
    bs.traf.operations = Operations()
    return config
 
class Operations(Entity):
    def __init__(self):
        super().__init__()
        self.operational_states = {}
        self.drone_manager = DroneManager()
    
    @stack.command
    def addtdwaypoints(self, acidx: 'acid', *args):
        # Args come in this order: lat, lon, alt, spd, TURNSPD/TURNRAD/FLYBY, turnspeed or turnrad value
        # (extended) add operation arg
        Route.addwaypoints(acidx, *args)

        wpcount = len(args)//6
        acid = bs.traf.id[acidx]
        acrte = Route._routes[acid]
        if hasattr(acrte, 'operation_wp'):
            acrte.operation_wp.extend(wpcount * [False])
        else:
            acrte.operation_wp = wpcount * [False]

        if hasattr(acrte, 'operation_duration'):
            acrte.operation_duration.extend(wpcount * [None])
        else:
            acrte.operation_duration = wpcount * [None]

        if hasattr(acrte, 'children'):
            acrte.children.extend(wpcount * [[None]])
        else:
            acrte.children = wpcount * [[None]]

        if hasattr(acrte, 'op_type'):
            acrte.op_type.extend(wpcount * [None])
        else:
            acrte.op_type = wpcount * [None]
        

    @stack.command
    def addoperationpoints(self, acidx: 'acid', wpname: 'wpname/coords', wptype: 'wptype', duration: 'duration', *args):
        # Args are only valid for SORTIE type modification and come in this order: 
        # type, lat_j, lon_j, wpname_k, alt, spd
        wptype = wptype.upper()
        wpname = wpname.upper()
        if len(args) !=0 and wptype == 'DELIVERY':
            bs.scr.echo('Invalid number of operational values, argument number must be 0 for delivery\
                type modification.')
            return

        if (len(args) !=1 and len(args) !=0) and wptype.upper() == 'RENDEZVOUS':
            bs.scr.echo('Invalid number of operational values, argument number must be 0 or 1 for rendezvous \
                type modification. If 1: child_name')
            return

        elif len(args) !=6 and wptype == 'SORTIE':
            bs.scr.echo('Invalid number of operational values, argument number must be 6 for sortie type modification.\
                        Order: type, lat_j, lon_j, wpname_k, alt, spd')
            return
        
        elif wptype not in ['DELIVERY', 'SORTIE', 'RENDEZVOUS']:
            bs.scr.echo('Invalid operation type, please select from: DELIVERY, SORTIE OR RENDEZVOUS')

        acid = bs.traf.id[acidx]
        acrte = Route._routes[acid]

        if len(wpname.split('/')) == 2:
            # Check whether coordinates are given. If so, look up wpname
            wpname = get_wpname(wpname, acrte)

        wpid = acrte.wpname.index(wpname)
        if acrte.wpflyby[wpid]:
            bs.scr.echo('TURNSPD waypoint required for operation waypoints. \
                        Turnspeed for waypoint(s) marked as delivery automatically adjusted to default value.')
        # this line for some reason stop the drone from spawning
        # Modify TURNSPD to 5 for all delivery waypoints
        # acrte.wpturnspd[wpid] = 5.0
        acrte.wpflyby[wpid] = False
        acrte.wpflyturn[wpid] = True

        # modify operational attributes of the wp
        acrte.operation_wp[wpid] = True
        acrte.operation_duration[wpid] = float(duration)
        if acrte.op_type[wpid] is None:
            acrte.op_type[wpid] = [wptype]
        else:
            acrte.op_type[wpid].extend([wptype])

        if wptype.upper() == 'SORTIE':
            if len(args[3].split('/')) == 2:
                # Check whether coordinates are given. If so, look up wpname. If not, None is returned
                wpname_k = get_wpname(args[3], acrte)
                lat_k = args[3].split('/')[0]
                lon_k = args[3].split('/')[1]
            else:
                wpname_k = args[3]
            try:
                wpid_k = acrte.wpname.index(wpname_k.upper())
                # truck, type, lat_i, lon_i, lat_j, lon_j, lat_k, lon_k, wpname_k, alt, spd
                child = self.drone_manager.add_drone(acid, args[0], acrte.wplat[wpid], acrte.wplon[wpid], 
                    args[1], args[2], acrte.wplat[wpid_k], acrte.wplon[wpid_k], wpname_k, args[4], args[5])
            except AttributeError:
                # This except block is used when wpname_k is None. The coordinates will be stored with the /
                # This means the operation point cannot be added yet since it is not a part of the truck route yet
                # Dronemanager class will recognize this and delay adding the operational point
                wpid_k = wpname_k
                child = self.drone_manager.add_drone(acid, args[0], acrte.wplat[wpid], acrte.wplon[wpid], 
                    args[1], args[2], None, None, args[3], args[4], args[5])

            if acrte.children[wpid][0] is None:
                acrte.children[wpid] = [child]
            else:
                acrte.children[wpid].extend([child])
        
        if wptype.upper() == 'RENDEZVOUS':
            if len(wpname.split('/')) == 2:
                # Check whether coordinates are given. If so, look up wpname
                wpid_k = acrte.wpname.index(wpname)
            else:
                wpid_k = acrte.wpname.index(wpname.upper())
            if args:
                if acrte.children[wpid_k][0] is None:
                    acrte.children[wpid_k] = [args[0]]
                else:
                    acrte.children[wpid_k].extend([args[0]])

    @timed_function(dt = bs.sim.simdt)
    def check_operation(self):
        """This function checks whether the next waypoint is an operational waypoint
        If so, call the operation handler"""
        for acid in bs.traf.id:
            acidx = bs.traf.id.index(acid)
            acrte = Route._routes[acid]
            iactwp = acrte.iactwp
            # if len(acrte.wpname) > iactwp+2:
            #     if acrte.wpname[iactwp] == 'TRUCK469' or acrte.wpname[iactwp] == 'TRUCK468' or acrte.wpname[iactwp] == 'TRUCK470' or acrte.wpname[iactwp] == 'TRUCK471' or acrte.wpname[iactwp] == 'TRUCK467':
            #         print(iactwp, acrte.wpname[iactwp])
            if iactwp == -1:
                # Manually set active wp to 0 if vehicle is at start, ensure that WP1 can also be an operation
                iactwp = 0
            # check whether or not the attribute exists. Will not exist if regular addwaypoints is called
            if hasattr(acrte, 'operation_wp'):
                _, actdist = qdrdist(bs.traf.lat[acidx], bs.traf.lon[acidx], acrte.wplat[iactwp], acrte.wplon[iactwp])
                # when distance is neglible, set SPD to 0 manually and start delivery process
                if acrte.operation_wp[iactwp] and actdist < 0.0025:
                    self.commence_operation(acrte, acidx, acid, iactwp)
                    
                # check whether an operation is active, or start timer when vehicle is stationary at ALT and SPD 0
                if acid in self.operational_states:
                    self.handle_operation(acrte, acidx, acid, iactwp)

    def commence_operation(self, acrte, acidx, acid, iactwp):
        """This function let's the vehicle come to a complete stop at alt 0, and triggers the operation handler once
        the stationary conditions are met."""
        # AC has a operation WP. When arrived at WP, decline to ALT and deliver package
        bs.traf.selspd[acidx] = 0
        bs.traf.swvnavspd[acidx] = False
        stack.stack(f'{acid} ALT 0')
        op_type = acrte.op_type[iactwp]
        # Track operational status, useful to discover if all operations have finished
        op_status = len(op_type) * [False]
        # once the sequence of commands starts, remove the operation tag such that this function is not called again
        acrte.operation_wp[iactwp] = False
        # log operation start to operation state dict. This will trigger the operation process.
        self.operational_states[acid] = {'start_time': bs.sim.simt, 'wp_index': iactwp, 
                                        'op_type': op_type, 'op_status': op_status}

    def handle_operation(self, acrte, acidx, acid, iactwp):
        """Handle the situation accordingly by declining (optional), waiting and performing situational tasks.
        e.g. waiting for drone rendezvous at location."""
        # AC is marked to make a delivery. Start the timer for the first time the altitude and speed are 0
        # At that time, the operation timer starts
        if bs.traf.alt[acidx] == 0 and bs.traf.cas[acidx] == 0 \
                and 'operation_start' not in self.operational_states[acid]:
            self.operational_states[acid]['operation_start'] = bs.sim.simt

        # check whether delivery has started
        elif 'operation_start' in self.operational_states[acid]:
            # speed and alt are 0, check if 'waiting' time is over
            if bs.sim.simt - self.operational_states[acid]['operation_start'] >= \
                            acrte.operation_duration[self.operational_states[acid]['wp_index']]:
                for idx, operation in enumerate(self.operational_states[acid]['op_type']):
                    # Iterate through the operations. If completed, mark it as done in operational status
                    if self.operational_states[acid]['op_status'][idx] == True:
                        # continue with next item if the operation is already done
                        continue
                    if operation == 'DELIVERY':
                        # Delivery is just 'waiting': no additional tasks
                        self.operational_states[acid]['op_status'][idx] = True
                    elif operation == 'SORTIE':
                        # Sortie means an AC is to be spawned and routed from the waypoint.
                        self.drone_manager.spawn_drone(acrte.children[self.operational_states[acid]['wp_index']][idx])
                        self.drone_manager.route_drone(acrte.children[self.operational_states[acid]['wp_index']][idx])
                        self.operational_states[acid]['op_status'][idx] = True
                    elif operation == 'RENDEZVOUS':
                        # Rendezvous entails waiting for the child drone to arrive at the location, or vice versa
                        self.drone_manager.complete_sortie(acid)
                        if self.drone_manager.check_rendezvous(
                                                    acrte.children[self.operational_states[acid]['wp_index']][idx]):
                            self.drone_manager.retrieve_drone(
                                                    acrte.children[self.operational_states[acid]['wp_index']][idx])
                            self.operational_states[acid]['op_status'][idx] = True
                        else:
                            continue

                # When all operation(s) has/ have occured, complete the operation
                if all(self.operational_states[acid]['op_status']):
                    self.complete_operation(acrte, acidx, acid, iactwp)

    def complete_operation(self, acrte, acidx, acid, iactwp):
        """This function lets the vehicle continue on its route after succesfully completing its operation(s)."""
        wp_index = self.operational_states[acid]['wp_index']
        # only if the vehicle is not a truck, continue with ascend. Otherwise, directly accelerate
        # After the waiting time, ascend to cruise altitude again
        if bs.traf.type[acidx] != 'TRUCK':
            stack.stack(f'{acid} ALT {acrte.wpalt[iactwp]/ft}')
            stack.stack(f'{acid} ATALT {acrte.wpalt[iactwp]/ft} \
                SPDAP {acid} {acrte.wpspd[iactwp]/kts}')
        else:
            stack.stack(f'SPDAP {acid} {acrte.wpspd[iactwp]/kts}')
        # QUESTION::: Why can this only be initiated at SPD lower than 5 ?
        stack.stack(f'ATSPD {acid} 5 LNAV {acid} ON')
        stack.stack(f'ATSPD {acid} 5 VNAV {acid} ON')
        # delivery is done, remove from the delivery states dict
        self.operational_states.pop(acid, None)
