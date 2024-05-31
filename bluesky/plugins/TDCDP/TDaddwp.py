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
        'plugin_type':     'sim',
        'reset': reset
    }
    bs.traf.Operations = Operations()
    return config
 
def reset():
    bs.traf.Operations.reset()

class Operations(Entity):
    def __init__(self):
        super().__init__()
        self.operational_states = {}
        self.drone_manager = DroneManager()
        self.trkdelqueue = []
    
    def reset(self):
        self.operational_states = {}
        self.drone_manager.reset()
        self.trkdelqueue = []

    @timed_function(dt = bs.sim.simdt)
    def check_operation(self):
        """This function checks whether the next waypoint is an operational waypoint
        If so, call the operation handler"""
        for acid in bs.traf.id:
            acidx = bs.traf.id.index(acid)
            acrte = Route._routes[acid]
            iactwp = acrte.iactwp
            # check whether or not the attribute exists. Will not exist if regular addwaypoints is called
            if hasattr(acrte, 'operation_wp') and iactwp > -1:
                _, actdist = qdrdist(bs.traf.lat[acidx], bs.traf.lon[acidx], acrte.wplat[iactwp], acrte.wplon[iactwp])
                # when distance is neglible, set SPD to 0 manually and start delivery process
                if acrte.operation_wp[iactwp] and actdist < 0.00025 and acid not in self.operational_states:
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
        t0 = len(op_type) * [np.inf]
        # get children at this node to add to the operational states
        children = []
        i = 0
        for op in op_type:
            if op == 'DELIVERY':
                # Deliveries do not have children, do not increment index i 
                children.append(None)
            else:
                # Sorties and rendezvouss do have children. Add the child by looking in the route and increment index
                children.append(acrte.children[iactwp][i])
                i += 1
        # log operation start to operation state dict. This will trigger the operation process.
        # This data is stored individually for each node and will be re-initialized once the simulation progresses
        self.operational_states[acid] = {
                                'wp_index': iactwp, # Current waypoint where the operation(s) are/is taking place
                                'op_type': op_type, # List of types of all operations at wp: sortie/deliv/rendezv.
                                'op_status': op_status, # List of status of all operations, True if completed
                                'children': children, # Child drones of operations that are taking place
                                't0': t0, # Start time of operation. Keeps track of all operations individually
                                'busy': False # Non vectorized variable. Keeps track if an operation is in progress
                                }

    def handle_operation(self, acrte, acidx, acid, iactwp):
        """Handle the situation accordingly by declining (optional), waiting and performing situational tasks.
        e.g. waiting for drone rendezvous at location."""
        # AC is marked to make a delivery. Start the timer for the first time the altitude and speed are 0
        # At that time, the operation timer starts
        if bs.traf.alt[acidx] == 0 and bs.traf.cas[acidx] == 0 \
                and 'ready2op' not in self.operational_states[acid]:
            self.operational_states[acid]['ready2op'] = True

        # check whether vehicle is stationary and thus ready to perform operation
        # If the vehicle is not stationary, the operation cannot be performed
        elif 'ready2op' in self.operational_states[acid]:
            for idx, operation in enumerate(self.operational_states[acid]['op_type']):
                # Iterate through the operations. If completed, mark it as done in operational status
                if self.operational_states[acid]['op_status'][idx] == True:
                    # continue with next item if the operation is already done
                    continue
                if operation == 'DELIVERY':
                    if self.operational_states[acid]['t0'][idx] == np.inf and not self.operational_states[acid]['busy']:
                        self.operational_states[acid]['t0'][idx] = bs.sim.simt
                        self.operational_states[acid]['busy'] = True
                    elif bs.sim.simt - self.operational_states[acid]['t0'][idx] >= \
                                acrte.operation_duration[self.operational_states[acid]['wp_index']]:
                        # Delivery is just 'waiting': no additional tasks
                        self.operational_states[acid]['op_status'][idx] = True
                        self.operational_states[acid]['busy'] = False
                elif operation == 'SORTIE':
                    if self.operational_states[acid]['t0'][idx] == np.inf and not \
                                self.operational_states[acid]['busy'] and \
                                self.drone_manager.drone_available(self.operational_states[acid]['children'][idx]):
                        self.operational_states[acid]['t0'][idx] = bs.sim.simt
                        self.operational_states[acid]['busy'] = True
                    elif bs.sim.simt - self.operational_states[acid]['t0'][idx] >= \
                                acrte.operation_duration[self.operational_states[acid]['wp_index']]:
                        # Sortie means an AC is to be spawned and routed from the waypoint.
                        self.drone_manager.spawn_drone(self.operational_states[acid]['children'][idx])
                        self.drone_manager.route_drone(self.operational_states[acid]['children'][idx])
                        self.operational_states[acid]['op_status'][idx] = True
                        self.operational_states[acid]['busy'] = False
                elif operation == 'RENDEZVOUS':
                    # Rendezvous entails waiting for the child drone to arrive at the location, or vice versa
                    # If the vehicle is a drone, then change its status to hovering
                    # If not, do nothing (this is managed within the drone_manager.complete_sortie())
                    self.drone_manager.complete_sortie(acid)
                    # Check whether rendezvous can be performed/ completed
                    if self.drone_manager.check_rendezvous(
                                                self.operational_states[acid]['children'][idx]):
                        if self.operational_states[acid]['t0'][idx] == np.inf  and not\
                                self.operational_states[acid]['busy']:
                            self.operational_states[acid]['t0'][idx] = bs.sim.simt
                            self.operational_states[acid]['busy'] = True
                        elif bs.sim.simt - self.operational_states[acid]['t0'][idx] >= \
                                acrte.operation_duration[self.operational_states[acid]['wp_index']]:
                            self.drone_manager.retrieve_drone(
                                                    self.operational_states[acid]['children'][idx])
                            # Set operation status to True, meaning operation has finished
                            self.operational_states[acid]['op_status'][idx] = True
                            # Remove child from operational states dict, operation is completed
                            self.operational_states.pop(self.operational_states[acid]['children'][idx],
                                                        None)
                            # Remove operation waypoint of drone
                            Route._routes[self.operational_states[acid]['children'][idx]].operation_wp[-1] = False
                            self.operational_states[acid]['busy'] = False
                        else:
                            continue

            # When all operation(s) has/ have occured, complete the operation
            if all(self.operational_states[acid]['op_status']):
                # Do not complete the operation if its the last truck wp, then it tries to continue
                if bs.traf.type[acidx] == 'TRUCK' and iactwp + 1 == len(acrte.wpname):
                    self.operational_states.pop(acid, None)
                    return
                self.complete_and_continue(acrte, acidx, acid, iactwp)

    def complete_and_continue(self, acrte, acidx, acid, iactwp):
        """This function lets the vehicle continue on its route after succesfully completing its operation(s)."""
        wp_index = self.operational_states[acid]['wp_index']
        # only if the vehicle is not a truck, continue with ascend. Otherwise, directly accelerate
        # After the waiting time, ascend to cruise altitude again
        if bs.traf.type[acidx] != 'TRUCK':
            bs.traf.swlnav[acidx] = False
            stack.stack(f'{acid} ALT {acrte.wpalt[iactwp]/ft}')
            stack.stack(f'{acid} ATALT {acrte.wpalt[iactwp]/ft} SPD {acid} {acrte.wpspd[iactwp]/kts}')
            stack.stack(f'{acid} ATSPD 2 LNAV {acid} ON')
            stack.stack(f'{acid} ATSPD 2 VNAV {acid} ON')
            bs.traf.swlnav[acidx] = True
        else:
            stack.stack(f'SPDAP {acid} 5')
            bs.traf.swlnav[acidx] = True
            bs.traf.swvnav[acidx] = True
            bs.traf.swvnavspd[acidx] = True
        # once the operation is done, remove the operation tag such that the wp can be passed
        acrte.operation_wp[wp_index] = False
        # operation is done, remove from the operational states dict
        self.operational_states.pop(acid, None)

    @stack.command(name='TRKDEL')
    def truckdelete(self, idx):
        """Delete a truck after it has completed its route and operations"""
        # Multiple delete not supported 
        # Get id of idx 
        id = bs.traf.id.index(idx)
        # Call the actual delete function, iif all operations have been succesfully completed
        if len(self.operational_states) == 0:
            bs.traf.delete(id)
            # Update number of aircraft
            bs.traf.ntraf = len(bs.traf.lat)
        else:
            self.trkdelqueue.append(idx)

        return True

    @timed_function(dt = bs.sim.simdt)
    def trkdelwhendone(self):
        """Checks which trucks are commanded to be deleted, and checks whether this deletion can be performed.
        If so, the truck is deleted"""
        for truck in self.trkdelqueue:
            if len(self.operational_states) == 0:
                id = bs.traf.id.index(truck)
                self.truckdelete(truck)
                self.trkdelqueue.remove(truck)
        return True
