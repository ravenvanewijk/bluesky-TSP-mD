"""Plugin to enable modified operation waypoints."""
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
from bluesky.plugins.TDCDP.data_logger import DataLogger

delivery_dist = 0.00025 * 4

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
        self.data_logger = DataLogger()
        self.custs_served = 0
        self.custs_served_list = []
        self.trk_op_time = 0
    
    def reset(self):
        self.operational_states = {}
        self.drone_manager.reset()
        self.trkdelqueue = []
        self.data_logger.shutdown()
        self.custs_served = 0
        self.custs_served_list = []
        self.trk_op_time = 0
    
    @stack.command(name='LOG')
    def config_log(self, new_file_name=None, new_log_dir=None):
        self.data_logger.change_log_file(new_file_name, new_log_dir)

    @timed_function(dt = bs.sim.simdt)
    def check_operation(self):
        """This function checks whether the next waypoint is an operational waypoint
        If so, call the operation handler"""
        for acid in bs.traf.id:
            acidx = bs.traf.id.index(acid)
            acrte = Route._routes[acid]
            iactwp = acrte.iactwp
            if bs.sim.simt <= bs.sim.simdt and len(acrte.wplat) > 0:
                iactwp = 0
            # check whether or not the attribute exists. Will not exist if regular addwaypoints is called
            if hasattr(acrte, 'operation_wp') and iactwp > -1:
                prevwp = max(0, iactwp - 1)
                prev2wp = max(0, iactwp - 2)
                _, actdist = qdrdist(bs.traf.lat[acidx], bs.traf.lon[acidx], 
                                    acrte.wplat[iactwp], acrte.wplon[iactwp])
                _, prevdist = qdrdist(bs.traf.lat[acidx], bs.traf.lon[acidx], 
                                    acrte.wplat[prevwp], acrte.wplon[prevwp])
                _, prev2dist = qdrdist(bs.traf.lat[acidx], bs.traf.lon[acidx], 
                                    acrte.wplat[prev2wp], acrte.wplon[prev2wp])
                actcond = acrte.operation_wp[iactwp] and \
                                actdist < delivery_dist
                prevcond = acrte.operation_wp[prevwp] and \
                                prevdist < delivery_dist
                prev2cond = acrte.operation_wp[prev2wp] and \
                                prev2dist < delivery_dist
                # when distance is neglible, set SPD to 0 manually and start operation process
                # Three checks:
                #   1. actcond: checks distance for active wp
                #   2. prevcond: checks distance for previous wp
                #   3. prev2cond: very rarely skips 2 wps without checking this
                #       wp. Happens if multiple operations are close
                if (prevcond or actcond or prev2cond)\
                        and acid not in self.operational_states:
                    if prevcond:
                        iactwp = max(0, iactwp - 1)
                    elif prev2cond:
                        iactwp = max(0, iactwp - 2) 
                    self.commence_operation(acrte, acidx, acid, iactwp)
                    
                # check whether an operation is active, or start timer when vehicle is stationary at ALT and SPD 0
                if acid in self.operational_states:
                    self.handle_operation(acrte, acidx, acid)

    def commence_operation(self, acrte, acidx, acid, iactwp):
        """This function let's the vehicle come to a complete stop at alt 0, and triggers the operation handler once
        the stationary conditions are met."""
        # AC has a operation WP. When arrived at WP, decline to ALT and deliver package
        bs.traf.selspd[acidx] = 0
        bs.traf.swvnavspd[acidx] = False
        stack.stack(f'{acid} ALT 0')
        op_type = acrte.op_type[iactwp]
        cust = acrte.custid[iactwp]
        # Track operational status, useful to discover if all operations have finished
        op_status = acrte.op_status[iactwp]
        t0 = acrte.op_t0[iactwp]
        op_duration = acrte.operation_duration[iactwp]
        # get children at this node to add to the operational states
        children = acrte.children[iactwp]
        # log operation start to operation state dict. This will trigger the operation process.
        # This data is stored individually for each node and will be re-initialized once the simulation progresses
        self.operational_states[acid] = {
                                'wp_index': iactwp, # Current waypoint where the operation(s) are/is taking place
                                'wp_coords': (acrte.wplat[iactwp], acrte.wplon[iactwp]), # Coords of the wp
                                'op_type': op_type, # List of types of all operations at wp: sortie/deliv/rendezv.
                                'op_status': op_status, # List of status of all operations, True if completed
                                'op_duration': op_duration, # List of operation durations
                                'children': children, # Child drones of operations that are taking place
                                't0': t0, # Start time of operation. Keeps track of all operations individually
                                't_a': np.inf, # Arrival time at the operation point, ready to operate
                                'busy': False, # Non vectorized variable. Keeps track if an operation is in progress
                                'custid': cust
                                }
        # Special (rare) case:
        # When Rendezvous wp == Sortie wp, sortie is completed before adding
        # rendezvous. Operational states dictionary of that sortie will be 
        # deleted before the rendezvous is added
        # Op status of the sortie should be manually set to True
        for idx in range(len(self.operational_states[acid]['op_status'])):
            cond1 = self.operational_states[acid]['op_type'][idx] == 'SORTIE'
            cond2 = self.operational_states[acid]['children'][idx] in \
                        self.drone_manager.active_drones
            if cond1 and cond2:
                child = self.operational_states[acid]['children'][idx]
                cond3 = self.drone_manager.active_drones[child]['status'] == 'OTW'
                if cond3:
                    self.operational_states[acid]['op_status'][idx] = True


    def handle_operation(self, acrte, acidx, acid):
        """Handle the situation accordingly by descending (optional), waiting and performing situational tasks.
        e.g. waiting for drone rendezvous at location."""
        # AC is marked to make a delivery. Start the timer for the first time the altitude and speed are 0
        # At that time, the operation timer starts
        iactwp = self.operational_states[acid]['wp_index']
        if bs.traf.alt[acidx] == 0 and bs.traf.cas[acidx] == 0 \
                and 'ready2op' not in self.operational_states[acid]:
            self.operational_states[acid]['ready2op'] = True
            self.operational_states[acid]['t_a'] = bs.sim.simt

            # Convert to numpy arrays for easy modification
            wplat_array = np.array(acrte.wplat)
            wplon_array = np.array(acrte.wplon)

            # Find the matching indices using boolean indexing
            matching_mask = (wplat_array == acrte.wplat[iactwp]) & (wplon_array 
                                                        == acrte.wplon[iactwp])
            # Update location of wp such that accuracy==100% at wp
            # Do this for all matching indices
            wplat_array[matching_mask] = bs.traf.lat[acidx]
            wplon_array[matching_mask] = bs.traf.lon[acidx]

            # convert back to lists
            acrte.wplat = wplat_array.tolist()
            acrte.wplon = wplon_array.tolist()

        # check whether vehicle is stationary and thus ready to perform operation
        # If the vehicle is not stationary, the operation cannot be performed
        elif 'ready2op' in self.operational_states[acid]:
            for idx, operation in enumerate(self.operational_states[acid]['op_type']):
                # Iterate through the operations. If completed, mark it as done in operational status
                if self.operational_states[acid]['op_status'][idx] == True:
                    # continue with next item if the operation is already done
                    continue
                # Whether or not the timer can be started
                # Only when not started yet and should not be busy with another operation
                start_timer_check =  self.operational_states[acid]['t0'][idx] == np.inf \
                                        and not self.operational_states[acid]['busy']
                # Whether the operation timer has been completed or still in progress
                op_timer_check = bs.sim.simt - self.operational_states[acid]['t0'][idx] >= \
                                self.operational_states[acid]['op_duration'][idx]

                if operation == 'DELIVERY':
                    if start_timer_check:
                        self.start_timer(acid, idx)
                    elif op_timer_check:
                        # Delivery is just 'waiting': no additional tasks
                        self.mark_done(acid, idx)
                        self.data_logger.log_customer_service(
                                                self.operational_states[acid],
                                                acid,
                                                idx,
                                                self.custs_served + 1,
                                                bs.sim.simt - self.operational_states[acid]['t0'][idx],
                                                self.operational_states[acid]['custid']
                                                            )
                        # Increment customers served by 1
                        self.custs_served += 1
                        self.custs_served_list.append(self.operational_states[acid]['custid'])
                        try:
                            self.drone_manager.active_drones[acid]['del_done'] = True
                        except:
                            pass
                
                elif operation == 'STOP':
                    if start_timer_check:
                        self.start_timer(acid, idx)
                    elif op_timer_check:
                        # Stop is just 'waiting': no additional tasks
                        self.mark_done(acid, idx)

                elif operation == 'SORTIE':
                    drone_avail = self.drone_manager.drone_available(
                                    self.operational_states[acid]['children'][idx]
                                                                    )           
                    if start_timer_check and drone_avail:
                        self.start_timer(acid, idx)
                    elif op_timer_check:
                        child = self.operational_states[acid]['children'][idx]
                        self.data_logger.log_droneop('SORTIE', child, bs.sim.simt)
                        self.perform_sortie(acid, idx)

                elif operation == 'RENDEZVOUS':
                    # Rendezvous entails waiting for the child drone to arrive at the location, or vice versa
                    # If the vehicle is a drone, then change its status to hovering
                    # If not, do nothing (this is managed within the drone_manager.complete_sortie())
                    self.drone_manager.complete_sortie(acid)
                    # Check whether rendezvous can be performed/ completed
                    if self.drone_manager.check_rendezvous(
                                                self.operational_states[acid]['children'][idx]):
                        if start_timer_check:
                            self.start_timer(acid, idx)
                        elif op_timer_check:
                            child = self.operational_states[acid]['children'][idx]
                            entity = 'UAV' if self.operational_states[child]['t_a'] < \
                                self.operational_states[acid]['t_a'] + \
                                sum(self.operational_states[acid]['op_duration'][:idx]) \
                                    else 'TRUCK'
                            waiting_time = abs(self.operational_states[child]['t_a']-
                                                (self.operational_states[acid]['t_a']+ \
                                sum(self.operational_states[acid]['op_duration'][:idx]))) 
                            childid = bs.traf.id.index(child)
                            self.data_logger.log_droneop('RENDEZVOUS', child, bs.sim.simt,
                                                entity, waiting_time, bs.traf.distflown[childid])
                            self.perform_rendezvous(acid, idx)
                        else:
                            continue

            # When all operation(s) has/ have occured, complete the operation
            if all(self.operational_states[acid]['op_status']):
                # Do not complete the operation if its the last truck wp, then it tries to continue
                if bs.traf.type[acidx].upper() == 'TRUCK' and iactwp + 1 == len(acrte.wpname):
                    # pop from operational states, operation is completed
                    self.operational_states.pop(acid, None)
                    return
                self.continue_en_route(acrte, acidx, acid, iactwp)

    def start_timer(self, acid, idx):

        # Start timer and set truck as 'busy'
        self.operational_states[acid]['t0'][idx] = bs.sim.simt
        self.operational_states[acid]['busy'] = True

    def mark_done(self, acid, idx):
        self.operational_states[acid]['op_status'][idx] = True
        self.operational_states[acid]['busy'] = False
        if acid == 'TRUCK':
            self.trk_op_time += self.operational_states[acid]['op_duration'][idx] 
            if idx == 0:
                # for first operation, add waiting time for the drone as well
                self.operational_states[acid]['t_a'] - self.operational_states[acid]['t0'][idx]

    def perform_sortie(self, acid, idx):
        # Sortie means an AC is to be spawned and routed from the waypoint.
        self.drone_manager.spawn_drone(self.operational_states[acid]['children'][idx])
        self.drone_manager.route_drone(self.operational_states[acid]['children'][idx])
        self.drone_manager.active_drones[self.operational_states[acid]['children'][idx]] \
                                        ['routed_ts'] = bs.sim.simt
        self.mark_done(acid, idx)

    def perform_rendezvous(self, acid, idx):
        self.drone_manager.retrieve_drone(
                        self.operational_states[acid]['children'][idx])
        self.mark_done(acid, idx)
        # Remove child from operational states dict, operation is completed
        child = self.operational_states[acid]['children'][idx]
        self.operational_states.pop(child, None)
        # Remove operation waypoint of drone
        Route._routes[self.operational_states[acid]['children'][idx]].operation_wp[-1] = False

    def continue_en_route(self, acrte, acidx, acid, iactwp):
        """This function lets the vehicle continue on its route after succesfully completing its operation(s)."""
        wp_index = self.operational_states[acid]['wp_index']
        # only if the vehicle is not a truck, continue with ascend. Otherwise, directly accelerate
        # After the waiting time, ascend to cruise altitude again
        if bs.traf.type[acidx].upper() != 'TRUCK':
            bs.traf.swlnav[acidx] = False
            stack.stack(f'{acid} ALT {acrte.wpalt[iactwp]/ft}')
            stack.stack(f'{acid} ATALT {acrte.wpalt[iactwp]/ft} SPD {acid} {acrte.wpspd[iactwp]/kts}')
            stack.stack(f'{acid} ATSPD 2 LNAV {acid} ON')
            stack.stack(f'{acid} ATSPD 2 VNAV {acid} ON')
            bs.traf.swlnav[acidx] = True
        else:
            # stack.stack(f'SPDAP {acid} 5')
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
        if len(self.operational_states) == 0 and\
            len(self.drone_manager.active_drones) == 0 and not\
            bs.traf.swlnav[id]:
            bs.traf.delete(id)
            # Update number of aircraft
            bs.traf.ntraf = len(bs.traf.lat)
            self.data_logger.log_data({'type': 'Completion', 'completion_time: ': bs.sim.simt})
            self.data_logger.shutdown()
            stack.stack('HOLD')
        else:
            self.trkdelqueue.append(idx)
        return True

    @timed_function(dt = bs.sim.simdt)
    def trkdelwhendone(self):
        """Checks which trucks are commanded to be deleted, and checks whether this deletion can be performed.
        If so, the truck is deleted"""
        for truck in self.trkdelqueue:
            id = bs.traf.id.index(truck)
            if len(self.operational_states) == 0 and not bs.traf.swlnav[id]:
                self.truckdelete(truck)
                self.trkdelqueue.remove(truck)
        return True

    def recondelete(self, idx):
        """Delete a recon truck after it has scouted its route and operations"""
        id = bs.traf.id.index(idx)
        bs.traf.delete(id)
        # Update number of aircraft
        bs.traf.ntraf = len(bs.traf.lat)

        # Create a list of keys to delete where 'truck' matches the given value
        drones_to_del = [key for key, drone in self.drone_manager.active_drones.items() if drone['truck'] == idx]

        # Remove those keys from the dictionary
        for key in drones_to_del:
            del self.drone_manager.active_drones[key]