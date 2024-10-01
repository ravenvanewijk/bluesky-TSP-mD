import bluesky as bs
import numpy as np
import json
from bluesky.traffic.autopilot import distaccel
from bluesky.tools.aero import nm, g0
from bluesky.tools.geo import kwikqdrdist

# Load json data of corresponding AC
with open('bluesky/resources/performance/OpenAP/rotor/aircraft.json') as json_data:
    raw_data = json.load(json_data)
    json_data.close()

def calc_inputs(acidx, rte, eta, op_duration, wp_indeces, custlat, custlon, alt, 
                hspd, vspd_up, vspd_down, delivery_time, truck_delivery_time,
                interp=0, iactwp=0):
    """
    Calculates various inputs required for routing, 
    including truck and drone ETAs.

    Params:
    - acidx: int, index of the aircraft
    - rte: object, route object containing waypoint and operational data
    - wp_indeces: list of int, list of waypoint indices
    - custlat: list of float, list of customer latitudes
    - custlon: list of float, list of customer longitudes
    - alt: float, cruise altitude in meters
    - hspd: float, horizontal speed of the drone in m/s
    - vspd_up: float, vertical ascent speed of the drone in m/s
    - vspd_down: float, vertical descent speed of the drone in m/s
    - delivery_time: float, time required for delivery of a drone
    - truck_delivery_time: float, time required for truck delivery

    Returns:
    - tuple, containing:
        - L: list of int, list of waypoint indices
        - P: list of int, list of waypoint indices
        - t_ij: dict, dictionary with drone travel time from i to j
        - t_jk: dict, dictionary with drone travel time including delivery 
        from j to k
        - d_j: dict, dictionary with the distance in nm from point i to
        delivery location j
        - T_i: dict, dictionary with truck ETA at waypoint i
        - T_k: dict, dictionary with truck ETA including delivery time at 
        waypoint k
        - T_ik: dict, dictionary with time difference between truck ETAs at 
        waypoints i and k
    """

    T_i = {}
    T_k = {}
    t_ij = {}
    t_jk = {}
    d_j = {}

    for i in wp_indeces:
        # truck_t = calc_truck_ETA(acidx, i)
        truck_t = calc_truck_ETA2(eta[iactwp:i + 1], op_duration[iactwp:i + 1],
                                                                    interp)
        # Add operational times of current ops
        if bs.traf.id[acidx] in bs.traf.Operations.operational_states.keys():
            op_state = bs.traf.Operations.operational_states['TRUCK']
            op_status = op_state['op_status']
            active_op = next((idx for idx, val in enumerate(op_status)\
                            if not val), None)
            for idx, op in enumerate(op_state['op_type']):
                if op_state['op_type'][idx] == 'DELIVERY':
                    duration = truck_delivery_time
                else:
                    duration = op_state['op_duration'][idx]
                if idx == active_op:
                    t0 = 0 if op_state['t0'][idx] == np.inf else op_state\
                                                                    ['t0'][idx]
                    truck_t += max(duration - (bs.sim.simt - t0), 0)
                elif idx >active_op:
                    truck_t += duration

        T_i[i] = truck_t

        T_k[i] = truck_t 

        _, dist = kwikqdrdist(rte.wplat[i], rte.wplon[i],
                                custlat, custlon)
        
        d_j[i] = dist

        t_ij[i] = calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, 3.5) + \
                                delivery_time
        # + delivery drone time to account for delivery on way back
        t_jk[i] = calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, 3.5) 

    T_ik = {}
    for i in wp_indeces:
        for k in wp_indeces:
            T_ik[(i, k)] = T_k[k] - T_i[i]
    
    P = L = wp_indeces

    return L, P, t_ij, t_jk, d_j, T_i, T_k, T_ik

def calc_truck_ETA(acidx, lastwp):
    """
    Calculates the estimated time of arrival (ETA) for the truck to reach the last waypoint.

    Params:
    - acidx: int, index of the aircraft
    - lastwp: int, index of the last waypoint

    Returns:
    - float, estimated time of arrival in seconds
    """
    # Get the aircraft information
    rte = bs.traf.ap.route[acidx]
    gs = bs.traf.gs[acidx]
    lat = bs.traf.lat[acidx]
    lon = bs.traf.lon[acidx]
    amax = bs.traf.perf.axmax[acidx]
    iactwp = rte.iactwp
    prev_turn_time = 0
    prev_turn_dist = 0
    prev_wplat, prev_wplon = lat, lon

    eta_time = 0
    tot_cruise = 0
    tot_decel = 0
    tot_turn = 0
    for wpidx in range(iactwp, lastwp + 1):
        wplat = rte.wplat[wpidx]
        wplon = rte.wplon[wpidx]
        # Add operational time of previous wp
        op_time = sum(value for value in 
                    rte.operation_duration[max(0, wpidx - 1)] \
                    if value is not None)
        eta_time += op_time
        # If the previous waypoint was a turn one, add extra time
        eta_time += prev_turn_time
        # Get its turnspeed 
        turnspd = rte.wpturnspd[wpidx]
        if rte.wpflyturn[wpidx] and iactwp != (len(rte.wplat)-1) and \
                                                    turnspd < gs:
            if wpidx + 1 >= len(rte.wplat):
                # End of route has been reached
                continue
            # Get the turn angle
            next_wplat, next_wplon = rte.wplat[wpidx+1], rte.wplon[wpidx+1]
            a1,_=bs.tools.geo.kwikqdrdist(prev_wplat,prev_wplon,wplat,wplon)
            a2,_=bs.tools.geo.kwikqdrdist(wplat,wplon,next_wplat,next_wplon)
            angle=abs(a2-a1)
            # edited this with 0 here...
            turnrate = np.degrees(g0 * np.tan(np.radians(25.)
                                                ) / turnspd)
            time_turn = angle/turnrate
            # Get the distance at which the aircraft starts decelerating
            ddecel = distaccel(gs, turnspd, amax)
            # Get the current distance to this waypoint
            totaldist = bs.tools.geo.kwikdist(prev_wplat, prev_wplon, 
                                    wplat, wplon) * nm
            cruise_dist = totaldist - ddecel - prev_turn_dist
            if cruise_dist < 0:
                # Just take it as 0
                cruise_dist = 0
                # TODO: This is innacurate but rare. Might fix later
            time_cruise = cruise_dist / rte.wpspd[acidx]
            time_decel = max((rte.wpspd[acidx] - turnspd), 0) / amax
            
            # Finally add to estimated time
            eta_time += time_cruise + time_decel + time_turn/2
            tot_cruise += time_cruise
            tot_decel += time_decel
            tot_turn += time_turn
            # Acceleration is symmetric to deceleration
            prev_turn_time = time_decel + time_turn/2
            prev_turn_dist = ddecel
            prev_wplat, prev_wplon = wplat, wplon
        else:
            # Normal cruise calculation
            totaldist = bs.tools.geo.kwikdist(prev_wplat, prev_wplon, 
                                    wplat, wplon)* nm
            cruise_dist = totaldist - prev_turn_dist
            if cruise_dist < 0:
                # Just take it as 0
                cruise_dist = 0
                # TODO: This is innacurate but rare. Might fix later
            time_cruise = cruise_dist / rte.wpspd[acidx]
            
            eta_time += time_cruise
            tot_cruise += time_cruise
            
            prev_turn_time = 0
            prev_turn_dist = 0
            prev_wplat, prev_wplon = wplat, wplon

    return eta_time

def calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, a, hspd0=0, vspd0=0,
                                                                alt0=0):
    """
    Calculates drone travel time considering acceleration.

    Params:
    - dist: float, distance in nautical miles
    - hspd: float, horizontal speed in m/s
    - vspd_up: float, vertical ascent speed in m/s
    - vspd_down: float, vertical descent speed in m/s
    - alt: float, cruise altitude in meters
    - a: float, horizontal acceleration in m/s^2
    """

    if vspd0 < 0 :
        alt_to_descend = alt0
        alt_to_climb = 0
    elif vspd0 > 0:
        alt_to_climb = alt - alt0
        alt_to_descend = alt
    elif np.isclose(0, dist, atol=1e-2) and alt0 == 0:
        # we're at the customer
        alt_to_climb = 0
        alt_to_descend = 0
    elif hspd0 > 0:
        # we're in cruise
        alt_to_climb = 0
        alt_to_descend = alt
    else:
        # not spawned yet or stationary at start loc
        alt_to_climb = alt
        alt_to_descend = alt

    a_h = a
    a_v = a * 2 / 3.5

    v_time_down = calc_vertical_time(vspd_down, alt_to_descend, a_v, abs(vspd0))
    h_time = calc_cruise_time(hspd, dist, a_h, hspd0)

    # if hspd0 > 0:
    #     # We are in cruise phase, so neglect the ascend time required
    #     total_time = v_time_down + h_time
    #     return total_time

    # If we dont hit these blocks, we're just in ascend or we need to calculate
    # the entire mission time. Doesn't matter, same logic
    v_time_up = calc_vertical_time(vspd_up, alt_to_climb, a_v, vspd0)

    total_time = h_time + v_time_up + v_time_down                                    

    return total_time


def calc_vertical_time(vspd, alt, a, vspd0=0):
    """
    Calculates drone ascend or descend travel time considering acceleration.

    Params:
    - vspd: float, vertical (absolute) speed in m/s
    - alt: float, cruise altitude in meters
    - a: float, vertical acceleration in m/s^2
    """
    if vspd0 > vspd:
        print('Something might be going wrong. Higher vspd than expected')
        vspd0 = vspd

    a_time = (vspd - vspd0) / a
    a_dist = 0.5 * a * a_time**2
    if a_dist * 2 > alt:
        a_time = (alt / 2 / a)**0.5
        v_time = 2 * a_time
    else:
        v_cruise_dist = alt - 2 * a_dist
        v_cruise_time = v_cruise_dist / vspd
        v_time = 2 * a_time + v_cruise_time
    
    return v_time

def calc_cruise_time(hspd, dist, a, hspd0=0):
    """
    Calculates drone travel cruise time considering acceleration.

    Params:
    - dist: float, distance in nautical miles
    - hspd: float, horizontal speed in m/s
    - a: float, horizontal acceleration in m/s^2
    """
    if hspd0 > hspd:
        hspd = hspd0

    h_dist = dist * nm
    a_time = (hspd - hspd0) / a
    a_dist = 0.5 * a * a_time**2
    if a_dist * 2 > h_dist:
        a_time = (h_dist / 2 / a)**0.5
        h_time = 2 * a_time
    else:
        h_cruise_dist = h_dist - 2 * a_dist
        h_cruise_time = h_cruise_dist / hspd
        h_time = 2 * a_time + h_cruise_time
    
    return h_time

def calc_truck_ETA2(eta, op_duration, interp=0):
    """Calculates the ETA of the truck to a certain location. The ETAs per 
    waypoint must be given and of same size as op_duration. Calculates the ETA
    up until the specified waypoint (can be modified by slicing the input).
    
    Params: type, description
    
    - eta: list, list of time required to arrive at waypoint (NOT cumulative)
    - op_duration: list, list of the operational time required
    - interp: float, percentage of the way already progressed to first wp """
    if len(eta) > 0:
        eta[0] = eta[0] * (1 - interp)
    # Add operational times
    time_per_wp = list(map(lambda e, o: e + sum(filter(None, o)) if o and 
                                o[0] is not None else e, eta, op_duration))
    return sum(time_per_wp)
     

def in_range(dist, R, distflown):
    """Calculates whether the target location is in range of the drone
    
    Arguments: type, description
    
    dist: float, distance to the target location in nm
    R: float, range of the drone in km
    distflown: float, distance flown by the drone in m"""
    return True if dist * nm + distflown <= R * 1000 else False

def turn_feasible(hdg, dist):
    if dist < 0.1 + (hdg - 60) * 0.005:
        return False
    
    return True