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
                hspd, vspd_up, vspd_down, delivery_time, truck_delivery_time):
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
        - B: float, battery life of a drone
    """
    B = 10**10

    T_i = {}
    T_k = {}
    t_ij = {}
    t_jk = {}
    d_j = {}

    for i in wp_indeces:
        # truck_t = calc_truck_ETA(acidx, i)
        truck_t = calc_truck_ETA2(eta[:i + 1], op_duration[:i + 1])
        T_i[i] = truck_t
        # + launch time
        T_k[i] = truck_t + truck_delivery_time

        _, dist = kwikqdrdist(rte.wplat[i], rte.wplon[i],
                                custlat, custlon)
        
        d_j[i] = dist

        t_ij[i] = calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, 3.5)
        # + delivery drone time to account for delivery on way back
        t_jk[i] = calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, 3.5) + \
                                delivery_time

    T_ik = {}
    for i in wp_indeces:
        for k in wp_indeces:
            T_ik[(i, k)] = T_k[k] - T_i[i]
    
    P = L = wp_indeces

    return L, P, t_ij, t_jk, d_j, T_i, T_k, T_ik, B

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

def calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, a, hspd0=0, vspd0=0):
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

    a_h = a
    a_v = a * 2 / 3.5

    v_time_down = calc_vertical_time(vspd_down, alt, a_v, abs(vspd0))
    h_time = calc_cruise_time(hspd, dist, a_h, hspd0)

    # if vspd0 < 0:
    #     # We are in descend phase already
    #     return v_time_down
    
    if hspd0 > 0:
        # We are in cruise phase, so neglect the ascend time required
        total_time = v_time_down + h_time
        return total_time

    # If we dont hit these blocks, we're just in ascend or we need to calculate
    # the entire mission time. Doesn't matter, same logic
    v_time_up = calc_vertical_time(vspd_up, alt, a_v, vspd0)

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
        print('Something might be going wrong. Higher vspd than expected')
        hspd0 = hspd

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

def calc_truck_ETA2(eta, op_duration):
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