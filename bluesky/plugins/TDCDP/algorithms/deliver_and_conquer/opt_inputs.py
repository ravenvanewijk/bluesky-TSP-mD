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

def calc_inputs(acidx, rte, wp_indeces, custlat, custlon, alt, hspd, vspd_up, 
                vspd_down, delivery_time, truck_delivery_time):
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

    for i in wp_indeces:
        truck_t = calc_truck_ETA(acidx, i)
        T_i[i] = truck_t
        # + launch time
        T_k[i] = truck_t + truck_delivery_time

        _, dist = kwikqdrdist(rte.wplat[i], rte.wplon[i],
                                custlat, custlon)
        
        t_ij[i] = calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, 3.5, i)
        # + delivery drone time
        t_jk[i] = calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, 3.5, i) + delivery_time

    T_ik = {}
    for i in wp_indeces:
        for k in wp_indeces:
            T_ik[(i, k)] = T_k[k] - T_i[i]
    
    P = L = wp_indeces

    return L, P, t_ij, t_jk, T_i, T_k, T_ik, B

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

def calc_drone_ETA(dist, hspd, vspd_up, vspd_down, alt, a, i):
    """
    Calculates drone travel time considering acceleration.

    Params:
    - dist: float, distance in nautical miles
    - hspd: float, horizontal speed in m/s
    - vspd_up: float, vertical ascent speed in m/s
    - vspd_down: float, vertical descent speed in m/s
    - alt: float, cruise altitude in meters
    - a: float, horizontal acceleration in m/s^2
    - a: float, vertical ascent acceleration in m/s^2
    - a: float, vertical descent acceleration in m/s^2
    """

    # Horizontal travel
    h_dist = dist * nm
    a_time = hspd / a
    a_dist = 0.5 * a * a_time**2
    if a_dist * 2 > h_dist:
        a_time = (h_dist / 2 / a)**0.5
        h_time = 2 * a_time
    else:
        h_cruise_dist = h_dist - 2 * a_dist
        h_cruise_time = h_cruise_dist / hspd
        h_time = 2 * a_time + h_cruise_time

    a = a * 2 / 3.5
    # Vertical ascent
    v_acc_time_up = vspd_up / a
    v_acc_dist_up = 0.5 * a * v_acc_time_up**2
    if v_acc_dist_up * 2 > alt:
        v_acc_time_up = (alt / 2 / a)**0.5
        v_time_up = 2 * v_acc_time_up
    else:
        v_cruise_dist_up = alt - 2 * v_acc_dist_up
        v_cruise_time_up = v_cruise_dist_up / vspd_up
        v_time_up = 2 * v_acc_time_up + v_cruise_time_up

    # Vertical descent
    v_acc_time_down = vspd_down / a
    v_acc_dist_down = 0.5 * a * v_acc_time_down**2
    if v_acc_dist_down * 2 > alt:
        v_acc_time_down = (alt / 2 / a)**0.5
        v_time_down = 2 * v_acc_time_down
    else:
        v_cruise_dist_down = alt - 2 * v_acc_dist_down
        v_cruise_time_down = v_cruise_dist_down / vspd_down
        v_time_down = 2 * v_acc_time_down + v_cruise_time_down

    total_time = h_time + v_time_up + v_time_down

    return total_time

