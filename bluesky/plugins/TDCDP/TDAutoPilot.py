import bluesky as bs
import numpy as np
from bluesky.traffic import Autopilot
from math import atan
from bluesky.tools import geo
from bluesky.tools.aero import ft, nm, cas2tas, g0


def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'TDAUTOPILOT',
        'plugin_type':     'sim'
    }
    return config

class TDAutoPilot(Autopilot):
    def __init__(self):
        super().__init__()

    #no longer timed @timed_function(name='fms', dt=bs.settings.fms_dt, manual=True)
    def wppassingcheck(self, qdr, dist): # qdr [deg], dist [m[
        """
        The actwp is the interface between the list of waypoint data in the route object and the autopilot guidance
        when LNAV is on (heading) and optionally VNAV is on (spd & altitude)

        actwp data contains traffic arrays, to allow vectorizing the guidance logic.

        Waypoint switching (just like the adding, deletion in route) are event driven commands and
        therefore not vectorized as they occur rarely compared to the guidance.

        wppassingcheck contains the waypoint switching function:
        - Check which aircraft i have reached their active waypoint
        - Reached function return list of indices where reached logic is True
        - Get the waypoint data to the actwp (active waypoint data)
        - Shift waypoint (last,next etc.) data for aircraft i where necessary
        - Shift and maintain data (see last- and next- prefix in varubale name) e.g. to continue a special turn
        - Prepare some VNAV triggers along the new leg for the VNAV profile (where to start descent/climb)
        """

        # Get list of indices of aircraft which have reached their active waypoint
        # This vectorized function checks the passing of the waypoint using a.o. the current turn radius
        self.idxreached = bs.traf.actwp.reached(qdr, dist, bs.traf.actwp.flyby,
                                       bs.traf.actwp.flyturn,bs.traf.actwp.turnrad,
                                       bs.traf.actwp.turnhdgr,bs.traf.actwp.swlastwp)

        # For the one who have reached their active waypoint, update vectorized leg data for guidance
        for i in self.idxreached:

            #debug commands to check VNAV state while passing waypoint
            #print("Passing waypoint",bs.traf.ap.route[i].wpname[bs.traf.ap.route[i].iactwp])
            #print("dist2wp,dist2vs",self.dist2wp[i]/nm,self.dist2vs[i]/nm) # distance to wp & distance to ToD/ToC

            # Save current wp speed for use on next leg when we pass this waypoint
            # VNAV speeds are always FROM-speeds, so we accelerate/decellerate at the waypoint
            # where this speed is specified, so we need to save it for use now
            # before getting the new data for the next waypoint

            # Get speed for next leg from the waypoint we pass now and set as active spd
            bs.traf.actwp.spd[i]    = bs.traf.actwp.nextspd[i]
            bs.traf.actwp.spdcon[i] = bs.traf.actwp.nextspd[i]

            # Execute stack commands for the still active waypoint, which we pass now
            self.route[i].runactwpstack()

            # Get next wp, if there still is one
            if not bs.traf.actwp.swlastwp[i]:
                lat, lon, alt, bs.traf.actwp.nextspd[i], \
                bs.traf.actwp.xtoalt[i], toalt, \
                    bs.traf.actwp.xtorta[i], bs.traf.actwp.torta[i], \
                    lnavon, flyby, flyturn, turnrad, turnspd, turnhdgr,\
                    bs.traf.actwp.next_qdr[i], bs.traf.actwp.swlastwp[i] =      \
                    self.route[i].getnextwp()  # [m] note: xtoalt,nextaltco are in meters


                bs.traf.actwp.nextturnlat[i], bs.traf.actwp.nextturnlon[i], \
                bs.traf.actwp.nextturnspd[i], bs.traf.actwp.nextturnrad[i], \
                bs.traf.actwp.nextturnhdgr[i],bs.traf.actwp.nextturnidx[i] = \
                    self.route[i].getnextturnwp()

            else:
                # Prevent trying to activate the next waypoint when it was already the last waypoint
                # In case of end of route/no more waypoints: switch off LNAV using the lnavon
                bs.traf.swlnav[i] = False
                bs.traf.swvnav[i] = False
                bs.traf.swvnavspd[i] = False
                continue # Go to next a/c which reached its active waypoint

            # Special turns: specified by turn radius or bank angle
            # If specified, use the given turn radius of passing wp for bank angle
            if flyturn:
                if turnspd<=0.:
                    turnspd = bs.traf.tas[i]

                # Heading rate overrides turnrad
                if turnhdgr>0:
                    turnrad = bs.traf.tas[i]*360./(2*np.pi*turnhdgr)

                # Use last turn radius for bank angle in current turn
                if bs.traf.actwp.turnrad[i] > 0.:
                    self.turnphi[i] = atan(bs.traf.actwp.turnspd[i]*bs.traf.actwp.turnspd[i]/ \
                                           (bs.traf.actwp.turnrad[i]*g0)) # [rad]
                else:
                    self.turnphi[i] = 0.0  # [rad] or leave untouched???

            else:
                self.turnphi[i] = 0.0  #[rad] or leave untouched???



            # Check LNAV switch returned by getnextwp
            # Switch off LNAV if it failed to get next wpdata
            if not lnavon and bs.traf.swlnav[i]:
                bs.traf.swlnav[i] = False
                # Last wp: copy last wp values for alt and speed in autopilot
                if bs.traf.swvnavspd[i] and bs.traf.actwp.nextspd[i]>= 0.0:
                    bs.traf.selspd[i] = bs.traf.actwp.nextspd[i]

            # In case of no LNAV, do not allow VNAV mode to be active
            bs.traf.swvnav[i] = bs.traf.swvnav[i] and bs.traf.swlnav[i]

            bs.traf.actwp.lat[i] = lat  # [deg]
            bs.traf.actwp.lon[i] = lon  # [deg]
            # 1.0 in case of fly by, else fly over
            bs.traf.actwp.flyby[i] = int(flyby)

            # Update qdr and turndist for this new waypoint for ComputeVNAV
            qdr[i], distnmi = geo.qdrdist(bs.traf.lat[i], bs.traf.lon[i],
                                          bs.traf.actwp.lat[i], bs.traf.actwp.lon[i])

            #dist[i] = distnmi * nm
            self.dist2wp[i] = distnmi*nm

            bs.traf.actwp.curlegdir[i] = qdr[i]
            bs.traf.actwp.curleglen[i] = self.dist2wp[i]

            # User has entered an altitude for the new waypoint
            if alt >= -0.01: # positive alt on this waypoint means altitude constraint
                bs.traf.actwp.nextaltco[i] = alt  # [m]
                bs.traf.actwp.xtoalt[i] = 0.0
            else:
                bs.traf.actwp.nextaltco[i] = toalt  # [m]

            #if not bs.traf.swlnav[i]:
            #    bs.traf.actwp.spd[i] = -997.

            # VNAV spd mode: use speed of this waypoint as commanded speed
            # while passing waypoint and save next speed for passing next wp
            # Speed is now from speed! Next speed is ready in wpdata
            if bs.traf.swvnavspd[i] and bs.traf.actwp.spd[i]>= 0.0:
                    bs.traf.selspd[i] = bs.traf.actwp.spd[i]

            # Update turndist so ComputeVNAV works, is there a next leg direction or not?
            if bs.traf.actwp.next_qdr[i] < -900.:
                local_next_qdr = qdr[i]
            else:
                local_next_qdr = bs.traf.actwp.next_qdr[i]

            # Calculate turn dist (and radius which we do not use now, but later) now for scalar variable [i]
            bs.traf.actwp.turndist[i], dummy = \
                bs.traf.actwp.calcturn_nonvec(bs.traf.tas[i], self.bankdef[i],
                                        qdr[i], local_next_qdr,i,turnrad,turnhdgr,flyturn)  # update turn distance for VNAV

            # Get flyturn switches and data
            bs.traf.actwp.flyturn[i]      = flyturn
            bs.traf.actwp.turnrad[i]      = turnrad
            bs.traf.actwp.turnspd[i]      = turnspd
            bs.traf.actwp.turnhdgr[i]     = turnhdgr

            # Pass on whether currently flyturn mode:
            # at beginning of leg,c copy tonextwp to lastwp
            # set next turn False
            bs.traf.actwp.turnfromlastwp[i] = bs.traf.actwp.turntonextwp[i]
            bs.traf.actwp.turntonextwp[i]   = False

            # Keep both turning speeds: turn to leg and turn from leg
            bs.traf.actwp.oldturnspd[i]  = bs.traf.actwp.turnspd[i] # old turnspd, turning by this waypoint
            if bs.traf.actwp.flyturn[i]:
                bs.traf.actwp.turnspd[i] = turnspd                  # new turnspd, turning by next waypoint
            else:
                bs.traf.actwp.turnspd[i] = -990.

            # Reduce turn dist for reduced turnspd
            if bs.traf.actwp.flyturn[i] and bs.traf.actwp.turnrad[i]<0.0 and bs.traf.actwp.turnspd[i]>=0.:
                turntas = cas2tas(bs.traf.actwp.turnspd[i], bs.traf.alt[i])
                bs.traf.actwp.turndist[i] = bs.traf.actwp.turndist[i]*turntas*turntas/(bs.traf.tas[i]*bs.traf.tas[i])

            # VNAV = FMS ALT/SPD mode incl. RTA
            self.ComputeVNAV(i, toalt, bs.traf.actwp.xtoalt[i], bs.traf.actwp.torta[i],
                             bs.traf.actwp.xtorta[i])

        

        # End of reached-loop: the per waypoint i switching loop

        # Update qdr2wp with up-to-date qdr, now that we have checked passing wp
        self.qdr2wp = qdr%360.

        # Continuous guidance when speed constraint on active leg is in update-method

        # If still an RTA in the route and currently no speed constraint
        for iac in np.where((bs.traf.actwp.torta > -99.)*(bs.traf.actwp.spdcon<0.0))[0]:
            iwp = bs.traf.ap.route[iac].iactwp
            if bs.traf.ap.route[iac].wprta[iwp]>-99.:

                 # For all a/c flying to an RTA waypoint, recalculate speed more often
                dist2go4rta = geo.kwikdist(bs.traf.lat[iac],bs.traf.lon[iac],
                                           bs.traf.actwp.lat[iac],bs.traf.actwp.lon[iac])*nm \
                               + bs.traf.ap.route[iac].wpxtorta[iwp] # last term zero for active wp rta

                # Set bs.traf.actwp.spd to rta speed, if necessary
                self.setspeedforRTA(iac,bs.traf.actwp.torta[iac],dist2go4rta)

                # If VNAV speed is on (by default coupled to VNAV), use it for speed guidance
                if bs.traf.swvnavspd[iac] and bs.traf.actwp.spd[iac]>=0.0:
                     bs.traf.selspd[iac] = bs.traf.actwp.spd[iac]