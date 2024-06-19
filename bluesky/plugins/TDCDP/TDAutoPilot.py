import bluesky as bs
import numpy as np
from bluesky.traffic import Autopilot
from bluesky.traffic.autopilot import distaccel, calcvrta
from bluesky.tools import geo
from bluesky.tools.misc import degto180
from bluesky.tools.aero import ft, nm, cas2tas, g0, tas2cas, vcas2tas, vcasormach2tas
from math import atan

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
                                       bs.traf.actwp.turnhdgr,bs.traf.actwp.swlastwp,
                                       bs.traf.actwp.operation)

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
                    bs.traf.actwp.idxnextrta[i], bs.traf.actwp.torta[i], \
                    lnavon, flyby, flyturn, turnrad, turnspd, turnhdgr,\
                    bs.traf.actwp.next_qdr[i], bs.traf.actwp.swlastwp[i],\
                    bs.traf.actwp.operation[i]\
                    = self.route[i].getnextwp()  # [m] note: xtoalt,nextaltco are in meters


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
                             bs.traf.actwp.idxnextrta[i])

        

        # End of reached-loop: the per waypoint i switching loop

        # Update qdr2wp with up-to-date qdr, now that we have checked passing wp
        self.qdr2wp = qdr%360.

        # Continuous guidance when speed constraint on active leg is in update-method

        # If still an RTA in the route and currently no speed constraint
        for iac in np.where((bs.traf.actwp.idxnextrta > -99.)*(bs.traf.actwp.spdcon<0.0))[0]:
            if self.inturn[iac]:
                continue
            acrte = self.route[iac]
            # Set bs.traf.actwp.spd to rta speed, if necessary
            if np.any(np.array(acrte.wprta[acrte.iactwp:]) > 0):
                self.setspeedforRTA(iac,bs.traf.actwp.torta[iac],bs.traf.actwp.idxnextrta[iac])

            # If VNAV speed is on (by default coupled to VNAV), use it for speed guidance
            if bs.traf.swvnavspd[iac] and bs.traf.actwp.spd[iac]>=0.0:
                    bs.traf.selspd[iac] = bs.traf.actwp.spd[iac]

    def update(self):
        # FMS LNAV mode:
        # qdr[deg],distinnm[nm]
        qdr, distinnm = geo.qdrdist(bs.traf.lat, bs.traf.lon,
                                    bs.traf.actwp.lat, bs.traf.actwp.lon)  # [deg][nm])

        self.qdr2wp  = qdr
        self.dist2wp = distinnm*nm  # Conversion to meters

        # Check possible waypoint shift. Note: qdr, dist2wp will be updated accordingly in case of wp switch
        self.wppassingcheck(qdr, self.dist2wp) # Updates self.qdr2wp when necessary

        #================= Continuous FMS guidance ========================

        # Note that the code below is vectorized, with traffic arrays, so for all aircraft
        # ComputeVNAV and inside waypoint loop of wppassingcheck, it was scalar (per a/c with index i)

        # VNAV altitude guidance logic (using the variables prepared by ComputeVNAV when activating waypoint)

        # First question is:
        # - Can we please we start to descend or to climb?
        #
        # The variable dist2vs indicates the distance to the active waypoint where we should start our climb/descend
        # Only use this logic if there is a valid next altitude constraint (nextaltco).
        #
        # Well, when Top of Descent (ToD) switch is on, descend as late as possible,
        # But when Top of Climb switch is on or off, climb as soon as possible, only difference is steepness used in ComputeVNAV
        # to calculate bs.traf.actwp.vs

        startdescorclimb = (bs.traf.actwp.nextaltco>=-0.1) * \
                           np.logical_or((bs.traf.alt>bs.traf.actwp.nextaltco) *\
                                         np.logical_or((self.dist2wp < self.dist2vs+bs.traf.actwp.turndist),
                                                       (np.logical_not(self.swtod))),
                                         bs.traf.alt<bs.traf.actwp.nextaltco)

        # print("self.dist2vs =",self.dist2vs)

        # If not lnav:Climb/descend if doing so before lnav/vnav was switched off
        #    (because there are no more waypoints). This is needed
        #    to continue descending when you get into a conflict
        #    while descending to the destination (the last waypoint)
        #    Use 0.1 nm (185.2 m) circle in case turndist might be zero
        self.swvnavvs = bs.traf.swvnav * np.where(bs.traf.swlnav, startdescorclimb,
                                        self.dist2wp <= np.maximum(0.1*nm,bs.traf.actwp.turndist))

        # Recalculate V/S based on current altitude and distance to next alt constraint
        # How much time do we have before we need to descend?
        # Now done in ComputeVNAV
        # See ComputeVNAV for bs.traf.actwp.vs calculation

        self.vnavvs  = np.where(self.swvnavvs, bs.traf.actwp.vs, self.vnavvs)
        #was: self.vnavvs  = np.where(self.swvnavvs, self.steepness * bs.traf.gs, self.vnavvs)

        # self.vs = np.where(self.swvnavvs, self.vnavvs, self.vsdef * bs.traf.limvs_flag)
        # for VNAV use fixed V/S and change start of descent
        selvs = np.where(abs(bs.traf.selvs) > 0.1, bs.traf.selvs, self.vsdef) # m/s
        self.vs  = np.where(self.swvnavvs, self.vnavvs, selvs)
        self.alt = np.where(self.swvnavvs, bs.traf.actwp.nextaltco, bs.traf.selalt)

        # When descending or climbing in VNAV also update altitude command of select/hold mode
        bs.traf.selalt = np.where(self.swvnavvs,bs.traf.actwp.nextaltco,bs.traf.selalt)

        # LNAV commanded track angle
        self.trk = np.where(bs.traf.swlnav, self.qdr2wp, self.trk)

        # FMS speed guidance: anticipate accel/decel distance for next leg or turn

        # Calculate actual distance it takes to decelerate/accelerate based on two cases: turning speed (decel)

        # Normally next leg speed (actwp.spd) but in case we fly turns with a specified turn speed
        # use the turn speed

        # Is turn speed specified and are we not already slow enough? We only decelerate for turns, not accel.
        turntas       = np.where(bs.traf.actwp.nextturnspd>0.0, vcas2tas(bs.traf.actwp.nextturnspd, bs.traf.alt),
                                 -1.0+0.*bs.traf.tas)
        # If the current cruise speed is smaller than the turntas, then just use
        # the current cruise speed to turn
        turntas = np.where(turntas < bs.traf.gs, turntas, bs.traf.gs)
        # Switch is now whether the aircraft has any turn waypoints
        swturnspd     = bs.traf.actwp.nextturnidx > 0
        turntasdiff   = np.maximum(0.,(bs.traf.tas - turntas)*(turntas>0.0))

        # t = (v1-v0)/a ; x = v0*t+1/2*a*t*t => dx = (v1*v1-v0*v0)/ (2a)
        dxturnspdchg = distaccel(turntas,bs.traf.tas, bs.traf.perf.axmax)

        # Decelerate or accelerate for next required speed because of speed constraint or RTA speed
        # Note that because nextspd comes from the stack, and can be either a mach number or
        # a calibrated airspeed, it can only be converted from Mach / CAS [kts] to TAS [m/s]
        # once the altitude is known.
        nexttas = vcasormach2tas(bs.traf.actwp.nextspd, bs.traf.alt)

        dxspdconchg = distaccel(bs.traf.tas, nexttas, bs.traf.perf.axmax)

        qdrturn, dist2turn = geo.qdrdist(bs.traf.lat, bs.traf.lon,
                                        bs.traf.actwp.nextturnlat, bs.traf.actwp.nextturnlon)

        self.qdrturn = qdrturn
        dist2turn = dist2turn * nm

        # Where we don't have a turn waypoint, as in turn idx is negative, then put distance
        # as Earth circumference.
        self.dist2turn = np.where(bs.traf.actwp.nextturnidx > 0, dist2turn, 40075000)

        # Check also whether VNAVSPD is on, if not, SPD SEL has override for next leg
        # and same for turn logic
        usenextspdcon = (self.dist2wp < dxspdconchg)*(bs.traf.actwp.nextspd>-990.) * \
                            bs.traf.swvnavspd*bs.traf.swvnav*bs.traf.swlnav

        useturnspd = np.logical_or(bs.traf.actwp.turntonextwp,
                                   (self.dist2turn < (dxturnspdchg+bs.traf.actwp.turndist))) * \
                                        swturnspd*bs.traf.swvnavspd*bs.traf.swvnav*bs.traf.swlnav

        # Hold turn mode can only be switched on here, cannot be switched off here (happeps upon passing wp)
        bs.traf.actwp.turntonextwp = bs.traf.swlnav*np.logical_or(bs.traf.actwp.turntonextwp,useturnspd)

        # Which CAS/Mach do we have to keep? VNAV, last turn or next turn?
        oncurrentleg = (abs(degto180(bs.traf.trk - qdr)) < 2.0) # [deg]
        inoldturn    = (bs.traf.actwp.oldturnspd > 0.) * np.logical_not(oncurrentleg)

        # Avoid using old turning speeds when turning of this leg to the next leg
        # by disabling (old) turningspd when on leg
        bs.traf.actwp.oldturnspd = np.where(oncurrentleg*(bs.traf.actwp.oldturnspd>0.), -998.,
                                            bs.traf.actwp.oldturnspd)

        # turnfromlastwp can only be switched off here, not on (latter happens upon passing wp)
        bs.traf.actwp.turnfromlastwp = np.logical_and(bs.traf.actwp.turnfromlastwp,inoldturn)
        # Select speed: turn sped, next speed constraint, or current speed constraint
        bs.traf.selspd = np.where(useturnspd,bs.traf.actwp.nextturnspd,
                                  np.where(usenextspdcon, bs.traf.actwp.nextspd,
                                           np.where((bs.traf.actwp.spdcon>=0)*bs.traf.swvnavspd,bs.traf.actwp.spd,
                                                                            bs.traf.selspd)))

        # Temporary override when still in old turn
        bs.traf.selspd = np.where(inoldturn*(bs.traf.actwp.oldturnspd>0.)*bs.traf.swvnavspd*bs.traf.swvnav*bs.traf.swlnav,
                                  bs.traf.actwp.oldturnspd,bs.traf.selspd)

        self.inturn = np.logical_or(useturnspd,inoldturn)

        # Below crossover altitude: CAS=const, above crossover altitude: Mach = const
        self.tas = vcasormach2tas(bs.traf.selspd, bs.traf.alt)

    def ComputeVNAV(self, idx, toalt, xtoalt, torta, idxnextrta):
        """
        This function to do VNAV (and RTA) calculations is only called only once per leg for one aircraft idx.
        If:
         - switching to next waypoint
         - when VNAV is activated
         - when a DIRECT is given

        It prepares the profile of this leg using the the current altitude and the next altitude constraint (nextaltco).
        The distance to the next altitude constraint is given by xtoalt [m] after active waypoint.

        Options are (classic VNAV logic, swtoc and swtod True):
        - no altitude constraint in the future, do nothing
        - Top of CLimb logic (swtoc=True): if next altitude constrain is baove us, climb as soon as possible with default steepness
        - Top of Descent Logic (swtod =True) Use ToD logic: descend as late aspossible, based on
          steepness. Prepare a ToD somewhere on the leg if necessary based on distance to next altitude constraint.
          This is done by calculating distance to next waypoint where descent should start

        Alternative logic (e.g. for UAVs or GA):
        - swtoc=False and next alt co is above us, climb with the angle/steepness needed to arrive at the altitude at
        the waypoint with the altitude constraint (xtoalt m after active waypoint)
        - swtod=False and next altco is below us, descend with the angle/steepness needed to arrive at at the altitude at
        the waypoint with the altitude constraint (xtoalt m after active waypoint)

        Output if this function:
        self.dist2vs = distance 2 next waypoint where climb/descent needs to activated
        bs.traf.actwp.vs =  V/S to be used during climb/descent part, so when dist2wp<dist2vs [m] (to next waypoint)
        """

        #print ("ComputeVNAV for",bs.traf.id[idx],":",toalt/ft,"ft  ",xtoalt/nm,"nm")
        #print("Called by",callstack()[1].function)

        # Check  whether active waypoint speed needs to be adjusted for RTA
        # sets bs.traf.actwp.spd, if necessary
        # debug print("xtorta+legdist =",(xtorta+legdist)/nm)
        self.setspeedforRTA(idx, torta, idxnextrta)  # all scalar

        # Check if there is a target altitude and VNAV is on, else return doing nothing
        if toalt < 0 or not bs.traf.swvnav[idx]:
            self.dist2vs[idx] = -999999. #dist to next wp will never be less than this, so VNAV will do nothing
            return

        # So: somewhere there is an altitude constraint ahead
        # Compute proper values for bs.traf.actwp.nextaltco, self.dist2vs, self.alt, bs.traf.actwp.vs
        # Descent VNAV mode (T/D logic)
        #
        # xtoalt  =  distance to go to next altitude constraint at a waypoint in the route
        #            (could be beyond next waypoint) [m]
        #
        # toalt   = altitude at next waypoint with an altitude constraint
        #
        # dist2vs = autopilot starts climb or descent when the remaining distance to next waypoint
        #           is this distance
        #
        #
        # VNAV Guidance principle:
        #
        #
        #                          T/C------X---T/D
        #                           /    .        \
        #                          /     .         \
        #       T/C----X----.-----X      .         .\
        #       /           .            .         . \
        #      /            .            .         .  X---T/D
        #     /.            .            .         .        \
        #    / .            .            .         .         \
        #   /  .            .            .         .         .\
        # pos  x            x            x         x         x X
        #
        #
        #  X = waypoint with alt constraint  x = Wp without prescribed altitude
        #
        # - Ignore and look beyond waypoints without an altitude constraint
        # - Climb as soon as possible after previous altitude constraint
        #   and climb as fast as possible, so arriving at alt earlier is ok
        # - Descend at the latest when necessary for next altitude constraint
        #   which can be many waypoints beyond current actual waypoint
        epsalt = 2.*ft # deadzone
        #
        if bs.traf.alt[idx] > toalt + epsalt:
            # Stop potential current climb (e.g. due to not making it to previous altco)
            # then stop immediately, as in: do not make it worse.
            if bs.traf.vs[idx]>0.0001:
                self.vnavvs[idx] = 0.0
                self.alt[idx] = bs.traf.alt[idx]
                if bs.traf.swvnav[idx]:
                    bs.traf.selalt[idx] = bs.traf.alt[idx]

            # Descent modes: VNAV (= swtod/Top of Descent logic) or aiming at next alt constraint

            # Calculate max allowed altitude at next wp (above toalt)
            bs.traf.actwp.nextaltco[idx] = toalt  # [m] next alt constraint
            bs.traf.actwp.xtoalt[idx]    = xtoalt # [m] distance to next alt constraint measured from next waypoint


            # VNAV ToD logic
            if self.swtod[idx]:
                # Get distance to waypoint
                self.dist2wp[idx] = nm*geo.kwikdist(bs.traf.lat[idx], bs.traf.lon[idx],
                                                 bs.traf.actwp.lat[idx],
                                                 bs.traf.actwp.lon[idx])  # was not always up to date, so update first

                # Distance to next waypoint where we need to start descent (top of descent) [m]
                descdist = abs(bs.traf.alt[idx] - toalt) / self.steepness  # [m] required length for descent, uses default steepness!
                self.dist2vs[idx] = descdist - xtoalt   # [m] part of that length on this leg

                #print(bs.traf.id[idx],"traf.alt =",bs.traf.alt[idx]/ft,"ft toalt = ",toalt/ft,"ft descdist =",descdist/nm,"nm")
                #print ("d2wp = ",self.dist2wp[idx]/nm,"nm d2vs = ",self.dist2vs[idx]/nm,"nm")
                #print("xtoalt =",xtoalt/nm,"nm descdist =",descdist/nm,"nm")

                # Exceptions: Descend now?
                #print("Active WP:",bs.traf.ap.route[idx].wpname[bs.traf.ap.route[idx].iactwp])
                #print("dist2wp,turndist, dist2vs= ",self.dist2wp[idx],bs.traf.actwp.turndist[idx],self.dist2vs[idx])
                if self.dist2wp[idx] - 1.02*bs.traf.actwp.turndist[idx] < self.dist2vs[idx]:  # Urgent descent, we're late![m]
                    # Descend now using whole remaining distance on leg to reach altitude
                    self.alt[idx] = bs.traf.actwp.nextaltco[idx]  # dial in altitude of next waypoint as calculated
                    t2go = self.dist2wp[idx]/max(0.01,bs.traf.gs[idx])
                    bs.traf.actwp.vs[idx] = (bs.traf.alt[idx]-toalt)/max(0.01,t2go)

                elif xtoalt<descdist: # Not on this leg, no descending is needed at next waypoint
                    # Top of decent needs to be on this leg, as next wp is in descent
                    bs.traf.actwp.vs[idx] = -abs(self.steepness) * (bs.traf.gs[idx] +
                                                                    (bs.traf.gs[idx] < 0.2 * bs.traf.tas[idx]) *
                                                                    bs.traf.tas[idx])

                else:
                    # else still level
                    bs.traf.actwp.vs[idx] = 0.0

            else:

                # We are higher but swtod = False, so there is no ToD descent logic, simply aim at next altco
                steepness_ = (bs.traf.alt[idx]-bs.traf.actwp.nextaltco[idx])/(max(0.01,self.dist2wp[idx]+xtoalt))
                bs.traf.actwp.vs[idx] = -abs(steepness_) * (bs.traf.gs[idx] +
                                                           (bs.traf.gs[idx] < 0.2 * bs.traf.tas[idx]) * bs.traf.tas[
                                                               idx])
                self.dist2vs[idx]      = 99999. #[m] Forces immediate descent as current distance to next wp will be less

                # print("in else swtod for ", bs.traf.id[idx])

        # VNAV climb mode: climb as soon as possible (T/C logic)
        elif bs.traf.alt[idx] < toalt - 9.9 * ft:
            # Stop potential current descent (e.g. due to not making it to previous altco)
            # then stop immediately, as in: do not make it worse.
            if bs.traf.vs[idx] < -0.0001:
                self.vnavvs[idx] = 0.0
                self.alt[idx] = bs.traf.alt[idx]
                if bs.traf.swvnav[idx]:
                    bs.traf.selalt[idx] = bs.traf.alt[idx]

            # Altitude we want to climb to: next alt constraint in our route (could be further down the route)
            bs.traf.actwp.nextaltco[idx] = toalt   # [m]
            bs.traf.actwp.xtoalt[idx]    = xtoalt  # [m] distance to next alt constraint measured from next waypoint
            self.alt[idx]          = bs.traf.actwp.nextaltco[idx]  # dial in altitude of next waypoint as calculated
            self.dist2vs[idx]      = 99999. #[m] Forces immediate climb as current distance to next wp will be less

            t2go = max(0.1, self.dist2wp[idx]+xtoalt) / max(0.01, bs.traf.gs[idx])
            if self.swtoc[idx]:
                steepness_ = self.steepness # default steepness
            else:
                steepness_ = (bs.traf.alt[idx] - bs.traf.actwp.nextaltco[idx]) / (max(0.01, self.dist2wp[idx] + xtoalt))

            bs.traf.actwp.vs[idx]  = np.maximum(steepness_*bs.traf.gs[idx],
                                       (bs.traf.actwp.nextaltco[idx] - bs.traf.alt[idx]) / t2go) # [m/s]
        # Level leg: never start V/S
        else:
            self.dist2vs[idx] = -999.  # [m]

        return

    def setspeedforRTA(self, idx, torta, idxnextrta):
        #debug print("setspeedforRTA called, torta,xtorta =",torta,xtorta/nm)
        # Overwrite these here because I don't want to find all the instances 
        # in the code where these are used wrongly
        acrte = bs.traf.ap.route[idx]
        if np.any(np.array(acrte.wprta[acrte.iactwp:]) > 0):
            foo= np.where(np.array(acrte.wprta) > 0)[0]
            idxnextrta = int(foo[np.where(foo >= acrte.iactwp)[0]][0])
            torta = acrte.wprta[idxnextrta]  
        else:
            # No RTAs to set
            return False

        # Calculate required CAS to meet RTA
        # for aircraft nr. idx (scalar)
        if torta < -90. or self.inturn[idx] : # -999 signals there is no RTA defined in remainder of route
            return False

        deltime = torta-bs.sim.simt # Remaining time to next RTA [s] in simtime
        if deltime>0: # Still possible?
            gsrta = self.calcvrta(idx, idxnextrta)

            # Subtract tail wind speed vector
            tailwind = (bs.traf.windnorth[idx]*bs.traf.gsnorth[idx] + bs.traf.windeast[idx]*bs.traf.gseast[idx]) / \
                        bs.traf.gs[idx]
            
            if np.isnan(tailwind):
                tailwind = 0 

            # Convert to CAS
            rtacas = tas2cas(gsrta-tailwind,bs.traf.alt[idx])

            # Performance limits on speed will be applied in traf.update
            if bs.traf.actwp.spdcon[idx]<0. and bs.traf.swvnavspd[idx]:
                bs.traf.actwp.spd[idx] = rtacas
                #print("setspeedforRTA: xtorta =",xtorta)

            return rtacas
        else:
            return False

    def calcvrta(self,acidx, idxnextrta):
        if idxnextrta < 0:
            return gs
        # Update the ground speed to match the RTA
        # Get the aircraft information
        rte = bs.traf.ap.route[acidx]
        gs = bs.traf.gs[acidx]
        lat = bs.traf.lat[acidx]
        lon = bs.traf.lon[acidx]
        amax = bs.traf.perf.axmax[acidx]
        iactwp = rte.iactwp
        # If ground speed is 0, then return accelerate by default
        if gs <= 0:
            return gs + amax * bs.sim.simdt
        # Calculate the time to get to it considering the current ground speed
        # as the cruise speed
        rte_time = 0
        prev_turn_time = 0
        prev_turn_dist = 0
        prev_wplat, prev_wplon = lat, lon
        for wpidx in range(iactwp, idxnextrta + 1):
            wplat = rte.wplat[wpidx]
            wplon = rte.wplon[wpidx]
            # If the previous waypoint was a turn one, add extra time
            rte_time += prev_turn_time
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
                turnrate = np.degrees(g0 * np.tan(self.bankdef[acidx]
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
                time_cruise = cruise_dist / gs
                time_decel = max((gs - turnspd), 0) / amax
                
                # Finally add to estimated time
                rte_time += time_cruise + time_decel + time_turn/2
                
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
                time_cruise = cruise_dist / gs
                
                rte_time += time_cruise
                
                prev_turn_time = 0
                prev_turn_dist = 0
                prev_wplat, prev_wplon = wplat, wplon
        
        # Is the aircraft going too slow?
        time_diff = rte_time - (bs.traf.actwp.torta[acidx] - bs.sim.simt)
        
        if abs(time_diff) < 0.1:
            # Within tolerance, do nothing
            return gs
        elif time_diff > 0:
            # Going too slow, accelerate
            # print('accelerating', time_diff)
            return gs + amax * bs.sim.simdt
        elif time_diff < 0:
            # print('decelerating', time_diff)
            # Going too fast, slow down
            return gs - amax * bs.sim.simdt
        else:
            return gs