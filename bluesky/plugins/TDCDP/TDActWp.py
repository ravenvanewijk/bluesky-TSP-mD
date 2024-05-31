from bluesky.traffic import ActiveWaypoint
from bluesky.tools.misc import degto180
from bluesky.tools.aero import g0
from bluesky.traffic import Route
import numpy as np
import bluesky as bs

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'TDACTWP',
        'plugin_type':     'sim'
    }
    return config


class TDActWp(ActiveWaypoint):
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.idxnextrta  = np.array([], dtype = int)    # Index of next RTA waypoint in route

    def create(self, n=1):
        super().create(n)
        # LNAV route navigation
        self.idxnextrta[-n:]  = -999   # Index of next RTA waypoint in route

    # Overwrite reached function, should be more strict on reaching waypoints
    def reached(self, qdr, dist, flyby, flyturn, turnrad, turnhdgr, swlastwp):
        # Calculate distance before waypoint where to start the turn
        # Note: this is a vectorized function, called with numpy traffic arrays
        # It returns the indices where the Reached criterion is True
        #
        # Turn radius:      R = V2 tan phi / g
        # Distance to turn: wpturn = R * tan (1/2 delhdg) but max 4 times radius
        # using default bank angle per flight phase
        # First calculate turn distance
        next_qdr = np.where(self.next_qdr < -900., qdr, self.next_qdr)
        turntas = np.where(self.turnspd<0.0,bs.traf.tas,self.turnspd)
        flybyturndist,turnrad = self.calcturn_vec(turntas,bs.traf.ap.bankdef,qdr,next_qdr,turnrad,turnhdgr,flyturn)

        # Turb dist iz ero for flyover, calculated distance for others
        self.turndist = np.logical_or(flyby,flyturn)*flybyturndist

        # Avoid circling by checking too close to waypoint based on ground speed, assumption using vicinity criterion:
        # flying away and within 4 sec distance based on ground speed (4 sec = sensitivity tuning parameter)

        close2wp = dist/(np.maximum(0.0001,np.abs(bs.traf.gs)))<4.0 # Waypoint is within 4 seconds flight time
        tooclose2turn = close2wp*(np.abs(degto180(bs.traf.trk % 360. - qdr % 360.)) > 90.)

        # When too close to waypoint or we have passed the active waypoint, based on leg direction,switch active waypoint
        # was:  away  = np.logical_or(close2wp,swlastwp)*(np.abs(degto180(bs.traf.trk%360. - qdr%360.)) > 90.) # difference large than 90
        awayorpassed =  np.logical_or(tooclose2turn,np.abs(degto180(qdr-bs.traf.actwp.curlegdir))>90.)
        # Should no longer be needed with leg direction
        # Ratio between distance close enough to switch to next wp when flying away
        # When within pro1 nm and flying away: switch also
        #proxfact = 1.02 # Turnradius scales this contant , factor => [turnrad]
        #incircle = dist<turnrad*proxfact
        #circling = away*incircle # [True/False] passed wp,used for flyover as well

        # Check whether shift based dist is required, set closer than WP turn distance
        # Detect indices
        #swreached = np.where(bs.traf.swlnav * np.logical_or(awayorpassed,np.logical_or(dist < self.turndist,circling)))[0]
        dist_logical = dist < self.turndist
        # # MODIFIED W.R.T. ORIGINAL: TRUCK SHOULD ALWAYS HAVE SUFFICIENT DISTANCE LEFT TO MAKE THE TURN.
        dist_logical = np.where(np.array(bs.traf.type) == 'TRUCK', False, dist_logical)
        swreached = np.where(bs.traf.swlnav * np.logical_or(awayorpassed,dist_logical))[0]
        # tooclose2turn
        # array([False])
        # Return indices for which condition is True/1.0 for a/c where we have reached waypoint
        return swreached


    # Calculate turn distance for array or scalar
    def calcturn_vec(self,tas,bank,wpqdr,next_wpqdr,turnrad=-999.,turnhdgr = -999.,flyturn=False):
        """Calculate distance to wp where to start turn and turn radius in meters"""

        # Tas is also used ti

        # Calculate turn radius in meters using current speed or use specified turnradius in m
        turnrad = np.where(np.logical_and(flyturn,turnrad+0.*tas>0.), #turn radius specified? (0.*tas for dimension)

                           # user specified radius
                           turnrad +0.*tas,


                           np.where(np.logical_and(flyturn,turnhdgr+0.*tas>0),

                                   # turn radius based on heading rate?
                                   tas/(2*np.pi)*(360./turnhdgr),

                                   # bank, tas => turn radius
                                   tas * tas / (np.maximum(0.01, np.tan(bank)) * g0)))#else none specified, calculate


        # Condition 1: Check if bs.traf.type is 'TRUCK'
        is_truck = np.array(bs.traf.type) == 'TRUCK'
        
        # Condition 2: Check if np.abs(degto180(wpqdr % 360 - next_wpqdr % 360)) > 179. 
        # This occurs when the truck travels in a road and has to follow the same road back to continue on its way
        angle_diff = np.abs(degto180(wpqdr % 360 - next_wpqdr % 360))
        large_angle = angle_diff > 179

        # Combine conditions
        conditions = is_truck & large_angle

        # Calculate turndist based on the conditions
        turndist = np.where(
            conditions,
            np.abs(turnrad * np.tan(np.radians(0.5 * np.abs(degto180(wpqdr % 360 - (next_wpqdr - 45) % 360))))),
            np.abs(turnrad * np.tan(np.radians(0.5 * angle_diff)))
        )
        return turndist,turnrad

    def calcturn_nonvec(self,tas,bank,wpqdr,next_wpqdr,i,turnrad=-999.,turnhdgr = -999.,flyturn=False):
        """Calculate distance to wp where to start turn and turn radius in meters"""

        # Calculate turn radius in meters using current speed or use specified turn radius in m
        if flyturn and turnrad + 0.0 * tas > 0.0:
            turnrad = turnrad + 0.0 * tas
        elif flyturn and turnhdgr + 0.0 * tas > 0.0:
            turnrad = tas / (2 * np.pi) * (360.0 / turnhdgr)
        else:
            turnrad = tas * tas / (np.maximum(0.01, np.tan(bank)) * g0)

        # Condition 1: Check if bs.traf.type is 'TRUCK'
        is_truck = bs.traf.type[i] == 'TRUCK' if i is not None else False
        
        # Condition 2: Check if angle difference is greater than 179
        angle_diff = np.abs(degto180(wpqdr % 360 - next_wpqdr % 360))
        large_angle = angle_diff > 179
        
        # Combine conditions
        conditions = is_truck and large_angle
        
        # Calculate turndist based on the conditions
        if conditions:
            turndist = np.abs(turnrad * np.tan(np.radians(0.5 * np.abs(degto180(wpqdr % 360 - (next_wpqdr - 45) % 360)))))
        else:
            turndist = np.abs(turnrad * np.tan(np.radians(0.5 * angle_diff)))

        return turndist, turnrad