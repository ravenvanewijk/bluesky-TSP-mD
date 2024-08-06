from bluesky.traffic.route import Route
import math
import numpy as np
import bluesky as bs
from bluesky.tools import geo
from bluesky.tools.aero import ft, g0, nm, mach2cas
from bluesky.tools.misc import degto180, txt2alt, txt2spd
from bluesky import stack
from bluesky.stack.cmdparser import command
from bluesky.stack.argparser import argparsers
from bluesky.plugins.TDCDP.TDdronemanager import get_wpname

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'TDRoute',
        'plugin_type':     'sim'
    }
    return config

class TDRoute(Route):
    def __init__(self, acid):
        super().__init__(acid)
        self.operation_wp = []
        self.operation_duration = []
        self.children = []
        self.op_type = []

    @stack.command
    def addtdwaypoints(acidx: 'acid', *args):
        # Args come in this order: lat, lon, alt, spd, TURNSPD/TURNRAD/FLYBY, turnspeed or turnrad value
        # (extended) add operation arg
        Route.addwaypoints(acidx, *args)
        # Operation managing
        wpcount = len(args)//6
        acid = bs.traf.id[acidx]
        acrte = Route._routes[acid]

        # Add operational attributes of waypoint(s)
        acrte.operation_wp.extend(wpcount * [False])
        acrte.operation_duration.extend(wpcount * [[None]])
        acrte.children.extend(wpcount * [[None]])
        acrte.op_type.extend(wpcount * [None])

        if acrte.iactwp < 0:
            # Direct towards first waypoint
            acrte.direct(acidx, acrte.wpname[0])

    
    @stack.command
    def addoperationpoints(vehicleidx: 'acid', wpname: 'wpname/coords', wptype: 'wptype', duration: 'duration', *args):
        # Args are only valid for SORTIE type modification and come in this order: 
        # type, UAVnumber, lat_j, lon_j, wpname_k, alt, spd, servicetime, recoverytime
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

        elif len(args) !=9 and wptype == 'SORTIE':
            bs.scr.echo('Invalid number of operational values, argument number must be 9 for sortie type modification.\
                        Order: type, UAVnumber, lat_j, lon_j, wpname_k, alt, spd, servicetime, recoverytime')
            return
        
        elif wptype not in ['DELIVERY', 'SORTIE', 'RENDEZVOUS']:
            bs.scr.echo('Invalid operation type, please select from: DELIVERY, SORTIE OR RENDEZVOUS')

        vehicleid = bs.traf.id[vehicleidx]
        acrte = Route._routes[vehicleid]

        if len(wpname.split('/')) == 2:
            # Check whether coordinates are given. If so, look up wpname
            print(wpname, acrte.wplat[acrte.wpname.index(get_wpname(wpname, acrte))], acrte.wplon[acrte.wpname.index(get_wpname(wpname, acrte))])
            wpname = get_wpname(wpname, acrte)

        wpid = acrte.wpname.index(wpname)
        if acrte.wpflyby[wpid]:
            bs.scr.echo('TURNSPD waypoint required for operation waypoints. \
                        Turnspeed for waypoint(s) marked as delivery automatically adjusted to default value.')
        # this line for some reason stop the drone from spawning
        # Modify TURNSPD to 5 for all delivery waypoints
        acrte.wpturnspd[wpid] = 2.0
        acrte.wpflyby[wpid] = False
        acrte.wpflyturn[wpid] = True

        # modify operational attributes of the wp
        acrte.operation_wp[wpid] = True
        if acrte.operation_duration[wpid][0] is None:
            acrte.operation_duration[wpid] = [float(duration)]
        else:
            acrte.operation_duration[wpid].extend([float(duration)])
            
        if acrte.op_type[wpid] is None:
            acrte.op_type[wpid] = [wptype]
        else:
            acrte.op_type[wpid].extend([wptype])

        if wptype.upper() == 'SORTIE':
            if len(args[4].split('/')) == 2:
                # Check whether coordinates are given. If so, look up wpname. If not, None is returned
                wpname_k = get_wpname(args[4], acrte)
                lat_k = args[4].split('/')[0]
                lon_k = args[4].split('/')[1]
            else:
                wpname_k = args[3]

            wpid_k = acrte.wpname.index(wpname_k.upper())
            # truck, type, UAVnumber, lat_i, lon_i, lat_j, lon_j, lat_k, lon_k,
            # wpname_k, alt, spd, servicetime, retrievaltime
            child = bs.traf.Operations.drone_manager.add_drone(vehicleid, 
                                        args[0], args[1], acrte.wplat[wpid], 
                                        acrte.wplon[wpid],args[2], args[3], 
                                        acrte.wplat[wpid_k], 
                                        acrte.wplon[wpid_k], 
                                        wpname_k, args[5], args[6], 
                                        args[7], args[8])

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

    @stack.command(aliases=("DIRECTTO", "DIRTO"))
    @staticmethod
    def direct(acidx: 'acid', wpname: 'wpinroute'):
        """DIRECT acid wpname
        
            Go direct to specified waypoint in route (FMS)"""
        acid = bs.traf.id[acidx]
        acrte = Route._routes[acid]
        wpidx = acrte.wpname.index(wpname)

        acrte.iactwp = wpidx
        bs.traf.actwp.lat[acidx]    = acrte.wplat[wpidx]
        bs.traf.actwp.lon[acidx]    = acrte.wplon[wpidx]
        bs.traf.actwp.flyby[acidx]  = acrte.wpflyby[wpidx]
        bs.traf.actwp.flyturn[acidx] = acrte.wpflyturn[wpidx]
        bs.traf.actwp.turnrad[acidx] = acrte.wpturnrad[wpidx]
        bs.traf.actwp.turnspd[acidx] = acrte.wpturnspd[wpidx]
        bs.traf.actwp.turnhdgr[acidx] = acrte.wpturnhdgr[wpidx]

        bs.traf.actwp.nextturnlat[acidx], bs.traf.actwp.nextturnlon[acidx], \
        bs.traf.actwp.nextturnspd[acidx], bs.traf.actwp.nextturnrad[acidx], \
        bs.traf.actwp.nextturnhdgr[acidx] , bs.traf.actwp.nextturnidx[acidx]\
            = acrte.getnextturnwp()

        # Determine next turn waypoint data

        # Do calculation for VNAV
        acrte.calcfp()

        bs.traf.actwp.xtoalt[acidx] = acrte.wpxtoalt[wpidx]
        bs.traf.actwp.nextaltco[acidx] = acrte.wptoalt[wpidx]

        if np.any(np.array(acrte.wprta[acrte.iactwp:]) > 0):
            foo= np.where(np.array(acrte.wprta) > 0)[0]
            idxnextrta = int(foo[np.where(foo >= acrte.iactwp)[0]][0])
            bs.traf.actwp.torta[acidx]    = acrte.wprta[idxnextrta]  
            bs.traf.actwp.idxnextrta[acidx]  = idxnextrta
        else:
            bs.traf.actwp.torta[acidx]    = -999.0 
            bs.traf.actwp.idxnextrta[acidx]  = -999

        #VNAV calculations like V/S and speed for RTA
        bs.traf.ap.ComputeVNAV(acidx, acrte.wptoalt[wpidx], acrte.wpxtoalt[wpidx],\
                                    bs.traf.actwp.torta[acidx],
                                    bs.traf.actwp.idxnextrta[acidx])

        # If there is a speed specified, process it
        if acrte.wpspd[wpidx]>0.:
            # Set target speed for autopilot

            if acrte.wpalt[wpidx] < 0.0:
                alt = bs.traf.alt[acidx]
            else:
                alt = acrte.wpalt[wpidx]

            # Check for valid Mach or CAS
            if acrte.wpspd[wpidx] <2.0:
                cas = mach2cas(acrte.wpspd[wpidx], alt)
            else:
                cas = acrte.wpspd[wpidx]

            # Save it for next leg
            bs.traf.actwp.nextspd[acidx] = cas

        # No speed specified for next leg
        else:
            bs.traf.actwp.nextspd[acidx] = -999.


        qdr_,dist_ = geo.qdrdist(bs.traf.lat[acidx], bs.traf.lon[acidx],
                             bs.traf.actwp.lat[acidx], bs.traf.actwp.lon[acidx])

        # Save leg length & direction in actwp data
        bs.traf.actwp.curlegdir[acidx] = qdr_      #[deg]
        bs.traf.actwp.curleglen[acidx] = dist_*nm  #[m]

        if acrte.wpflyturn[wpidx] and acrte.wpturnrad[wpidx]>0.: # turn radius specified
            turnrad = acrte.wpturnrad[wpidx]
        # Overwrite is hdgrate  defined
        if acrte.wpflyturn[wpidx] and acrte.wpturnhdgr[wpidx] > 0.: # heading rate specified
            turnrad = bs.traf.tas[acidx]*360./(2*pi*acrte.wpturnhdgr[wpidx])
        else:                                                          # nothing specified, use default bank ang;e
            turnrad = bs.traf.tas[acidx]*bs.traf.tas[acidx]/math.tan(math.radians(acrte.bank)) / g0 / nm  # [nm]default bank angle e.g. 25 deg

        bs.traf.actwp.turndist[acidx] = np.logical_or( acrte.wpturnhdgr[wpidx]>0.,
                                                    bs.traf.actwp.flyby[acidx] > 0.5)  *   \
                    turnrad*abs(math.tan(0.5*math.radians(max(5., abs(degto180(qdr_ -
                    acrte.wpdirfrom[acrte.iactwp]))))))    # [nm]

        if not acrte.operation_wp[acrte.iactwp] or acrte.iactwp > 0:
            # If first point is not an operation point, turn on autopilot
            bs.traf.swlnav[acidx] = True
        else:
            # Let operation handler perform operation instead; do nothing
            bs.traf.swlnav[acidx] = False

        return True

    @stack.command(name='RTA')
    @staticmethod
    def SetRTA(acidx: 'acid', wpname: 'wpinroute', time: 'time'):  # all arguments of setRTA
        """ RTA acid, wpname, time
        
            Add RTA to waypoint record"""
        acid = bs.traf.id[acidx]
        acrte = Route._routes[acid]
        if wpname is None:
            wpidx = len(acrte.wpname)-1
        else:
            wpidx = acrte.wpname.index(wpname)
        acrte.wprta[wpidx] = time

        # Recompute route and update actwp because of RTA addition
        acrte.direct(acidx, acrte.wpname[acrte.iactwp])

        return True

    def getnextwp(self):
        """Go to next waypoint and return data"""

        if self.flag_landed_runway:

            # when landing, LNAV is switched off
            lnavon = False

            # no further waypoint
            nextqdr = -999.

            # and the aircraft just needs a fixed heading to
            # remain on the runway
            # syntax: HDG acid,hdg (deg,True)
            name = self.wpname[self.iactwp]

            # Change RW06,RWY18C,RWY24001 to resp. 06,18C,24
            if "RWY" in name:
                rwykey = name[8:10]
                if len(name)>10:
                    if not name[10].isdigit():
                        rwykey = name[8:11]
            # also if it is only RW
            else:
                rwykey = name[7:9]
                if len(name) > 9:
                    if not name[9].isdigit():
                        rwykey = name[7:10]

            # Use this code to look up runway heading
            wphdg = bs.navdb.rwythresholds[name[:4]][rwykey][2]

            # keep constant runway heading
            stack.stack("HDG " + str(self.acid) + " " + str(wphdg))

            # start decelerating
            stack.stack("DELAY " + "10 " + "SPD " + str(self.acid) + " " + "10")

            # delete aircraft
            stack.stack("DELAY " + "42 " + "DEL " + str(self.acid))

            swlastwp = (self.iactwp == self.nwp - 1)
            # print(f'printing from getnextwp: current wp is {self.iactwp}')
            return self.wplat[self.iactwp],self.wplon[self.iactwp],   \
                           self.wpalt[self.iactwp],self.wpspd[self.iactwp],   \
                           self.wpxtoalt[self.iactwp],self.wptoalt[self.iactwp], \
                           self.wpxtorta[self.iactwp], self.wptorta[self.iactwp], \
                           lnavon,self.wpflyby[self.iactwp], \
                           self.wpflyturn[self.iactwp],self.wpturnrad[self.iactwp], \
                           self.wpturnspd[self.iactwp], self.wpturnhdgr[self.iactwp], \
                           nextqdr, swlastwp, self.operation_wp[self.iactwp]

        # Switch LNAV off when last waypoint has been passed
        lnavon = self.iactwp < self.nwp -1

        # if LNAV on: increase counter
        if lnavon:
            self.iactwp += 1

        # Activate switch to indicate that this is the last waypoint (for lenient passing logic in actwp.Reached function)
        swlastwp = (self.iactwp == self.nwp-1)

        nextqdr = self.getnextqdr()

        # in case that there is a runway, the aircraft should remain on it
        # instead of deviating to the airport centre
        # When there is a destination: current = runway, next  = Dest
        # Else: current = runway and this is also the last waypoint
        if (self.wptype[self.iactwp] == 5 and
                self.wpname[self.iactwp] == self.wpname[-1]) or \
           (self.wptype[self.iactwp] == 5 and self.iactwp+1<self.nwp and
                self.wptype[self.iactwp + 1] == 3):

            self.flag_landed_runway = True

        #print ("getnextwp:",self.wpname[self.iactwp],"   torta = ",self.wptorta[self.iactwp])
        if np.any(np.array(self.wprta[self.iactwp:]) > 0):
            foo= np.where(np.array(self.wprta) > 0)[0]
            idxnextrta = int(foo[np.where(foo >= self.iactwp)[0]][0])
            torta = self.wptorta[idxnextrta]
        else:
            idxnextrta = -999
            torta = -999.0

        return self.wplat[self.iactwp],self.wplon[self.iactwp],   \
               self.wpalt[self.iactwp],self.wpspd[self.iactwp],   \
               self.wpxtoalt[self.iactwp],self.wptoalt[self.iactwp],\
               idxnextrta,torta,\
               lnavon,self.wpflyby[self.iactwp], \
               self.wpflyturn[self.iactwp], self.wpturnrad[self.iactwp], \
               self.wpturnspd[self.iactwp], self.wpturnhdgr[self.iactwp],\
               nextqdr, swlastwp, self.operation_wp[self.iactwp]

    @stack.command(name='TDRTAs')
    @staticmethod
    def TDSetRTAs(acidx: 'acid', *args):
        """Convert wp coords to wpname and call regular setRTA vanilla method"""
        if len(args)%2 != 0:
            bs.scr.echo('You missed a set RTA value, arguement number must be a multiple of 2.')
            return
        
        args = np.reshape(args, (int(len(args)/2), 2))

        for RTAdata in args:
            wpname = RTAdata[0]
            if len(wpname.split('/')) == 2:
                acid = bs.traf.id[acidx]
                acrte = Route._routes[acid]
                # Check whether coordinates are given. If so, look up wpname
                wpname = get_wpname(wpname, acrte)
            time = argparsers['time'].parse(RTAdata[1])[0]
            # Call regular function
            Route.SetRTA(acidx, wpname, time)

    @stack.command(name='TDRTA')
    @staticmethod
    def TDSetRTA(acidx: 'acid', wpname, time: 'time'):  # all arguments of setRTA:
        """Convert wp coords to wpname and call regular setRTA vanilla method"""
        
        if len(wpname.split('/')) == 2:
            acid = bs.traf.id[acidx]
            acrte = Route._routes[acid]
            # Check whether coordinates are given. If so, look up wpname
            wpname = get_wpname(wpname, acrte)
        

        # Call regular function
        Route.SetRTA(acidx, wpname, time)