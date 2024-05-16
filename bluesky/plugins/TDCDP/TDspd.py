"""Plugin to ensure VNAV stays on after receiving a SPD command, such that AC keeps tracking WPT"""

import bluesky as bs
from bluesky.core import Entity
from bluesky import stack

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name':     'SPDAP',
        'plugin_type':     'sim'
    }
    bs.traf.SPDAP = SPDAP()
    return config

class SPDAP(Entity):
    def __init__(self):
        super().__init__()

    @stack.command(name='SPDAP', aliases=("SPEEDAP"))
    def selspdapcmd(self, idx: 'acid', casmach: 'spd'):  # SPDAP command
    # def selspdapcmd(self, idx: 'acid', casmach: 'spd'):  # SPDAP command
        """ SPDAP acid, casmach (= CASkts/Mach) 
        
            Select autopilot speed. """
        # Depending on or position relative to crossover altitude,
        # we will maintain CAS or Mach when altitude changes
        # We will convert values when needed

        bs.traf.selspd[idx] = casmach

        # Used to be: Switch off VNAV: SPD command overrides
        # Modified: this ensures autopilot mode does not change when SPDAP command is given
        # bs.traf.swvnavspd[idx]   = False
        return True
        