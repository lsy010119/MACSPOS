import threading
import asyncio
import time
import numpy                as np
from   mavsdk               import System

from lib_sitl.connector     import Connector
from lib_sitl.telemetry     import Telemetry
from lib_sitl.controller    import Controller
from lib_sitl.agent_sitl    import AgentSITL

import macspos  as ms


class FLAGS:

    def __init__(self):

        self.FLAG_all_connected = False



class FCRXTX(threading.Thread):

    def __init__(self, agents_sitl, macspos, flags):
        '''
        FCReceiver

            updating the telemetry data of agents in SITL for running MACSPOS

            connector
            - MAVLINK connection via MAVSDK API server

            reciever
            - update telemetry data 

        '''
        super().__init__()

        self.agents_sitl    = agents_sitl
        self.macspos        = macspos
        self.flags          = flags


        reciever_connector = Connector(agents_sitl, macspos, self.flags)
        telemetry          = Telemetry(agents_sitl, macspos, self.flags)
        controller         = Controller(agents_sitl, macspos, self.flags)
        

        ### Generate an event loop ###         
        control_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(control_loop)


        ### Connect to Simulator ###
        control_loop.create_task(reciever_connector.connect_all(control_loop))


        ### Activating Telemetry ###
        telems = []

        for agent in self.agents_sitl:

            telems.append(control_loop.create_task(telemetry.telem_posvelNED(agent)))


        ### Activating Controller ###
        controllers = []

        for agent in self.agents_sitl:

            control_loop.create_task(controller.control_agent(agent))


        ### Run connector & telemetry Asynchronousely ###
        control_loop.run_until_complete(asyncio.gather(*telems))




class GZSITL:

    
    def __init__(self, wps, v_min, v_max, d_s, t_st, period_replan, period_predhr):
        
        self.agents_macspos = []

        for id, wp in enumerate(wps):   self.agents_macspos.append(ms.Agent(id,wp))

        self.macspos = ms.MACSPOS(self.agents_macspos, v_min, v_max, d_s, t_st,\
                                  period_replan, period_predhr, 0)

        FCR = FCReceiver(0,0)

    # def 



if __name__ == "__main__":

    flags = FLAGS()

    agent1 = AgentSITL(0)
    agent2 = AgentSITL(1)
    agent3 = AgentSITL(2)
    
    agents_sitl = [agent1,agent2,agent3]

    gz = FCRXTX(agents_sitl,0,flags)