import threading
import asyncio
import time
from   numpy                import array, zeros
from   mavsdk               import System

from lib_sitl.connector     import Connector
from lib_sitl.telemetry     import Telemetry
from lib_sitl.controller    import Controller
from lib_sitl.agent_sitl    import AgentSITL

import macspos  as ms


class PublicParams:

    def __init__(self,spawn_loc,N):

        self.FLAG_connected     = [False]*N
        self.FLAG_mission_done  = [False]*N
        self.FLAG_initialized   = [False]*N

        self.spawn_loc          = spawn_loc



class FCRXTX(threading.Thread):

    def __init__(self, agents_sitl, macspos, simparams):
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
        self.simparams      = simparams


        reciever_connector = Connector(agents_sitl, macspos, self.simparams)
        telemetry          = Telemetry(agents_sitl, macspos, self.simparams)
        controller         = Controller(agents_sitl, macspos, self.simparams)
        

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

    
    def __init__(self, wps, v_min, v_max, d_s, t_st, period_replan, period_predhr, spawn_loc):
        
        self.agents_macspos = []
        self.agents_sitl = []

        for id, wp in enumerate(wps):
            
            self.agents_macspos.append(ms.Agent(id,wp))
            self.agents_sitl.append(AgentSITL(id))

        self.simparams  = PublicParams(spawn_loc,len(self.agents_sitl))

        self.macspos    = ms.MACSPOS(self.agents_macspos, v_min, v_max, d_s, t_st,\
                                     period_replan, period_predhr, 0, self.simparams)

        self.gzsim      = FCRXTX(self.agents_sitl,self.macspos,self.simparams)










if __name__ == "__main__":

    # wp1 = array([[-6,-4,-2,0,2,4,6],
    #             [3,2,1,0,-1,-2,-3]])

    # wp2 = array([[6,4,2,0,-2,-4,-6],
    #             [3,2,1,0,-1,-2,-3]])

    # wp3 = array([[0,0,0,0,0,0,0],
    #             [-6,-4,-2,0,2,4,6]])

    # wp1 = array([[0],
    #              [5]])
    # wp2 = array([[-5],
    #              [0]])
    # wp3 = array([[0],
    #              [-5]])

    wp1 = array([[15,0], [8,8], [6,5], [0,0]]).T
    wp2 = array([ [15,5], [9,3], [4,1], [0,5]]).T
    wp3 = array([ [15,3], [6,0], [5,6], [0,9]]).T


    wps             = [wp1,wp2,wp3]
    v_min           = 0.5
    v_max           = 1.0
    d_s             = 4
    t_st            = zeros((3,1))
    period_replan   = 5
    period_predhr   = 0.1

    spawn_loc       = [array([3,0]),array([6,0]),array([9,0])]

    gzsitl = GZSITL(wps, v_min, v_max, d_s, t_st, period_replan, period_predhr, spawn_loc)