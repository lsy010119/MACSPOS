from copy import deepcopy
from numpy import array, zeros, where, hstack, linspace, savetxt
from numpy.linalg import norm

from time import time,sleep

from macspos.structs.agent      import Agent
from macspos.structs.waypoint   import WayPoint

import matplotlib.pyplot as plt


def predict_posvel(agents, period_predhr, heading_wps):

    for agent in agents:

        heading_wp = heading_wps[agent.id]

        if heading_wp == 0:

            break

        wp_0 = WayPoint(agent.id, deepcopy(agent.pos) + period_predhr*deepcopy(agent.vel))

        print("waypoints bef : ")
        for wp in agent.waypoints:

            print(f"{wp.loc}")


        wp_updated = agent.waypoints[heading_wp:]

        wp_updated.insert(0,wp_0)

        agent.waypoints = wp_updated
            
        agent.updateData()

        print("waypoints aft : ")
        for wp in agent.waypoints:

            print(f"{wp.loc}")
        

    # for agent in agents:
    #     print(f"=== Agent.{agent.id} ===")
    #     print(f"heading {heading_wps[agent.id]}...")

    print(agent)



def calc_velin(agents, TIME_startctrl, simparams):

    t_curr = time()

    heading_wps = [1]*len(agents)

    for agent in agents:

        t_set = agent.t_prf

        if simparams.FLAG_mission_done[agent.id]:

            agent.velin = zeros(2)

        else:

            try:

                heading_wp = where(t_set > t_curr - TIME_startctrl)[0][0]

                # print("======================")
                # print("t_set : \n",t_set)
                # print("elepsed time : \n",t_curr - TIME_startctrl)
                # print("heading waypoints : \n",heading_wp)

                heading_wps[agent.id] = heading_wp

                v_des = agent.v_prf[heading_wp-1]

                agent.velin = v_des*(agent.waypoints[heading_wp].loc - agent.pos)/norm(agent.waypoints[heading_wp].loc - agent.pos)

            except:

                heading_wps[agent.id] = 0

                agent.velin = zeros(2)

                simparams.FLAG_mission_done[agent.id] = True

        # print(heading_wp)
        # print(t_set, t_curr - TIME_startctrl)


    return heading_wps
