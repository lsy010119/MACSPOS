from copy import deepcopy
from numpy import array, zeros, where
from numpy.linalg import norm

from time import time,sleep

from macspos.structs.agent      import Agent
from macspos.structs.waypoint   import WayPoint

import matplotlib.pyplot as plt


def predict_posvel(agents, period_predhr, heading_wps):

    for agent in agents:

        heading_wp = heading_wps[agent.id]

        wp_0 = WayPoint(agent.id, deepcopy(agent.pos) + period_predhr*deepcopy(agent.vel))

        wp_updated = deepcopy(agent.waypoints[heading_wp:])

        wp_updated.insert(0,wp_0)

        agent.waypoints = wp_updated
            
        agent.updateData()

        
    print(f"=== Agent.{agents[2]} ===")
    print(f"heading {heading_wps[agents[2].id]}...")
    print("waypoints : ")

    for wp in wp_updated:

        print(f"{wp.loc}")


def calc_velin(agents, TIME_startctrl):

    t_curr = time()

    heading_wps = [1]*len(agents)

    for agent in agents:

        t_set = agent.t_prf

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

            heading_wps[agent.id] = len(agent.waypoints)-1

            agent.velin = zeros(3)    

        # print(heading_wp)
        # print(t_set, t_curr - TIME_startctrl)


    return heading_wps
