from numpy import array, zeros, where
from numpy.linalg import norm

from time import time,sleep

from macspos.structs.agent      import Agent
from macspos.structs.waypoint   import WayPoint

import matplotlib.pyplot as plt


def predict_posvel(agents, period_replan, period_predhr):

    for agent in agents:

        ### count passed wp ###
        t_set = agent.t_prf

        passing_segment = where(t_set > period_replan - period_predhr)[0][0] - 1



def check_passedwp(agents):

    pass


def calc_velin(agents, TIME_startctrl):

    v_in = [0]*len(agents)

    t_curr = time()

    for agent in agents:

        t_set = agent.t_prf

        try:

            heading_wp = where(t_set > t_curr - TIME_startctrl)[0][0]

        except:
            heading_wp = 1

        # print(heading_wp)
        # print(t_set, t_curr - TIME_startctrl)

        v_des = agent.v_prf[heading_wp-1]
        v_in[agent.id] = v_des*(agent.waypoints[heading_wp].loc - agent.pos)/norm(agent.waypoints[heading_wp].loc - agent.pos)

    return v_in