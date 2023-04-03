from numpy import array,zeros
from time  import sleep

import macspos as ms


wp1 = array([[0,2,4,6],
             [0,2,4,6]])

agent1 = ms.Agent(id = 0, waypoints = wp1)

wp2 = array([[6,4,2,0],
             [0,2,4,6]])

agent2 = ms.Agent(id = 1, waypoints = wp2)

agents = [agent1,agent2]

wp3 = array([[3,3,3,3],
             [6,4,2,0]])

agent3 = ms.Agent(id = 2, waypoints = wp3)

agents = [agent1,agent2,agent3]



macspos_lc = ms.MACSPOS(agents, \
                        v_min = 0.1, v_max = 0.3, d_safe = 3, \
                        t_st = zeros((3,1)), \
                        period_replan = 2, split_interval = 0)


while True:

    sleep(0.1)