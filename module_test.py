from numpy import array
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



macspos_lc = ms.MACSPOS(0,agents)


while True:

    sleep(0.1)