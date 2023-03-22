from numpy import array
from time  import sleep

import macspos as ms





wp1 = array([[1,2,3,4,5,6],
             [2,3,4,5,6,7]])

agent1 = ms.Agent(id = 0, waypoints = wp1)

macspos_lc = ms.MACSPOS(1,1)


i = 0

while i < 100: 

    i += 1

    sleep(1)

    print("main : ",macspos_lc.sharedmemory.agents)