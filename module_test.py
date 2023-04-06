from numpy import array,zeros
from time  import sleep

import macspos as ms
import matplotlib.pyplot as plt


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
                        v_min = 0.1, v_max = 2, d_safe = 3, \
                        t_st = zeros((3,1)), \
                        period_replan = 2, period_predhr = 1,\
                        split_interval = 0)


fig1 = plt.figure()
fig2 = plt.figure()
fig3 = plt.figure()

traj = fig1.add_subplot(1,1,1)
velc = fig2.add_subplot(1,1,1)
cptc = fig3.add_subplot(1,1,1)

while True:

    ### visualize ###
    
    traj.cla()
    velc.cla()

    endtime = max(macspos_lc.sharedmemory.t)

    # print(endtime)

    for agent in macspos_lc.sharedmemory.agents:

        wparray = zeros((2,agent.N))
        velarray = zeros((2,2*agent.N+2))

        velarray[0,0] = -1
        velarray[0,-1] = agent.t_prf[-1]+30

        for i in range(agent.N):

            wparray[0,i] = agent.waypoints[i].loc[0]
            wparray[1,i] = agent.waypoints[i].loc[1]

            velarray[0,1+2*i] = agent.t_prf[i]
            velarray[0,2+2*i] = agent.t_prf[i]

            try:
                velarray[1,2+2*i] = agent.v_prf[i]
                velarray[1,3+2*i] = agent.v_prf[i]
            except:
                # print('err')       
                pass                 

        traj.plot(wparray[0],wparray[1],'-o')

        # print(velarray)

        velc.plot(velarray[0],velarray[1],'-',linewidth = 3,label=f"Agent#{agent.id+1}")

        velc.set_xlim(0,endtime)
        velc.set_ylim(macspos_lc.sharedmemory.v_min - 1, macspos_lc.sharedmemory.v_max + 1)
        velc.hlines(macspos_lc.sharedmemory.v_min,-1,endtime+1,linestyle='--',color='black',linewidth=1)
        velc.hlines(macspos_lc.sharedmemory.v_max,-1,endtime+1,linestyle='--',color='black',linewidth=1)

        # cptc.bar()
    plt.pause(2)

    sleep(0.1)
plt.show()