from turtle import width
from numpy import array,zeros,arange
from time  import sleep

import macspos as ms
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib.axes import Axes
from matplotlib.animation import FuncAnimation

wp1 = array([[-6,-4,-2,0,2,4,6],
             [3,2,1,0,-1,-2,-3]])

# wp1 = array([[0,0], [6,5], [8,8], [15,0]]).T

agent1 = ms.Agent(id = 0, waypoints = wp1)

wp2 = array([[6,4,2,0,-2,-4,-6],
             [3,2,1,0,-1,-2,-3]])
# wp2 = array([ [15,5], [9,3], [4,1], [0,5]]).T

agent2 = ms.Agent(id = 1, waypoints = wp2)

agents = [agent1,agent2]

wp3 = array([[0,0,0,0,0,0,0],
             [-6,-4,-2,0,2,4,6]])
# wp3 = array([ [15,3], [6,0], [5,6], [0,9]]).T

agent3 = ms.Agent(id = 2, waypoints = wp3)

agents = [agent1,agent2,agent3]



macspos_lc = ms.MACSPOS(agents, \
                        v_min = 0.8, v_max = 1.2, d_safe = 3, \
                        t_st = zeros((3,1)), \
                        period_replan = 2, period_predhr = 2,\
                        split_interval = 0)


fig1 = plt.figure(figsize=(15,10))
fig2 = plt.figure(figsize=(15,15))
fig3 = plt.figure(figsize=(15,15))
fig4 = plt.figure()

traj = fig1.add_subplot(1,1,1)
velc = fig2.add_subplot(2,1,1)
cptc = fig3.add_subplot(2,1,1)
traja = fig4.add_subplot(1,1,1)

delt = 0.1


xlim = [-8,8]
ylim = [-6.5,6.5]

while True:

    ### visualize ###
    
    UAVcolors = ['blue', 'orchid', 'darkgreen', 'olive', 'teal', 'skyblue']

    trajecs = []
    traj.cla()
    velc.cla()
    cptc.cla()

    try:

        endtime = max(macspos_lc.sharedmemory.t)
    
    except: continue

    # print(endtime)

    for agent in macspos_lc.sharedmemory.agents:

        trajecs.append(agent.calculate_trajec(delt))

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
            except: pass                 

        traj.plot(wparray[0],wparray[1],'-',color=UAVcolors[agent.id],linewidth=3,\
                  label=f"Agent #{agent.id+1}")
        

        traj.scatter(wparray[0,1:-1],wparray[1,1:-1],\
                    c=UAVcolors[agent.id],\
                    s= 150)

        traj.scatter(agent.waypoints[0].loc[0],agent.waypoints[0].loc[1],\
                    facecolors='none', edgecolors=UAVcolors[agent.id],\
                    s= 200,linewidth=3)

        traj.scatter(agent.waypoints[-1].loc[0],agent.waypoints[-1].loc[1],\
                    c=UAVcolors[agent.id],\
                    marker='x',\
                    s= 200,linewidth=3)


        velc.plot(velarray[0],velarray[1],color=UAVcolors[agent.id],linewidth=4,\
                  label=f"Agent #{agent.id+1}")


    cp_pairs = []
    cp_residual = []

    for idx,cp in enumerate(macspos_lc.sharedmemory.cp_time_residual):

        agent_i = macspos_lc.sharedmemory.agents[cp[0]]
        agent_j = macspos_lc.sharedmemory.agents[cp[1]]

        cp_i = agent_i.waypoints[cp[2]].loc
        cp_j = agent_j.waypoints[cp[3]].loc
        
        t_i  = cp[4]
        t_j  = cp[5]

        traj.scatter(cp_i[0],cp_i[1],\
                    c="red",\
                    marker='^',\
                    s= 300,zorder=10)

        cp_pairs.append("#%d & #%d"%(cp[0]+1,cp[1]+1))
        cp_residual.append(abs(t_i[0] - t_j[0]))

    cptc.bar(arange(len(macspos_lc.sharedmemory.cp_time_residual)),cp_residual,\
             width=0.5,color="grey")

    cptc.set_xticks(arange(len(macspos_lc.sharedmemory.cp_time_residual)))
    cptc.set_xticklabels(cp_pairs,fontsize=15)

    cptc.hlines(macspos_lc.sharedmemory.t_safe,-1,len(macspos_lc.sharedmemory.cp_time_residual)+1,linestyle='--',color='red',linewidth=3)

    cptc.set_xlim(-0.5,len(macspos_lc.sharedmemory.cp_time_residual)-0.5)
    cptc.set_ylim(0,20)

    cptc.grid()

    traj.scatter(-100,-100,\
                facecolors='none', edgecolors="black",\
                s= 200,linewidth=3,\
                label="starting point")

    traj.scatter(-100,-100,\
                c="black",\
                marker='x',\
                s= 200,linewidth=3,\
                label="destination point")


    velc.hlines(macspos_lc.sharedmemory.v_min,-1,endtime+1,linestyle='--',color='black',linewidth=2)
    velc.hlines(macspos_lc.sharedmemory.v_max,-1,endtime+1,linestyle='--',color='black',linewidth=2)


    traj.legend(loc="best",prop={'size':15})
    # traj.axis("equal")
    traj.grid()

    traj.set_xlim(xlim[0],xlim[1])
    traj.set_ylim(ylim[0],ylim[1])

    traj.set_xlabel("x[m]",fontsize=20)
    traj.set_ylabel("y[m]",fontsize=20)
    traj.tick_params(axis='both', which='major', labelsize=20)

    velc.legend(loc="best",prop={'size':15})
    velc.grid()

    velc.set_xlim(0,endtime)
    velc.set_ylim(macspos_lc.sharedmemory.v_min - 0.3, macspos_lc.sharedmemory.v_max + 0.3)

    velc.set_xlabel("time[s]",fontsize=20)
    velc.set_ylabel("velocity[m/s]",fontsize=20)
    velc.tick_params(axis='both', which='major', labelsize=20)

    cptc.tick_params(axis='both', which='major', labelsize=20)
    cptc.set_ylabel("time difference [s]",fontsize=20)

    fig1.savefig("figs/traj")
    fig2.savefig("figs/vel")
    fig3.savefig("figs/cps")

    traj.cla()
    velc.cla()
    cptc.cla()

    ''' Trajectory Plot '''

    # traj.set_title(r"$\bf figure 1$  UAV trajectory")
    traja.set_xlabel(r"$\bf x(m)$",fontsize=10)
    traja.set_ylabel(r"$\bf y(m)$",fontsize=10)

    traja.set_xlim(xlim[0],xlim[1])
    traja.set_ylim(ylim[0],ylim[1])
    traja.grid()
    traja.axis('equal')


    for i,trajec in enumerate(trajecs):
        
        color = UAVcolors[i]

        traja.plot(trajec[0],trajec[1],'--',label=r'$UAV^{(%d)}$'%(i+1),color = color)

        agent_i = macspos_lc.sharedmemory.agents[i]

        for wp in agent_i.waypoints:
            
            traja.plot(wp.loc[0],wp.loc[1],'o',color = color)


    uav_position = traja.plot([],[],'k*',markersize=10)[0]
    

    def update(timestep):

        traj_x = []
        traj_y = []

        for i,trajec in enumerate(trajecs):
    
            traj_x.append(trajec[0,timestep])
            traj_y.append(trajec[1,timestep])

        uav_position.set_data(traj_x,traj_y)
    
    traja.legend()

    sim = FuncAnimation(fig=fig4, func=update, frames=int(endtime/delt), interval=0.1) 
    sim.save('figs/manuever.gif', fps=30, dpi=100)


    break

# plt.show()