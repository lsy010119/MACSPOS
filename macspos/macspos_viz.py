from threading import Thread

from macspos.lib.waypoint_handlers      import *
from macspos.lib.trajectory_handlers    import *

from time   import sleep,time

class MACSPOSViz(Thread):



    def __init__(self, sharedmemory, simparams):

        self.sharedmemory   = sharedmemory
        self.simparams      = simparams

        super().__init__()
        self.daemon = True
        self.start()

    

    def run(self):

        fig1 = plt.figure()
        fig2 = plt.figure()
        

        viz = fig1.add_subplot(1,1,1)
        
        vel = fig2.add_subplot(2,1,1)


        vel_list = zeros((3,len(self.sharedmemory.agents)))

        UAVcolors = ['blue', 'orchid', 'darkgreen', 'olive', 'teal', 'skyblue']

        plt.pause(0.01)

        while not all(self.simparams.FLAG_initialized):

            pass

        start_time = time()

        while not all(self.simparams.FLAG_mission_done): 

            curr_time = time()


            viz.cla()
            vel.cla()

            vel_list = hstack((vel_list,zeros((3,1))))

            for agent in self.sharedmemory.agents:
                
                wparray = zeros((2,len(agent.waypoints)))

                for idx,wp in enumerate(agent.waypoints):

                    wparray[:,idx] = wp.loc

                viz.plot(wparray[0],wparray[1],"-o",color=UAVcolors[agent.id])

                vel_list[agent.id,-1] = norm(agent.vel)

                # print(f"Agent.{agent.id}")
                # print(agent.pos)
                # print(agent.vel)
                # print(norm(agent.velin))
                # print(agent.v_prf)

                # viz.quiver(agent.pos[0],agent.pos[1], agent.vel[0],agent.vel[1])
                viz.scatter(agent.pos[0],agent.pos[1])

                vel.plot(range(len(vel_list[0])),vel_list[agent.id].T,color=UAVcolors[agent.id],label=f"Agent #{agent.id+1}")
                vel.hlines(self.sharedmemory.v_min,-10,10000,linestyle='--',color='black',linewidth=2)
                vel.hlines(self.sharedmemory.v_max,-10,10000,linestyle='--',color='black',linewidth=2)
                vel.set_ylim(self.sharedmemory.v_min - 0.3, self.sharedmemory.v_max + 0.3)
                vel.set_xlim(0, len(vel_list[0]))
                
                # except: pass

            viz.set_xlim(-1,16)
            viz.set_ylim(-1,10)
            viz.grid()
            viz.set_xlabel("x[m]",fontsize=15)
            viz.set_ylabel("y[m]",fontsize=15)

            vel.set_ylabel("velocity [m/s]",fontsize=15)
            vel.legend()
            vel.grid()

            plt.pause(0.00001)

            # sleep(0.00001)

        elepsed_time = curr_time - start_time

        # fig3 = plt.figure()
        # velplots = [fig3.add_subplot(3,1,1), fig3.add_subplot(3,1,2), fig3.add_subplot(3,1,3)]

        # for agent in self.sharedmemory.agents:

        #     velarray = zeros((2,2*agent.N+2))

        #     velarray[0,0] = -1
        #     velarray[0,-1] = agent.t_prf[-1]+30

        #     for i in range(agent.N):

        #         velarray[0,1+2*i] = agent.t_prf[i]
        #         velarray[0,2+2*i] = agent.t_prf[i]

        #         try:
        #             velarray[1,2+2*i] = agent.v_prf[i]
        #             velarray[1,3+2*i] = agent.v_prf[i]
        #         except: pass                 

        #     velplots[agent.id].plot(velarray[0],velarray[1],linewidth=1,linestyle="-",\
        #                         label=f"Agent #{agent.id+1} command")
            
        #     velplots[agent.id].hlines(self.sharedmemory.v_min,-10,10000,linestyle='--',color='black',linewidth=1)
        #     velplots[agent.id].hlines(self.sharedmemory.v_max,-10,10000,linestyle='--',color='black',linewidth=1)
        #     velplots[agent.id].set_ylim(self.sharedmemory.v_min - 0.3, self.sharedmemory.v_max + 0.3)
            
        #     velplots[agent.id].set_xlim(0, elepsed_time)

        #     velplots[agent.id].plot(linspace(0,elepsed_time,len(vel_list[0])),vel_list[agent.id].T,label=f"Agent #{agent.id+1} sensored")

        #     velplots[agent.id].legend()

        #     savetxt(f"vel_list_sim{agent.id}.txt",vel_list[agent.id])
        #     savetxt(f"vel_list_cmd{agent.id}.txt",velarray)

        

        plt.show()
