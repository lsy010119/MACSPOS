import enum
from threading import Thread

from macspos.lib.waypoint_handlers      import *
from macspos.lib.trajectory_handlers    import *

from time   import sleep,time

class MACSPOSViz(Thread):



    def __init__(self, sharedmemory):


        self.sharedmemory   = sharedmemory

        super().__init__()
        self.daemon = True
        self.start()

    

    def run(self):

        fig = plt.figure()
        viz = fig.add_subplot(1,1,1)


        UAVcolors = ['blue', 'orchid', 'darkgreen', 'olive', 'teal', 'skyblue']


        while True: 

            viz.cla()

            for agent in self.sharedmemory.agents:
                
                wparray = zeros((2,len(agent.waypoints)))

                for idx,wp in enumerate(agent.waypoints):

                    wparray[:,idx] = wp.loc

                viz.plot(wparray[0],wparray[1],"-o",color=UAVcolors[agent.id])

                

                # print(f"Agent.{agent.id}")
                # print(agent.pos)
                # print(agent.vel)
                # print(norm(agent.velin))
                # print(agent.v_prf)

                # viz.quiver(agent.pos[0],agent.pos[1], agent.vel[0],agent.vel[1])
                viz.scatter(agent.pos[0],agent.pos[1])

                # except: pass

            viz.set_xlim(-1,16)
            viz.set_ylim(-1,12)

            plt.pause(0.001)

            sleep(0.00001)

