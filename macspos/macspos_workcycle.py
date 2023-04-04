from threading import Thread
from numpy     import arange

from macspos.lib.admm import ADMM
from macspos.lib.waypoint_handlers  import *

import time

class MACSPOSWC(Thread):



    def __init__(self, sharedmemory):

        self.sharedmemory = sharedmemory

        self.admm = ADMM(self.sharedmemory)

        super().__init__()
        self.daemon = True
        self.start()

    

    def run(self):

        fig1 = plt.figure()
        fig2 = plt.figure()
        fig3 = plt.figure()

        traj = fig1.add_subplot(1,1,1)
        velc = fig2.add_subplot(1,1,1)
        cptc = fig3.add_subplot(1,1,1)

        while True: 

            if self.sharedmemory.FLAG_runadmm:

                self.sharedmemory.FLAG_runadmm = False
                
                print("WC start")
                
                start = time.time()

                #### calculate vel profile ####
                self.admm.run()                

                end = time.time()

                print(f"ADMM Runtime : {end-start} sec")

                ### visualize ###
                '''
                traj.cla()
                velc.cla()

                for agent in self.sharedmemory.agents:

                    wparray = zeros((2,agent.N))
                    velarray = zeros((2,2*agent.N+2))

                    velarray[0,0] = -1
                    velarray[0,-1] = agent.t_prf[-1]+1

                    for i in range(agent.N):

                        wparray[0,i] = agent.waypoints[i].loc[0]
                        wparray[1,i] = agent.waypoints[i].loc[1]

                        velarray[0,1+2*i] = agent.t_prf[i]
                        velarray[0,2+2*i] = agent.t_prf[i]

                        try:
                            velarray[1,2+2*i] = agent.v_prf[i]
                            velarray[1,3+2*i] = agent.v_prf[i]
                        except:
                            print('err')                        

                    traj.plot(wparray[0],wparray[1],'-o')

                    print(velarray)

                    velc.plot(velarray[0],velarray[1],'-')

                    # cptc.bar()

                self.sharedmemory.update()


                plt.pause(0.001)
                '''
                print("WC done")





        plt.show()
