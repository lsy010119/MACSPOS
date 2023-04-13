from threading import Thread

from macspos.lib.waypoint_handlers      import *
from macspos.lib.trajectory_handlers    import *

from time   import sleep,time


class MACSPOSWC(Thread):


    def __init__(self, sharedmemory, admm, simparams):


        self.sharedmemory   = sharedmemory
        self.admm           = admm
        self.simparams      = simparams

        super().__init__()
        self.daemon = True
        self.start()
    

    def run(self):

        # fig = plt.figure()
        # viz = fig.add_subplot(1,1,1)
        heading_wps = [1]*len(self.sharedmemory.agents)

        while True: 

            if self.sharedmemory.FLAG_runadmm:

                self.sharedmemory.FLAG_runadmm = False
                
                # print("WC start")

                ### prediction ###
                predict_posvel(self.sharedmemory.agents, self.sharedmemory.period_predhr, heading_wps)

                self.sharedmemory.update()
                # start = time()                    

                #### calculate vel profile ####
                self.admm.run()                

                # end = time()

                # print(f"ADMM Runtime : {end-start} sec")



                # print("WC done")

            if self.sharedmemory.FLAG_ctrlin and all(self.simparams.FLAG_initialized):

                ### control for replanning period ###
                TIME_curr = time()

                heading_wps = calc_velin(self.sharedmemory.agents, self.sharedmemory.TIME_startctrl)

                # print(v_in)
                if TIME_curr - self.sharedmemory.TIME_startctrl >= self.sharedmemory.period_replan:

                    self.sharedmemory.FLAG_ctrlin = False
                
                # print("ctrl : ",TIME_curr - self.sharedmemory.TIME_startctrl)

                sleep(0.0001)
