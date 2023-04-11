from threading import Thread

from macspos.lib.waypoint_handlers      import *
from macspos.lib.trajectory_handlers    import *

from time   import sleep,time

class MACSPOSWC(Thread):



    def __init__(self, sharedmemory, admm):


        self.sharedmemory   = sharedmemory
        self.admm           = admm

        super().__init__()
        self.daemon = True
        self.start()

    

    def run(self):

        while True: 

            if self.sharedmemory.FLAG_runadmm:

                self.sharedmemory.FLAG_runadmm = False
                
                # print("WC start")

                ### prediction ###
                
                start = time()

                #### calculate vel profile ####
                self.admm.run()                

                end = time()

                # print(f"ADMM Runtime : {end-start} sec")

                predict_posvel(self.sharedmemory.agents,self.sharedmemory.period_replan, self.sharedmemory.period_predhr)

                # self.sharedmemory.update()


                # print("WC done")

            if self.sharedmemory.FLAG_ctrlin:

                ### control for replanning period ###
                TIME_curr = time()

                # print("ctrl")
                v_in = calc_velin(self.sharedmemory.agents, self.sharedmemory.TIME_startctrl)

                # print(v_in)
                # if TIME_curr - self.sharedmemory.TIME_startctrl >= self.sharedmemory.period_replan:

                #     self.sharedmemory.FLAG_ctrlin = False


                sleep(0.1)
