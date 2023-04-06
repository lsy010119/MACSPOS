from threading import Thread
from time      import sleep,time

from macspos.shared_memory      import SharedMemory
from macspos.macspos_workcycle  import MACSPOSWC
from macspos.lib.admm           import ADMM


class MACSPOS(Thread):



    def __init__(self, agents, v_min, v_max, d_safe, t_st, period_replan, period_predhr, split_interval):

        self.sharedmemory = SharedMemory(agents, v_min, v_max, d_safe, t_st, period_replan, period_predhr, split_interval)
        self.admm         = ADMM(self.sharedmemory)
        self.workcycle    = MACSPOSWC(self.sharedmemory,self.admm)
                
        super().__init__()
        self.daemon = True
        self.start()
    

    def run(self):


        while True: 

            print("macspos")

            t1 = time()

            self.sharedmemory.FLAG_runadmm = True

            sleep(self.sharedmemory.period_predhr)

            t2 = time()

            self.sharedmemory.FLAG_ctrlin = True

            self.sharedmemory.TIME_startctrl = time()

            sleep(self.sharedmemory.period_replan-self.sharedmemory.period_predhr)

            t3 = time()

            # print("planning",t2-t1)
            # print("ctrl to admm",t3-t2)