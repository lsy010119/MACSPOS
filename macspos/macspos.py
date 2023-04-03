from threading import Thread
from time      import sleep

from macspos.shared_memory      import SharedMemory
from macspos.macspos_workcycle  import MACSPOSWC


class MACSPOS(Thread):



    def __init__(self, agents, v_min, v_max, d_safe, t_st, period_replan, split_interval):


        self.sharedmemory = SharedMemory(agents, v_min, v_max, d_safe, t_st, period_replan, split_interval)
        workcycle         = MACSPOSWC(self.sharedmemory)


        super().__init__()
                
        self.daemon = True
        
        self.start()

    

    def run(self):


        while True: 

            print("macspos")

            self.sharedmemory.FLAG_run = True

            # sleep(60)
            sleep(self.sharedmemory.period_replan)