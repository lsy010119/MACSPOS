from threading import Thread
from time      import sleep

from macspos.shared_memory      import SharedMemory
from macspos.macspos_workcycle  import MACSPOSWC


class MACSPOS(Thread):



    def __init__(self, params, agents):


        self.sharedmemory = SharedMemory(params,agents)
        workcycle         = MACSPOSWC(self.sharedmemory)


        super().__init__()
                
        self.daemon = True
        
        self.start()

    

    def run(self):


        while True: 

            print("macspos")

            self.sharedmemory.FLAG_run = True

            sleep(10)
            # sleep(self.sharedmemory.period_replan)