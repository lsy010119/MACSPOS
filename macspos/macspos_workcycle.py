from threading import Thread
import time

from macspos.lib.admm import ADMM
from macspos.lib.waypoint_handlers  import *

class MACSPOSWC(Thread):



    def __init__(self, sharedmemory):

        self.sharedmemory = sharedmemory

        self.admm = ADMM(self.sharedmemory)

        super().__init__()
        self.daemon = True
        self.start()

    

    def run(self):


        while True: 

            if self.sharedmemory.FLAG_run:

                self.sharedmemory.FLAG_run = False
                
                print("WC start")
                
                start = time.time()

                #### calculate vel profile ####
                self.admm.run()                

                end = time.time()

                print(end-start)

                self.sharedmemory.update()

                print("WC done")


