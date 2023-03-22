from threading import Thread
from time      import sleep


class MACSPOSWC(Thread):



    def __init__(self, sharedmemory):

        self.sharedmemory = sharedmemory

        super().__init__()
        self.daemon = True
        self.start()

    

    def run(self):

        i = 0

        while True: 

            if self.sharedmemory.FLAG_run:

                self.sharedmemory.FLAG_run = False
                
                print("WC start")
                #### calculate vel profile ####
                self.sharedmemory.agents += 0.5
                
                sleep(0.5)
                
                
                print("WC done")

