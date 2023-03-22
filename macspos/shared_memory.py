



class SharedMemory:


    def __init__(self, params, agents):
        
        # self.period_replan  = params["PERIOD_REPLANNING"]
        # self.split_interval = params["SPLIT_INTERVAL"]
        
        self.agents         = agents

        self.FLAG_run       = False
        