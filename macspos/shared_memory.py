from numpy import array, ones, zeros, eye, block

from macspos.lib.waypoint_handlers  import insert_cp


class SharedMemory:


    def __init__(self, params, agents):
        
        # self.period_replan  = params["PERIOD_REPLANNING"]
        # self.split_interval = params["SPLIT_INTERVAL"]

        self.period_replan  = 2
        self.split_interval = 0

        ### Agents ###
        self.agents         = agents           # a list of Agents

        ### Agent number ###
        self.K              = len(agents)                # the number of the agents

        ### Waypoint number ###
        self.N              = -1                # sum of the nuber of waypoints
        self.N_max          = -1            # the maximum of the nummber of waypoints
        self.N_c            = -1              # the number of the collision points

        ### Constraints ###
        self.v_min          = -1            # minimum velocity
        self.v_max          = -1            # maximum velocity
        self.d_safe         = -1           # safety distance
        self.t_safe         = -1           # safety time difference
        
        ### Constants ###
        self.cps            = -1              # collision points
        self.t_st           = zeros((self.K,1))             # starting time

        ### Flags ###
        self.FLAG_run       = False

        self.update()

    
    def update(self):

        ### update cps ###

        self.cps = insert_cp(self.agents)

        self.N_c = len(self.cps)

        ### update N ###
        
        self.N = 0
        self.N_max = 0

        for agent in self.agents:

            agent.updateData()

            self.N += agent.N

            if agent.N > self.N_max: self.N_max = agent.N


        