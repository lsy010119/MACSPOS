from numpy import array, ones, zeros, eye, block

from macspos.lib.waypoint_handlers  import insert_cp


class SharedMemory:


    def __init__(self, agents, v_min, v_max, d_safe, t_st, period_replan, period_predhr ,split_interval ):        

        self.period_replan      = period_replan
        self.period_predhr      = period_predhr
        self.split_interval     = split_interval

        ### Agents ###
        self.agents             = agents                    # a list of Agents

        ### Agent number ###
        self.K                  = len(agents)               # the number of the agents

        ### Waypoint number ###
        self.N                  = -1                        # sum of the nuber of waypoints
        self.N_max              = -1                        # the maximum of the nummber of waypoints
        self.N_c                = -1                        # the number of the collision points

        ### Constraints ###
        self.v_min              = v_min                     # minimum velocity
        self.v_max              = v_max                     # maximum velocity
        self.d_safe             = d_safe                    # safety distance
        self.t_safe             = self.d_safe/self.v_max    # safety time difference
        
        ### Constants ###
        self.cps                = -1                        # collision points
        self.t_st               = t_st                      # starting time
        
        ### ADMM Results ###
        self.cp_time_residual   = -1

        ### Flags ###
        self.FLAG_runadmm       = False
        self.FLAG_cntrlin        = False

        self.update()

    
    def update(self):

        ### update cps ###

        self.cps = insert_cp(self.agents)

        self.cp_time_residual = [0]*len(self.cps)

        self.N_c = len(self.cps)

        ### update N ###
        
        self.N = 0
        self.N_max = 0

        for agent in self.agents:

            agent.updateData()

            self.N += agent.N

            if agent.N > self.N_max: self.N_max = agent.N


        