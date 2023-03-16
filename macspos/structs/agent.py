


class Agent:
    


    def __init__(self, id, waypoints):
        """
        ## Agent
        
        ### members

            id        : int             = id of the agent
            waypoints : WayPoint[*]     = waypoints given to the agent
            N         : int             = the number of the waypoints
            lengths   : double[*]       = length of the segments
            pos       : array(3,1)      = current position of the agent
            vel       : array(3,1)      = current velocity of the agent

        ### methods


        """

        self.id          = id
        self.waypoints   = waypoints
        self.N           = len(self.waypoints)
        self.lengths     = []


        self.pos         = []
        self.vel         = []
    

    def _calLengths(self):

        pass


    def _reCountWayPoints(self):
    
        self.N = len(self.waypoints)
        
        for i in range(self.N):

            self.waypoints[i].idx = i

    
    def updateData(self):

        self._reCountWayPoints()