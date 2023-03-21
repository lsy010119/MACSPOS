from macspos.structs.waypoint import WayPoint


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
        self.waypoints   = self._array2WPlist(waypoints)
        self.N           = len(self.waypoints)
        self.lengths     = []


        self.pos         = []
        self.vel         = []
    

    def _array2WPlist(self, waypoints):

        N = len(waypoints.T)

        wplist = [0]*N

        for i in range(N):

            wp = WayPoint( id = self.id, loc = [waypoints[0,i],waypoints[1,i]])

            wplist[i] = wp

        return wplist


    def _calLengths(self):

        pass


    def _reCountWayPoints(self):
    
        self.N = len(self.waypoints)
        
        for i in range(self.N):

            self.waypoints[i].idx = i

    
    def updateData(self):

        self._reCountWayPoints()