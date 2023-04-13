from numpy import array, zeros, arange, vstack, hstack
from numpy.linalg import norm
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
        self.N           = -1
        self.lengths     = -1

        self.t_prf       = -1
        self.v_prf       = -1

        self.pos         = zeros(2)
        self.vel         = zeros(2)

        self.velin       = zeros(2)

        self.updateData()    


    def _array2WPlist(self, waypoints):

        N = len(waypoints.T)

        wplist = [0]*N

        for i in range(N):

            wp = WayPoint( id = self.id, loc = array([waypoints[0,i],waypoints[1,i]]))

            wplist[i] = wp

        return wplist


    def _calLengths(self):

        self.lengths = zeros((self.N-1,1))

        for i in range(self.N-1):

            self.lengths[i] = norm(self.waypoints[i].loc - self.waypoints[i+1].loc)


    def _reCountWayPoints(self):
    
        self.N = len(self.waypoints)
        
        for i in range(self.N):

            self.waypoints[i].idx = i

    
    def updateData(self):

        self._reCountWayPoints()
        self._calLengths()


    def calculate_trajec(self, delt):

        traj = zeros((2,1))

        del_t = self.t_prf[1:] - self.t_prf[:-1]

        for k in range(self.N - 1):

            wp_k1 = self.waypoints[k].loc
            wp_k2 = self.waypoints[k+1].loc

            del_xk = ( wp_k2[0] - wp_k1[0] ) / ( del_t[k] / delt )
            del_yk = ( wp_k2[1] - wp_k1[1] ) / ( del_t[k] / delt )


            if del_xk != 0 and del_yk == 0:     # for case moving horizontally

                traj_xk = arange( wp_k1[0], wp_k2[0], del_xk ) 
                traj_yk = zeros(len(traj_xk)) + wp_k1[1]


            elif del_xk == 0 and del_yk != 0:   # for case moving vertically

                traj_yk = arange( wp_k1[1], wp_k2[1], del_yk ) 
                traj_xk = zeros(len(traj_yk)) + wp_k1[0]


            else:                               # for other cases
                traj_xk = arange( wp_k1[0], wp_k2[0], del_xk ) 
                traj_yk = arange( wp_k1[1], wp_k2[1], del_yk ) 


            traj_k = vstack(( traj_xk, traj_yk ))

            traj = hstack((traj,traj_k))

        traj = traj[:,1:]

        return traj