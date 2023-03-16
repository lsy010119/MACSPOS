

class WayPoint:



    def __init__(self, id, loc, is_cp = False, collide_with = []):
        """
        ## WayPoint

        ### members

            id           : int             = id of the agent with the waypoint
            idx          : int             = index of the waypoint
            loc          : int[2]          = location of the waypoint
            is_cp        : bool            = is the waypoint is collision point
            collide_with : int             = id of the agent collides with
        """

        self.id = id
        self.idx = -1
        self.loc = loc
        self.is_cp = is_cp
        self.collide_with = collide_with