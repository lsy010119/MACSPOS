from numpy import array, zeros
from copy   import deepcopy
from macspos.structs.agent      import Agent
from macspos.structs.waypoint   import WayPoint


def _check_cp(wp_11, wp_12, wp_21, wp_22):
    """
    ### check cp
    check the collision points between two segments

    input   :   WayPoint wp_11, WayPoint wp_12, WayPoint wp_21, WayPoint wp_22
    output  :   int cases, ndarray loc
    
    """


    ### waypoint coordinates ###
    x_11, y_11 = wp_11.loc[0], wp_11.loc[1]
    x_12, y_12 = wp_12.loc[0], wp_12.loc[1]
    
    x_21, y_21 = wp_21.loc[0], wp_21.loc[1]
    x_22, y_22 = wp_22.loc[0], wp_22.loc[1]


    if (y_22 - y_21)*(x_12 - x_11) - (x_22 -x_21)*(y_12 - y_11) == 0: 
        
        ''' Case#0 : No intersection (Segments are Parallel) '''

        cases = 0
        loc = []

        return cases, loc # no intersections
    
    else:

        t = ( (x_22 - x_21)*(y_11- y_21) - (y_22 - y_21)*(x_11 - x_21) ) / ( (y_22 - y_21)*(x_12 - x_11) - (x_22 -x_21)*(y_12 - y_11) )
        s = ( (x_12 - x_11)*(y_11- y_21) - (y_12 - y_11)*(x_11 - x_21) ) / ( (y_22 - y_21)*(x_12 - x_11) - (x_22 -x_21)*(y_12 - y_11) )
        
        if ( t < 0 or 1 < t ) or ( s < 0 or 1 < s ):

            ''' Case#0 : No intersection (Intersection locates outer of the segment) '''

            cases = 0
            loc = []

            return cases, loc # no intersections


        elif ( 0 < t < 1 ) and ( 0 < s < 1 ):

            ''' Case#1 : Intersection on the segment '''

            cases = 1

            loc = zeros(2)

            loc[0] = x_11 + t*(x_12 -x_11)
            loc[1] = y_11 + t*(y_12 -y_11)

            return cases, loc


        elif ( t == 0 ) and ( 0 < s < 1 ):

            ''' Case#2 : Intersection is P_11 '''

            cases = 2

            loc = zeros(2)

            loc[0] = x_11
            loc[1] = y_11

            return cases, loc
        

        elif ( s == 0 ) and ( 0 < t < 1 ):

            ''' Case#4 : Intersection is P_21 '''

            cases = 3

            loc = zeros(2)

            loc[0] = x_21
            loc[1] = y_21

            return cases, loc
        

        elif ( t == 0 ) and ( s == 0 ):

            ''' Case#6 : P_11 == P_21 '''

            cases = 4

            loc = zeros(2)

            loc[0] = x_11
            loc[1] = y_11

            return cases, loc
        

        else:

            cases = 0

            loc = zeros(2)

            return cases, loc



def insert_cp(agents):
    """
    ### insert collision points
    insert the collision points in the waypoints list

    input   :   Agent[*] agents
    output  :   [WayPoint,WayPoint][*] cps

    """


    K = len(agents)

    cps = []

    wps_inserted = []

    for agent in agents:

        wps_inserted.append(deepcopy(agent.waypoints))


    for i in range(K-1):

        for j in range(i+1,K):

            wps_i_inserted = wps_inserted[i]
            wps_j_inserted = wps_inserted[j]

            N_i = len(wps_i_inserted)
            N_j = len(wps_j_inserted)

            for n in range(N_i-1,0,-1):

                wp_11 = wps_i_inserted[n-1]
                wp_12 = wps_i_inserted[n]
                
                N_j = len(wps_j_inserted)

                for m in range(N_j-1,0,-1):

                    wp_21 = wps_j_inserted[m-1]
                    wp_22 = wps_j_inserted[m]

                    cases, loc = _check_cp(wp_11,wp_12,wp_21,wp_22)


                    if cases == 0:

                        continue

                    elif cases == 1:                

                        wp_1c = WayPoint(i ,loc, is_cp=True, collide_with=[j])
                        wp_2c = WayPoint(j ,loc, is_cp=True, collide_with=[i])
                        
                        wps_i_inserted.insert(n,wp_1c)
                        wps_j_inserted.insert(m,wp_2c)

                        c_lr = [wp_1c, wp_2c]

                        cps.append(c_lr)                

                    elif cases == 2:  

                        wp_2c = WayPoint(j, loc, is_cp=True, collide_with=[j])

                        wps_j_inserted.insert(m,wp_2c)

                        c_lr = [wps_i_inserted[n-1], wp_2c]

                        cps.append(c_lr)                


                    elif cases == 3:                

                        wp_1c = WayPoint(i, loc, is_cp=True, collide_with=[j])
                        wp_2c = WayPoint(j, loc, is_cp=True, collide_with=[j])

                        wps_i_inserted.insert(n,wp_1c)
                        wps_j_inserted[m-1] = wp_2c       

                        c_lr = [wp_1c, wp_2c]

                        cps.append(c_lr)                


                    elif cases == 4:                

                        c_lr = [wps_i_inserted[n-1], wps_j_inserted[m-1]]

                        cps.append(c_lr)                


    for i in range(K):

        agents[i].waypoints = wps_inserted[i]

    return cps