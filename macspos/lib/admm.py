from numpy import array, zeros, ones, eye, block, diag, tril, where, arange
from numpy.linalg import inv, norm
from time           import sleep

import matplotlib.pyplot as plt

class ADMM:



    def __init__(self, sharedmemory):

        self.sharedmemory = sharedmemory

        ### Agents ###
        self.agents         = sharedmemory.agents           # a list of Agents

        ### Agent number ###
        self.K              = sharedmemory.K                # the number of the agents

        ### Waypoint number ###
        self.N              = sharedmemory.N                # sum of the nuber of waypoints
        self.N_max          = sharedmemory.N_max            # the maximum of the nummber of waypoints
        self.N_c            = sharedmemory.N_c              # the number of the collision points

        ### Constraints ###
        self.v_min          = sharedmemory.v_min            # minimum velocity
        self.v_max          = sharedmemory.v_max            # maximum velocity
        self.d_safe         = sharedmemory.d_safe           # safety distance
        self.t_safe         = sharedmemory.t_safe           # safety time difference
        
        ### Optimize Variables ###
        self.t              = zeros((self.N,1))             # optimize variable
        self.s              = zeros((self.N-self.K,1))      # slack variable
        self.x              = zeros((self.N_c,1))           # slack variable
        self.lam            = 10*ones((self.N+self.N_c,1))  # lagrangian
        self.p              = 1000                          # step size

        ### Constants ###
        self.d              = zeros((self.N-self.K,1))      # lengths of the semgents
        self.cps            = sharedmemory.cps              # collision points
        self.t_st           = sharedmemory.t_st             # starting time

        ### Code Optmization Stuffs ###
        self.S = zeros((self.N_max-1,self.N_max))           # S_1 : for velocity constraints
        self.S[:,:-1] += diag(-ones(self.N_max-1))
        self.S[:,1:] += diag(ones(self.N_max-1))

        self.base_idx = zeros(self.K, dtype=int)            # [0, N1, ... , sum(N1,...,NK-1)]



    def _calc_N_sum_list(self):

        N_sum = 0

        for idx in range(self.K - 1):

            N_sum += self.agents[idx].N

            self.base_idx[idx+1] = N_sum


    def _calc_coeff_matrices(self):

        self._calc_N_sum_list()

        ### Cost Function ###
        Q = zeros((int(self.K*(self.K-1)/2),self.N))
        q = zeros((self.N,1))

        ### Coeff ###
        d = zeros((self.N-self.K,1))
        t_st = zeros((self.K,1))

        ### Constraints ###
        S_1 = zeros((self.N-self.K, self.N))
        S_2 = zeros((self.N_c, self.N))
        S_3 = zeros((self.K, self.N))


        ### q, d, S_1, S_3 ###

        for i in range(self.K):

            q[ self.base_idx[i]+self.agents[i].N - 1, 0 ] = 1

            S_1[self.base_idx[i] - i    :   self.base_idx[i] + self.agents[i].N - i - 1, \
                self.base_idx[i]        :   self.base_idx[i] + self.agents[i].N          ] = \
            self.S[                     :   self.agents[i].N - 1,\
                                        :   self.agents[i].N     ]

            S_3[i,self.base_idx[i]] = 1

            d[self.base_idx[i] - i :   self.base_idx[i] + self.agents[i].N - i - 1] = \
            self.agents[i].lengths
        

        ### Q ###

        temp_i = 0

        for i in range(self.K-1):            
            for j in range(i+1,self.K):

                Q[temp_i,self.base_idx[i] + self.agents[i].N - 1] = 1
                Q[temp_i,self.base_idx[j] + self.agents[j].N - 1] = -1

                temp_i += 1


        ### S_2 ###

        for idx,cp in enumerate(self.cps):

            agent_id_i = cp[0].id
            agent_id_j = cp[1].id

            wp_idx_i = cp[0].idx
            wp_idx_j = cp[1].idx
            
            S_2[idx , self.base_idx[agent_id_i] + wp_idx_i] = 1
            S_2[idx , self.base_idx[agent_id_j] + wp_idx_j] = -1


        ### Coeff Matrices ###

        P = Q.T@Q
        
        A = block([[S_1],
                   [S_2],
                   [S_3]])
        
        B = block([[eye(self.N - self.K)],
                   [zeros((self.N_c + self.K, self.N - self.K))]])
        
        C = block([[zeros((self.N - self.K, self.N_c))],
                   [eye(self.N_c)],
                   [zeros((self.K, self.N_c))]])
        
        c = block([[d/self.v_min],
                   [zeros((self.N_c,1))],
                   [self.t_st]])

        return P,q,S_1,S_2,S_3,d,t_st
    

    def _calc_cost_function(self, P,q,S_1,S_2,S_3,d,t_st, t,s,x, p):

        J = t.T@P@t + q.T@t

        r1 = norm(S_1@t + s)
        r2 = norm(S_2@t - x)
        r3 = norm(S_3@t - t_st)

        r = array([r1,r2,r3])

        return J,r
        

    def _update_t(self, P,q,S_1,S_2,S_3,d,t_st, s,x,w1,w2,w3, p):

        A = block([[S_1],[S_2],[S_3]])

        b = block([[-s + w1],[-x+w2],[-t_st+w3]])

        print("s",s)
        print("w1",w1)

        t = - inv(P + (p/2)*A.T@A) @ (q + p*A.T@b)

        return t


    def _update_s(self, P,q,S_1,S_2,S_3,d,t_st, t,x,w1,w2,w3, p):

        s = S_1@t + w1
        print(w1)
        print(-S_1@t + d/self.v_min)

        ### projection ###

        for i in range(self.N-self.K):

            s_i = s[i]

            if s_i < d[i]/self.v_min:

                s[i] = d[i]/self.v_min

            elif s_i > d[i]/self.v_max:
                
                print("si = ",d[i]/self.v_min - d[i]/self.v_max)
                s[i] = d[i]/self.v_max

        print(s)

        return s
    

    def _update_x(self, P,q,S_1,S_2,S_3,d,t_st, t,s,w1,w2,w3, p):
    
        x   = S_2@t + w2

        ### projection ###

        for i in range(self.N_c):

            x_i = x[i]


            if abs(x_i) < self.t_safe and x_i >= 0:

                x[i] = self.t_safe

            elif abs(x_i) < self.t_safe and x_i < 0:

                x[i] = -self.t_safe

            print(x_i,"->",x[i])

        return x


    def _update_w(self, P,q,S_1,S_2,S_3,d,t_st, t,s,x,w1,w2,w3, p):

        w1 = w1 + S_1@t - s
        
        w2 = w2 + S_2@t - x
        
        w3 = w3 + S_3@t - t_st
        

        return w1,w2,w3


    def run(self):
        '''
            minimize t^TQ^TQt + q^Tt
            
            s.t      s = S_1t + d/v_min
                     x = S_2t
                     s E C
                     x E D 
        '''
        ### step size ###
        p        = 100
        max_iter = 1000

        ### update coefficient matrices ###
        P,q,S_1,S_2,S_3,d,t_st = self._calc_coeff_matrices()
        
        ### init optimization variables ###
        t_c   = zeros((self.N,            1))
        s_c   = zeros((self.N - self.K,   1))
        x_c   = zeros((self.N_c,          1))

        w1_c  = zeros((self.N - self.K,   1))
        w2_c  = zeros((self.N_c,          1))
        w3_c  = zeros((self.K,            1))
        
        t_p   = t_c
        s_p   = s_c
        x_p   = x_c

        w1_p   = w1_c
        w2_p   = w2_c
        w3_p   = w3_c

        ### init cost ###
        J_c,r_c = self._calc_cost_function(P,q,S_1,S_2,S_3,d,t_st, t_c,s_c,x_c, p)

        iter = 0

        J_list = zeros(max_iter)
        r_list = zeros((3, max_iter))


        for i in range(max_iter):

            print("======")
            print(t_c,"\n")
            print(s_c,"\n")
            print(x_c,"\n")
            print(S_1@t_c,"\n")
            print(S_2@t_c,"\n")
            print(S_3@t_c,"\n")



            ### update variables ###

            t_c = self._update_t(P,q,S_1,S_2,S_3,d,t_st,    s_p,x_p,w1_p,w2_p,w3_p, p)
            s_c = self._update_s(P,q,S_1,S_2,S_3,d,t_st,    t_c,x_p,w1_p,w2_p,w3_p, p)
            x_c = self._update_x(P,q,S_1,S_2,S_3,d,t_st,    t_c,s_c,w1_p,w2_p,w3_p, p)
            
            w1_c,w2_c,w3_c = self._update_w(P,q,S_1,S_2,S_3,d,t_st,    t_c,s_c,x_c,w1_p,w2_p,w3_p, p)

            ### update cost ###
            J_p,r_p = J_c,r_c
            J_c,r_c = self._calc_cost_function(P,q,S_1,S_2,S_3,d,t_st, t_c,s_c,x_c, p)

            J_list[i]   = J_c
            r_list[:,i] = r_c
            
            # print(w2_c)

            # print(w1_c,w2_c,w3_c)

            ### stopping criterion ###
            # if 0 < J_p - J_c < 0.01*J_p:

            #     print("converged")

            #     break

            t_p   = t_c
            s_p   = s_c
            x_p   = x_c

            w1_p   = w1_c
            w2_p   = w2_c
            w3_p   = w3_c

            iter += 1

            sleep(0.1)

        # plt.plot(arange(iter),r_list[:,:iter].T)
        # plt.plot(arange(iter),J_list[:iter])
        # plt.show()
