from numpy import array, zeros, ones, eye, block, diag, tril, where, arange
from numpy.linalg import inv, norm

from copy           import deepcopy
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

        ### Coefficient Matrices ###
        self.P              = -1                            # Cost quadratic term coeff matrix
        self.q              = -1                            # Cost linear term coeff matrix
        self.S_1            = -1                            # Constraints coefficient matrix
        self.S_2            = -1                            # Constraints coefficient matrix
        self.S_3            = -1                            # Constraints coefficient matrix
        
        ### Optimize Variables ###
        self.t_p            = -1                            # optimize variable
        self.s_p            = -1                            # slack variable
        self.x_p            = -1                            # slack variable
        self.w1_p           = -1                            # lagrangian
        self.w2_p           = -1                            # lagrangian
        self.w3_p           = -1                            # lagrangian

        self.t_c            = -1                            # optimize variable
        self.s_c            = -1                            # slack variable
        self.x_c            = -1                            # slack variable
        self.w1_c           = -1                            # lagrangian
        self.w2_c           = -1                            # lagrangian
        self.w3_c           = -1                            # lagrangian

        self.p              = 1000                          # step size

        ### Constants ###
        self.d              = -1                            # lengths of the semgents
        self.t_st           = sharedmemory.t_st             # starting time

        ### Collision Points ###
        self.cps            = sharedmemory.cps              # collision points


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
        self.q = zeros((self.N,1))

        ### Coeff ###
        self.d = zeros((self.N-self.K,1))

        ### Constraints ###
        self.S_1 = zeros((self.N-self.K, self.N))
        self.S_2 = zeros((self.N_c, self.N))
        self.S_3 = zeros((self.K, self.N))


        ### q, d, S_1, S_3 ###

        for i in range(self.K):

            self.q[ self.base_idx[i]+self.agents[i].N - 1, 0 ] = 1

            self.S_1[self.base_idx[i] - i    :   self.base_idx[i] + self.agents[i].N - i - 1, \
                self.base_idx[i]        :   self.base_idx[i] + self.agents[i].N          ] = \
            self.S[                     :   self.agents[i].N - 1,\
                                        :   self.agents[i].N     ]

            self.S_3[i,self.base_idx[i]] = 1

            self.d[self.base_idx[i] - i :   self.base_idx[i] + self.agents[i].N - i - 1] = \
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
            
            self.S_2[idx , self.base_idx[agent_id_i] + wp_idx_i] = 1
            self.S_2[idx , self.base_idx[agent_id_j] + wp_idx_j] = -1


        ### Coeff Matrices ###

        self.P = Q.T@Q


    def _calc_cost_function(self):

        t = self.t_p
        s = self.s_p
        x = self.x_p

        J_p = t.T@self.P@t + self.q.T@t

        r1 = norm(self.S_1@t + s)
        r2 = norm(self.S_2@t - x)
        r3 = norm(self.S_3@t - self.t_st)

        r_p = array([r1,r2,r3])

        t = self.t_c
        s = self.s_c
        x = self.x_c

        J_c = t.T@self.P@t + self.q.T@t

        # r1 = norm(self.S_1@t - s)
        # r2 = norm(self.S_2@t - x)
        # r3 = norm(self.S_3@t - self.t_st)
        r1 = norm(self.w1_c)
        r2 = norm(self.w2_c)
        r3 = norm(self.w3_c)

        r_c = array([r1,r2,r3])

        return J_p,r_p, J_c, r_c 
        

    def _update_t(self):

        s   = deepcopy(self.s_p)
        x   = deepcopy(self.x_p)

        w1  = deepcopy(self.w1_p)
        w2  = deepcopy(self.w2_p)
        w3  = deepcopy(self.w3_p)

        A = block([[self.S_1],[self.S_2],[self.S_3]])

        b = block([[-s + w1],[-x+w2],[-self.t_st+w3]])

        self.t_p = deepcopy(self.t_c)
        self.t_c = - inv(self.P + (self.p/2)*A.T@A) @ (0.5*(self.q + self.p*A.T@b))

        # print("t : \n",s,"->\n",self.s_c)


    def _update_s(self):

        t   = deepcopy(self.t_c)

        w1  = deepcopy(self.w1_p)

        s = self.S_1@t + w1

        # print(w1)
        # print(-self.S_1@t + self.d/self.v_min)

        ### projection ###

        for i in range(self.N-self.K):

            s_i = s[i]

            if s_i > self.d[i]/self.v_min:

                s[i] = self.d[i]/self.v_min

            elif s_i < self.d[i]/self.v_max:
                
                s[i] = self.d[i]/self.v_max

        # print("s : \n",s,"->\n",self.s_c)

        self.s_p = deepcopy(self.s_c)
        self.s_c = s


    def _update_x(self):
    
        t   = deepcopy(self.t_c)
        w2  = deepcopy(self.w2_p)

        x   = self.S_2@t + w2

        ### projection ###

        for i in range(self.N_c):

            x_i = x[i]


            if abs(x_i) < self.t_safe and x_i >= 0:

                x[i] = self.t_safe

            elif abs(x_i) < self.t_safe and x_i < 0:

                x[i] = -self.t_safe

        # print("x : \n",x,"->\n",self.x_c)

        self.x_p = deepcopy(self.x_c)
        self.x_c = x


    def _update_w(self):

        t   = deepcopy(self.t_c)
        s   = deepcopy(self.s_c)
        x   = deepcopy(self.x_c)

        w1  = deepcopy(self.w1_p)
        w2  = deepcopy(self.w2_p)
        w3  = deepcopy(self.w3_p)

        w1_c = w1 + self.S_1@t - s
        
        w2_c = w2 + self.S_2@t - x
        
        w3_c = w3 + self.S_3@t - self.t_st
        
        # print("w1 : \n",w1,"->\n",w1_c)
        # print("w2 : \n",w2,"->\n",w2_c)
        # print("w3 : \n",w3,"->\n",w3_c)

        self.w1_p,self.w2_p,self.w3_p = deepcopy(self.w1_c),deepcopy(self.w2_c),deepcopy(self.w3_c)
        self.w1_c,self.w2_c,self.w3_c = w1_c,w2_c,w3_c


    def run(self):
        '''
            minimize t^TQ^TQt + q^Tt
            
            s.t      s = S_1t + d/v_min
                     x = S_2t
                     s E C
                     x E D 
        '''
        ### step size ###
        p        = 1000000
        max_iter = 1000

        ### update coefficient matrices ###
        self._calc_coeff_matrices()
        
        ### init optimization variables ###
        self.t_c   = zeros((self.N,            1))
        self.s_c   = zeros((self.N - self.K,   1))
        self.x_c   = zeros((self.N_c,          1))

        self.w1_c  = zeros((self.N - self.K,   1))
        self.w2_c  = zeros((self.N_c,          1))
        self.w3_c  = zeros((self.K,            1))
        
        self.t_p   = deepcopy(self.t_c)
        self.s_p   = deepcopy(self.s_c)
        self.x_p   = deepcopy(self.x_c)

        self.w1_p  = deepcopy(self.w1_c)
        self.w2_p  = deepcopy(self.w2_c)
        self.w3_p  = deepcopy(self.w3_c)

        ### init cost ###
        J_p,r_p,J_c,r_c = self._calc_cost_function()

        iter = 0

        J_list = zeros(max_iter)
        r_list = zeros((3, max_iter))


        for i in range(max_iter):

            ### update variables ###

            self._update_t()
            self._update_s()
            self._update_x()
            self._update_w()

            ### update cost ###
            J_p,r_p,J_c,r_c = self._calc_cost_function()

            J_list[i]   = J_c
            r_list[:,i] = r_c
            
            # print(w2_c)

            # print(w1_c,w2_c,w3_c)

            ### stopping criterion ###
            if 0 < J_p - J_c < 0.01*J_p:

                print("converged")

                break

            iter += 1


        ### Return Results ###

        for agent in self.sharedmemory.agents:

            t_set = deepcopy(self.t_c[self.base_idx[agent.id]:self.base_idx[agent.id]+agent.N])

            d_set = agent.lengths

            delt_set = t_set[1:] - t_set[:-1]

            v_set = d_set/delt_set

            agent.t_prf = t_set
            agent.v_prf = v_set

            # print(f"{id}'th Agent's Initial Time : \n{t_set[0]}")

            # print(f"{id}'th Agent's Velocity Profile : \n{v_set}")


        for i,cp in enumerate(self.cps):

            agent_id_i = cp[0].id
            agent_id_j = cp[1].id

            wp_idx_i = cp[0].idx
            wp_idx_j = cp[1].idx
            
            t_i = self.t_c[self.base_idx[agent_id_i] + wp_idx_i]
            t_j = self.t_c[self.base_idx[agent_id_j] + wp_idx_j]

            self.sharedmemory.cp_time_residual[i] = (agent_id_i , agent_id_j, t_i, t_j)

            # print(f"Collision Point Time Difference : \n{abs(t_i - t_j)}")

            


        # plt.plot(arange(iter),r_list[0,:iter].T,label="residual")
        # # plt.plot(arange(iter),J_list[:iter],label="J")
        # plt.legend()
        # plt.show()
