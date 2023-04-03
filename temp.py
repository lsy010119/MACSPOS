from numpy import array, zeros, ones, eye, block, diag, tril, where, arange
from numpy.linalg import inv, norm

import matplotlib.pyplot as plt

class ADMM:



    def __init__(self, sharedmemory):

        self.sharedmemory = sharedmemory

        ### Agents ###
        self.agents = sharedmemory.agents # a list of Agents

        ### Agent number ###
        self.K = sharedmemory.K # the number of the agents

        ### Waypoint number ###
        self.N = sharedmemory.N # sum of the nuber of waypoints
        self.N_max = sharedmemory.N_max # the maximum of the nummber of waypoints
        self.N_c = sharedmemory.N_c # the number of the collision points

        ### Constraints ###
        self.v_min = sharedmemory.v_min # minimum velocity
        self.v_max = sharedmemory.v_max # maximum velocity
        self.d_safe = sharedmemory.d_safe # safety distance
        self.t_safe = sharedmemory.t_safe # safety time difference
        
        ### Optimize Variables ###
        self.t = zeros((self.N,1)) # optimize variable
        self.p = 1000 # step size

        ### Constants ###
        self.d = zeros((self.N-self.K,1)) # lengths of the semgents
        self.cps = sharedmemory.cps # collision points
        self.t_st = sharedmemory.t_st # starting time

        ### Code Optmization Stuffs ###
        self.S = zeros((self.N_max-1,self.N_max)) # S_1 : for velocity constraints
        self.S[:,:-1] += diag(-ones(self.N_max-1))
        self.S[:,1:] += diag(ones(self.N_max-1))

        self.base_idx = zeros(self.K, dtype=int) # [0, N1, ... , sum(N1,...,NK-1)]
        
        '''
        self.rho = 10
        self.t0 = 
        '''


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
        S1 = zeros((self.N-self.K, self.N))
        S2 = zeros((self.N_c, self.N))
        S3 = zeros((self.K, self.N))


        ### q, d, S1, S3 ###

        for i in range(self.K):

            q[ self.base_idx[i]+self.agents[i].N - 1, 0 ] = 1

            S1[self.base_idx[i] - i : self.base_idx[i] + self.agents[i].N - i - 1, \
                self.base_idx[i] : self.base_idx[i] + self.agents[i].N ] = \
            self.S[ : self.agents[i].N - 1,\
                                        : self.agents[i].N ]

            S3[i,self.base_idx[i]] = 1

            d[self.base_idx[i] - i : self.base_idx[i] + self.agents[i].N - i - 1] = \
            self.agents[i].lengths


        ### Q ###

        temp_i = 0

        for i in range(self.K-1):            
            for j in range(i+1,self.K):

                Q[temp_i,self.base_idx[i] + self.agents[i].N - 1] = 1
                Q[temp_i,self.base_idx[j] + self.agents[j].N - 1] = -1

                temp_i += 1


        ### S2 ###

        for idx,cp in enumerate(self.cps):

            agent_id_i = cp[0].id
            agent_id_j = cp[1].id

            wp_idx_i = cp[0].idx
            wp_idx_j = cp[1].idx
            
            S2[idx , self.base_idx[agent_id_i] + wp_idx_i] = 1
            S2[idx , self.base_idx[agent_id_j] + wp_idx_j] = -1


        ### Coeff Matrices ###

        P = Q.T@Q
        A = block([(self.rho/2)**0.5*S3], [(self.rho/2)**0.5*S1], [(self.rho/2)**0.5*S2])
        PATA = -inv(P + A.T@A)
        PATAq = PATA@q/2
        PATAAT = PATA@A^T

        return P,q,S1,S2,S3,PATAq,PATAAT
    

    def _calc_cost_function(self,P,q,t):

        J = t.T@P@t + q.T@t

        return J
        

    def _update_t(self,PATAq,PATAAT,z_bef,zeta_bef,w0_bef,w1_bef,w2_bef):

        B = block([(self.rho/2)**0.5*(-self.t0+w0_bef)], [(self.rho/2)**0.5*(-z_bef+w1_bef)], [(self.rho/2)**0.5*(-zeta_bef+w2_bef)])
        t = PATAq + PATAAT@B

        return t


    def _update_z(self,t,S1,w1_bef):
        
        '''
        z_aft = S1@t + w1_bef
        '''
        
        return z_aft
    

    def _update_zeta(self,t,S2,w2_bef):
    
        '''
        zeta_aft = S2@t + w2_bef
        '''
        
        return zeta_aft


    def _update_w(self,t,w0_bef,w1_bef,w2_bef,z_aft,zeta_aft,S1,S2,S3):

        w0_aft = w0_bef + S3@t - self.t0
        w1_aft = w1_bef + S1@t - z_aft
        w2_aft = w2_bef + S2@t - zeta_aft

        return w0_aft,w1_aft,w2_aft


    def run(self):
        
        ### step size ###
        max_iter = 10
        cost = zeros(max_iter)

        ### update coefficient matrices ###
        P,q,S1,S2,S3,PATAq,PATAAT = self._calc_coeff_matrices()
        
        ### init optimization variables ###
        z_bef = zeros((self.N - self.K, 1))
        zeta_bef = zeros((self.N_c, 1))
        w0_bef = zeros((self.K, 1))
        w1_bef = zeros((self.N - self.K, 1))
        w2_bef = zeros((self.N_c, 1))

        for iter in range(max_iter):

            ### update variables ###
            t = self._update_t(self,PATAAT,z_bef,zeta_bef,w0_bef,w1_bef,w2_bef)
            z_aft = self._update_z(self,t,S1,w1_bef)
            zeta_aft = self._update_zeta(self,t,S2,w2_bef)
            w0_aft,w1_aft,w2_aft = _update_w(self,t,w0_bef,w1_bef,w2_bef,z_aft,zeta_aft,S1,S2,S3)
            
            ###
            cost[iter] = self._calc_cost_function(self,P,q,t)
            
            ### update variables ###
            z_bef = z_aft
            zeta_bef = zeta_aft
            w0_bef = w0_aft
            w1_bef = w1_aft
            w2_bef = w2_aft


        # plt.plot(arange(iter),r_c_list[:,:iter].T)
        plt.plot(arange(iter),con1_list[:,:iter].T)
        # plt.plot(arange(iter),con2_list[:,:iter].T)
        # plt.plot(arange(iter),con3_list[:,:iter].T)
        plt.show()