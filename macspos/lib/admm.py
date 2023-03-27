from numpy import array, zeros, ones, eye, block, diag, tril
from numpy.linalg import inv, norm


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

        return P,q,A,B,C,c
    

    def _update_t(self, P,q,A,B,C,c, s,x,w, p):

        t = - inv(P + (p/2)*A.T@A) @ (q.T + (p/2)*(B@s + C@x - c + w).T@A)

        return t


    def _update_s(self, P,q,A,B,C,c, t,x,w, p):

        S_1 = A[ : self.N - self.K, : ]
        c1  = c[ : self.N - self.K, : ]
        w1  = w[ : self.N - self.K, : ]

        s = -S_1@t + c1 - w1

        ### projection ###


        return s
    

    def _update_x(self, P,q,A,B,C,c, t,s,w, p):
    
        S_2 = A[ self.N - self.K : self.N - self.K + self.N_c, : ]
        w2  = w[ self.N - self.K : self.N - self.K + self.N_c, : ]
    
        x   = S_2@t + w2

        ### projection ###


        return x


    def _update_w(self, P,q,A,B,C,c, t,s,x,w, p):

        w = w + A@t + B@s + C@x - c

        return w


    def run(self):
        '''
            minimize t^TQ^TQt + q^Tt
            
            s.t      s = S_1t + d/v_min
                     x = S_2t
                     s E C
                     x E D 
        '''
        ### step size ###
        p = 10

        ### update coefficient matrices ###
        P,q,A,B,C,c = self._calc_coeff_matrices()
        
        ### init optimization variables ###
        t_c   = zeros((self.N + self.N_c, 1))
        s_c   = zeros((self.N - self.K,   1))
        x_c   = zeros((self.N_c,          1))
        w_c   = zeros((self.N + self.N_c, 1))
        
        t_p   = t_c
        s_p   = s_c
        x_p   = x_c
        w_p   = w_c

        
        for i in range(20):

            ### update variables ###

            t_c = self._update_t(P,q,A,B,C,c, s_p,x_p,w_p,      p)
            s_c = self._update_t(P,q,A,B,C,c, t_p,x_p,w_p,      p)
            x_c = self._update_t(P,q,A,B,C,c, t_p,s_p,w_p,      p)
            w_c = self._update_t(P,q,A,B,C,c, t_p,s_p,x_p,w_p,  p)

            ### stopping criterion ###

            


            t_p   = t_c
            s_p   = s_c
            x_p   = x_c
            w_p   = w_c



