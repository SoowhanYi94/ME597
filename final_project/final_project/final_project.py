import cvxpy as cp
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import scipy as sp


def sig(x, r):
    result = np.zeros_like(x)
    nonzero_indices = np.nonzero(x)
    result[nonzero_indices] = np.sign(x[nonzero_indices]) * np.abs(x[nonzero_indices]) ** r
    return result
def desired_trajectory(t, starting_point, starting_velocity, starting_acc):
    S_t_k = [0,0] #starting_point
    V_t_k = [0,0] #starting_velocity
    S_t_k1 = [10,4]
    V_t_k1 = [10,0]
    phi_t_k =      1
    phi_dot_t_k =  1
    phi_t_k1 =     1
    phi_dot_t_k1 = 1
    # if t < 10:
    #     S_t_k1 = starting_point +[2,0]
    #     V_t_k1 = [0,0]
    #     t_k = 0
    #     phi_t_k = 1
    #     phi_dot_t_k = 1
    #     phi_t_k1 = 1
    #     phi_dot_t_k1 = 1
    # elif t <20:
    #     S_t_k1 = starting_point + [5,0]
    #     V_t_k1 = [3,0]
    #     t_k = 10
    # elif t <30:
    #     S_t_k1 = starting_point + [15,0]
    #     V_t_k1 = [3,0]
    #     t_k = 20
    # elif t < 40:
    #     S_t_k1 = starting_point + [25,0]
    #     V_t_k1 = [3,0]
    #     t_k = 30
    # elif t < 50:
    #     S_t_k1 = starting_point + [35,0]
    #     V_t_k1 = [3,0]
    #     t_k = 40
    # elif t < 60:
    #     S_t_k1 = starting_point + [45,0]
    #     V_t_k1 = [3,0]
    #     t_k = 50
    
    # elif t < 70:
    #     S_t_k1 = starting_point + [55,0]
    #     V_t_k1 = [3,0]
    #     t_k = 60
    
    # elif t < 80:
    #     S_t_k1 = starting_point + [65,0]
    #     V_t_k1 = [3,0]
    #     t_k = 70
    # else:
    #     S_t_k1 = starting_point + [75,0]
    #     V_t_k1 = [3,0]
    #     t_k = 80
    delta_t = 1
    Delta_t = np.linalg.inv(np.array([[1, 0, 0, 0],[0,1,0,0], [1,delta_t, np.power(delta_t,2), np.power(delta_t,3)],[0, 1, 2*delta_t, 3*np.power(delta_t,2)]]))
    print()
    c = np.kron(Delta_t, np.eye(2))@np.row_stack([S_t_k, V_t_k,S_t_k1, V_t_k1]).reshape(8)
    

    sigma = Delta_t@np.row_stack([phi_t_k, phi_dot_t_k,phi_t_k1, phi_dot_t_k1]).reshape(4)
    
    
   
    return c, sigma

def P_dot(p, t, graph, weight_dic, formation,desired_trajectory, num, dim, order):
    ## p = current position information where p_i is in d dimension
    ## t = time frame
    ## P = list of length of n, containing P, dot_P, double_dot_P, ... n_dot_p at time t the desired tracjetory of the first leader, which has to be nth order differentiable equation. 
    ## Tracking Control Algorithm Design for first leader
    ## Control gains
    k_11 = 1
    k_12 = 16
    
    beta_1 = 0.5
    # control input for second leader group
    rho_j = 0.2
    k_j1 = 16
    k_j2 = 16
    #control input for followers
    gamma_i = 1
    ## the first leader position and velosity is used to generate desired trajectory. 
    p1 = p[0:dim]
    v1 = p[dim*num:(dim)*(num +1)]
    a1 = p[2*(dim*num):2*(dim*num +1)]

    p_cur = p[0:dim*num]
    v_cur = p[dim*num:2*dim*num]
    a_cur = p[2*dim*num:3*dim*num]
    c, sigma = desired_trajectory
    t_k = 0
    b = c[0:2]+ c[2:4] *(t-t_k) + c[4:6] *np.power(t-t_k,2) + c[6:8]*np.power(t-t_k,3)
    b_dot = 3*c[6:8]*np.power(t-t_k,2) + 2* c[4:6] *(t-t_k)+ c[2:4]
    b_ddot = 6*c[6:8]*(t-t_k) + 2* c[4:6] 

    Phi_t = sigma[0]+ sigma[1] *(t-t_k) + sigma[2] *np.power(t-t_k,2) + sigma[3]*np.power(t-t_k,3)
    Phi_dot_t = 3*sigma[3]*np.power(t-t_k,2) + 2* sigma[2] *(t-t_k)+ sigma[1]
    Phi_ddot_t = 6*sigma[3]*(t-t_k) + 2* sigma[2] 
    P, V, Acc = b, b_dot, b_ddot
    P_aug = np.vstack([P, V, Acc])
    p_1_temp = np.vstack([p1,v1,a1])
    ## This calculates the finite time backstepping approach to track desired position.
    alpha_1 = np.zeros(order*dim)
    z_1 = np.zeros(order*dim)
    u1 = 0
    z_dot_1 = np.zeros(order*dim)
    dot_alpha_1 = np.zeros(order*dim)
    for i in range(order):
        if i == 0:
            z_11 = np.array(p1) - np.array(P)
            alpha_11 = - k_11*sig(z_11, beta_1) - k_12*z_11 + P_aug[i+1]
            alpha_1[0:2] += alpha_11
            z_1[0:2] += z_11
            z_dot_11 = p_1_temp[i+1] - P_aug[i+1]
            z_dot_1[0:2] += z_dot_11
            dot_alpha_11 = p_1_temp[i+2] - k_11*sig(z_11, beta_1-1) - k_12*z_11
            dot_alpha_1[0:2] += dot_alpha_11
        elif i < order - 1:
            z_1i = p_1_temp[i]- alpha_1[2*(i-1):2*i]
            z_1[2*(i):2*(i)+2] += z_1i
            alpha_1i = dot_alpha_1[2*(i-1):2*(i-1)+2] - k_11*sig(z_1i, beta_1)- z_1[2*(i-1):2*i] - k_12*z_1i
            z_1i1 = p_1_temp[i+1] - alpha_1i
            z_dot_1i = z_1i1 - k_11*sig(z_1i, beta_1) - k_12*z_1i - z_1[2*(i-1):2*i]
            z_dot_1[2*(i):2*(i)+2] += z_dot_1i

            dot_alpha_1i = P_aug[i +1] - z_dot_1i
            alpha_1[2*(i):2*(i)+2] += alpha_1i
            dot_alpha_1[2*(i):2*(i)+2] += dot_alpha_1i
        else:
            z_1i = p_1_temp[i]- alpha_1[2*(i-1):2*i]
            z_1[2*(i):2*(i)+2] += z_1i
            alpha_1i = dot_alpha_1[2*(i-1):2*(i-1)+2] - k_11*sig(z_1i, beta_1)- z_1[2*(i-1):2*i] - k_12*z_1i
            z_dot_1i = - k_11*sig(z_1i, beta_1) - k_12*z_1i - z_1[2*(i-1):2*i]
            z_dot_1[2*(i):2*(i)+2] += z_dot_1i

            # dot_alpha_1i = z_dot_1i* (-k_11*sig(z_1i, beta_1-1))
            # alpha_1[2*(i):2*(i)+2] += alpha_1i
            # dot_alpha_1[2*(i):2*(i)+2] += dot_alpha_1i
            u1 = alpha_1i
    ##Control algorithm design for the second leader group
    # weight : {edge: nx.get_edge_attribute(graph, "weight")for edge in graph.edges()}
    
    M = 4

    # nodes = [i for i in range(2, M+1)]
    edges = []
    for i in range(2, M+1):
        for j in graph.neighbors(i):
            if (((i,j) in graph.edges()) and (j in range(2,M+1))):
                edges.append((i,j))
    nx.set_edge_attributes(graph,weight_dic, "weight")
    
    # A = nx.adjacency_matrix(graph, nodelist=nodes, weight="weight").toarray()
    # L_s = nx.laplacian_matrix(graph, nodelist=nodes, weight="weight").toarray()
    ## got it wrong here Start from here again
    ## P_aug is suspicious. 
    # print(p2)
    # dot_pos_2_M = -rho_j * np.sign(np.matmul(np.kron((L_s + np.diag(A[:,0])), np.eye(dim)), (p2_M - np.kron(np.array(np.ones(M-1)).reshape(3,1), np.array(p1).reshape(1,2)).reshape((M-1)*dim))))
    # dot_vel_2_M = -rho_j * np.sign(np.matmul(np.kron((L_s + np.diag(A[:,0])), np.eye(dim)), (v2_M - np.kron(np.array(np.ones(M-1)).reshape(3,1), np.array(v1).reshape(1,2)).reshape((M-1)*dim))))
    # dot_acc_2_M = -rho_j * np.sign(np.matmul(np.kron((L_s + np.diag(A[:,0])), np.eye(dim)), (a2_M - np.kron(np.array(np.ones(M-1)).reshape(3,1), np.array(Acc).reshape(1,2)).reshape((M-1)*dim))))
    # dot_pva_2_M = np.vstack([dot_pos_2_M, dot_vel_2_M, dot_acc_2_M])

    ##A(t) transformation needed
    
    A_t = Phi_t * np.eye(dim)
    A_dot_t = Phi_dot_t * np.eye(dim)
    A_ddot_t = Phi_ddot_t * np.eye(dim)
    # A_t         = 0* np.eye(dim)
    # A_dot_t     = 0* np.eye(dim)
    # A_ddot_t    = 0* np.eye(dim)
    ## Scaling design
    
    u2_M =  np.zeros(dim*(M-1))
    r = formation
    for j in range(1, M): ## j = agent number
        p_j = p_cur[j]
        v_j = v_cur[j]
        a_j = a_cur[j]
        P_j = A_t@r[j] + P
        V_j = A_dot_t@r[j] + V
        A_j = A_ddot_t@r[j] + Acc
        p_temp = [p_j, v_j, a_j]
        P_temp = [P_j, V_j, A_j]
        alpha_2_M = np.zeros(dim*(M-1))
        z_2_M = np.zeros(dim*(M-1))
        dot_z_2_M = np.zeros(dim*(M-1))
        dot_alpha_2_M = np.zeros(dim*(M-1))
        u2_j = 0
        for i in range(order):
            if i == 0:
                z_j1 = np.array(p_j) - np.array(P_j)
                alpha_j1 = - k_j1*sig(z_j1, beta_1) - k_j2*z_j1 + P_temp[i+1]
                alpha_2_M[0:2] += alpha_j1
                z_2_M[0:2] += z_j1
                z_dot_2_M = p_temp[i+1] - P_temp[i+1]
                
                dot_z_2_M[0:2] += z_dot_2_M
                dot_alpha_j1 = p_temp[i+2] - k_11*sig(z_dot_2_M, beta_1-1) - k_12*z_dot_2_M
                dot_alpha_2_M[0:2] += dot_alpha_j1
            elif i < order -1 :
                z_ji = p_temp[i]- alpha_2_M[2*(i-1):2*i]
                z_2_M[2*(i):2*(i)+2] += z_ji
                alpha_ji = dot_alpha_2_M[2*(i-1):2*(i-1)+2] - k_j1*sig(z_ji, beta_1)- z_2_M[2*(i-1):2*i] - k_j2*z_ji
                z_ji1 = p_temp[i+1] - alpha_ji
                z_dot_ji = z_ji1 - k_j1*sig(z_1i, beta_1) - k_j2*z_1i - z_1[2*(i-1):2*i]
                dot_z_2_M[2*(i):2*(i)+2] += z_dot_ji

                dot_alpha_2_M_i = P_temp[i +1] - z_dot_ji
                alpha_2_M[2*(i):2*(i)+2] += alpha_ji
                dot_alpha_2_M[2*(i):2*(i)+2] += dot_alpha_2_M_i
            else:
                z_ji = p_temp[i]- alpha_2_M[2*(i-1):2*i]
                alpha_ji = dot_alpha_2_M[2*(i-1):2*i] - k_j1*sig(z_ji, beta_1)- z_2_M[2*(i-1):2*i]-k_j2*z_ji
                alpha_1[2*(i):2*(i)+2] += alpha_ji
                z_1[2*(i):2*(i)+2] += z_ji
                u2_j = alpha_ji

        u2_M[dim*(j-1):dim*(j-1)+2] +=u2_j


    ## PD control algorithm design with constant gains
    m = order -1
    B = np.array(np.row_stack([np.column_stack([list(np.zeros(m)),np.eye(m)]), np.zeros(m + 1)]))
    C = np.array(np.column_stack([[np.zeros(m)],[1]])).transpose()
    R = 1
    Q = np.eye(m+1)
    
    P_riccati = sp.linalg.solve_continuous_are(a = B, b=C, q=Q, r = R)
    # K = [] ##has to be calculated with ARE(Algebraic Riccati Equation)
    p_f = p[dim*M:dim*num]
    v_f = p[dim*(num+M):(dim)*(2*num)]
    a_f = p[(dim*(2*num + M)):(dim)*(3*num)]
    pf_aug = np.vstack([p_f,v_f,a_f])

    
    pcur_aug = np.vstack([p_cur, v_cur, a_cur])
    
    eta_dot_i = np.zeros(4)
    u3 = np.zeros(dim*(num - M))
    K = np.kron(C.T @ P_riccati,np.eye(dim))
    dot_x_f = []
    for i in range(M + 1, num+1):
        s_i = 0
        u_temp = 0
        i_0 = i-M-1
        for j in graph.neighbors(i):
            ## i-1, j-1 because i and j are node numbers not indices.
            x_i = pcur_aug[:,dim*(i-1):dim*(i-1)+2].reshape(dim*order)
            x_j = pcur_aug[:,dim*(j-1):dim*(j-1)+2].reshape(dim*order)
            w = 0
            if ((i,j) in weight_dic):
                w = weight_dic[(i,j)] 
            elif ((j,i) in weight_dic):
                w = weight_dic[(j,i)] 
            s_i +=  w* (x_i - x_j)
            u_temp+= w*K @(x_i - x_j).T
        u3[i_0:i_0+2] = -eta_dot_i[i_0]*u_temp
        dot_eta_i = gamma_i * (s_i.T @np.kron(P_riccati @ C @ C.T @P_riccati, np.eye(dim))) @ s_i
        eta_dot_i[i_0:i_0+2] += dot_eta_i
        dot_x_f_i = (np.kron(B, np.eye(dim)) @ np.reshape(pf_aug[:,i_0:i_0+2], order * dim) + np.kron(C, np.eye(dim)) @ u3[i_0:i_0+2])
        if i == M+1:
            dot_x_f = dot_x_f_i
        else:
            dot_x_f = np.column_stack([dot_x_f, dot_x_f_i])
    print(dot_x_f)
    dot_x_f_pos = dot_x_f[0:2,:]
    dot_x_f_vel = dot_x_f[2:4,:]
    dot_x_f_acc = dot_x_f[4:6,:]
    print(dot_x_f_pos)
    print(dot_x_f_vel)
    print(dot_x_f_acc)
    x_l = pcur_aug[:,0:dim*M]
    dot_x_l = []
    for i in range(M):
            
        dot_x_i = np.kron(B , np.eye(2)) @ np.reshape(x_l[:,2*i:2*i+2], dim*order)
        if i ==0:
            dot_x_i += (list(np.zeros(len(dot_x_i)-2))) + list(u1)
            dot_x_l = dot_x_i
        else:
            dot_x_i +=  (list(np.zeros(len(dot_x_i)-2))) + list(u2_M[2*(i-1):2*(i-1)+2])
            dot_x_l =np.vstack([dot_x_l,dot_x_i])
    dot_x_l_pos = np.reshape(dot_x_l[:,0:2],dim*M)
    dot_x_l_vel = np.reshape(dot_x_l[:,2:4],dim*M)
    dot_x_l_accel = np.reshape(dot_x_l[:,4:6],dim*M)
    ret = list(dot_x_l_pos) + list(dot_x_f_pos.T.reshape(8)) +list(dot_x_l_vel) +list(dot_x_f_vel.T.reshape(8)) +list( dot_x_l_accel)+list( dot_x_f_acc.T.reshape(8)) + list(eta_dot_i)
    return ret

def create_formation(num, dim, P_bar):
    graph = nx.empty_graph()
    graph.add_nodes_from([i for i in range(1, num + 1)])
    graph.add_edges_from([(1,2), (1,4), (1,5), (1,6), (2,3), (2,5), (2,8), (3,4), (4, 6),(4,7),(5,6),(5,7), (6,8),(7,8)])
    nx.set_node_attributes(graph, {i: np.random.randint(0, 4, 2) for i in range(1, num+1)}, name="pos")
    nx.set_node_attributes(graph, {i: np.random.randint(0, 4, 2) for i in range(1, num+1)}, name="vel")
    nx.set_node_attributes(graph, {i: np.random.randint(0, 4, 2) for i in range(1, num+1)}, name="acc")
    number_of_edges = len(graph.edges)

    D_D = nx.incidence_matrix(graph, oriented=True).toarray()
    E = [[]]
    
    for i in range(num):
        
        if i == 0:
            E = P_bar.T@ D_D @np.diag(D_D.T[:,i])
        else:
            E = np.row_stack([E,P_bar.T@ D_D @np.diag(D_D.T[:,i])])
    z = sp.linalg.null_space(E)
    U, s, VT = sp.linalg.svd(P_bar)
    U_2 = U[:,dim+1:]
    M = []
    omega = []
    c = cp.Variable(z.shape[1])
    for i in range(z.shape[1]):
        if i == 0:
            omega = cp.multiply(c[i], z[:,i])
            # M = cp.multiply(c[i],U_2.T @ D_D @np.diag(z[:,i]) @ D_D.T @ U_2)
        else:
            omega += cp.multiply(c[i], z[:,i])
            # M += cp.multiply(c[i],U_2.T @ D_D @np.diag(z[:,i]) @ D_D.T @ U_2)
    gamma = cp.Variable(1)
    objective = cp.Minimize(0)
    constraints = [cp.trace(cp.diag(omega)) == 1, U_2.T @ D_D @cp.diag(omega) @ D_D.T @ U_2 >>0]
    prob = cp.Problem(objective, constraints=constraints)
    result = prob.solve()
    edges = list(graph.edges())
    edge_labels = [ '%.3f' % elem for elem in omega.value]
    weight = {edges[i]: omega.value[i] for i in range(len(edges))}
    nx.set_edge_attributes(graph, edge_labels, "weight")
    pos = nx.get_node_attributes(graph,"pos")
    nx.draw(graph,pos = pos, with_labels = True)            
    nx.draw_networkx_edge_labels(graph, pos = pos, edge_labels=weight)
    plt.show()
    return graph, weight, pos

def simulate(graph, weight_dic, formation,  num, dim, order):
    delta = 10
    iteration = 10 #number of iteration to simulate limit
    time_lim = iteration *delta
    t = np.linspace(0, time_lim, iteration+1)
    eta_i = np.reshape([56 for k in range(num-4)],4)

    p = np.append(np.reshape(list(nx.get_node_attributes(graph,"pos").values()) + list(nx.get_node_attributes(graph,"vel").values()) + list(nx.get_node_attributes(graph,"acc").values()), (num*dim*order)) ,eta_i)
    p1 = p[0:dim]
    v1 = p[dim*num:(dim)*(num )+2]
    a1 = p[2*(dim*num):2*(dim*num )+2]
    desired_trajectory_ = desired_trajectory(t, p1, v1, a1)
    trajectory_P = sp.integrate.odeint(P_dot,p, t, args = (graph, weight_dic, formation, desired_trajectory_, num, dim, order))

    labels = []
    for i in range(num):
        labels.append(f"x{i}")
    # plt.figure()
    # pos = {i+1 : (trajectory_P[-2,2*i], trajectory_P[-2,2*i + 1]) for i in range(num)}
    # print(pos)
    # nx.set_node_attributes(graph, pos, "pos")
    # nx.draw(graph,pos = pos, with_labels = True)   
    plt.figure()         
    for i in range(num):
        # plt.figure()
        plt.plot(trajectory_P[-1,2*i], trajectory_P[-1,2*i +1], 'ko')
        # plt.plot(trajectory_P[0,2*i], trajectory_P[0,2*i +1], 'ro')

        plt.plot(trajectory_P[:,2*i], trajectory_P[:,2*i +1], label =labels[i])
        plt.xlabel("Position x")
        plt.ylabel("Position y")
    plt.legend()
    plt.show()
    L  = nx.laplacian_matrix(graph).toarray()


def main():
    num = 8
    dim = 2
    formation = [[0,0],[-1,1],[-1,0],[-1,-1],[-2,1],[-2,-1],[-3,1],[-3,-1]]
    P_bar = np.array(np.column_stack([[[0,0],[-1,1],[-1,0],[-1,-1],[-2,1],[-2,-1],[-3,1],[-3,-1]], np.ones(num)]))
    graph, weight_dic, pos_dic = create_formation(num, dim, P_bar)
    simulate(graph, weight_dic, formation, num, dim, order =3)
    name = 'custom'
    

    
if __name__ == "__main__":
    main()

