import cvxpy as cp
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import scipy as sp


def sig(z, beta):
    ret = 0
    if (z.dtype == int) or (z.dtype == float):
        ret = np.sign(z) *np.power(np.abs(z),beta)
    else:
        ret = np.multiply(np.sign(z),sp.linalg.fractional_matrix_power(np.abs(z),beta)) 
    return ret
def desired_trajectory(t, starting_point, starting_velocity, starting_acc):
    S_t_k = starting_point
    V_t_k = starting_velocity
    S_t_k1 = 0
    V_t_k1 = 0
    t_k = 0
    if t < 10:
        S_t_k1 = starting_point +[2,0]
        V_t_k1 = [0,0]
        t_k = 0
    elif t <20:
        S_t_k1 = starting_point + [5,0]
        V_t_k1 = [3,0]
        t_k = 10
    elif t <30:
        S_t_k1 = starting_point + [15,0]
        V_t_k1 = [3,0]
        t_k = 20
    elif t < 40:
        S_t_k1 = starting_point + [25,0]
        V_t_k1 = [3,0]
        t_k = 30
    elif t < 50:
        S_t_k1 = starting_point + [35,0]
        V_t_k1 = [3,0]
        t_k = 40
    elif t < 60:
        S_t_k1 = starting_point + [45,0]
        V_t_k1 = [3,0]
        t_k = 50
    
    elif t < 70:
        S_t_k1 = starting_point + [55,0]
        V_t_k1 = [3,0]
        t_k = 60
    
    elif t < 80:
        S_t_k1 = starting_point + [65,0]
        V_t_k1 = [3,0]
        t_k = 70
    else:
        S_t_k1 = starting_point + [75,0]
        V_t_k1 = [3,0]
        t_k = 80
    delta_t = 10
    Delta_t = np.linalg.inv(np.array([[1, 0, 0, 0],[0,1,0,0], [1,delta_t, delta_t^2, delta_t^3],[0, 1, 2*delta_t, 3*delta_t^2]]))
    
    c = np.kron(Delta_t, np.eye(2))@np.row_stack([S_t_k, V_t_k,S_t_k1, V_t_k1]).reshape(8)
    b = c[0:2]+ c[2:4] *(t-t_k) + c[4:6] *np.power(t-t_k,2) + c[6:8]*np.power(t-t_k,3)
    b_dot = 3*c[6:8]*np.power(t-t_k,2) + 2* c[4:6] *(t-t_k)+ c[2:4]
    b_ddot = 6*c[6:8]*(t-t_k) + 2* c[4:6] 
    
    P = b
    V = b_dot
    Acc =  b_ddot
    return P, V, Acc
def P_dot(p, t, graph, weight_dic, num, dim, order):
    ## p = current position information where p_i is in d dimension
    ## t = time frame
    ## P = list of length of n, containing P, dot_P, double_dot_P, ... n_dot_p at time t the desired tracjetory of the first leader, which has to be nth order differentiable equation. 
    ## Tracking Control Algorithm Design for first leader
    ## Control gains
    k_11 = 1
    k_12 = 16
    beta_1 = 0.5
    # control input for second leader group
    rho_j = 12
    #control input for followers
    gamma_i = 0.5
    ## the first leader position and velosity is used to generate desired trajectory. 
    p1 = p[0:dim]
    v1 = p[dim*num:(dim)*(num +1)]
    a1 = p[2*(dim*num):2*(dim*num +1)]
    P, V, Acc = desired_trajectory(t, p1, v1, a1)
    P_aug = np.vstack([P, V, Acc])
    ## This calculates the finite time backstepping approach to track desired position.
    alpha_1 = np.zeros(len(P)*dim)
    z_1 = np.zeros(len(P)*dim)
    u1 = 0
    for i in range(order):
        if i == 0:
            z_11 = np.array(p1) - np.array(P)
            alpha_11 = - k_11*sig(z_11, beta_1) - k_12*z_11 + P[i+1]
            
            alpha_1[0:2] += alpha_11
            z_1[0:2] += z_11
        
        else:
            z_1i = p1[i-1]- alpha_1[2*(i-1):2*i]
            alpha_1i = alpha_1[2*(i-1):2*i] - k_11*sig(z_1i, beta_1)- z_1[2*(i-1):2*i]
         
            alpha_1[i:i+2] += alpha_1i
            z_1[i:i+2] += z_1i
            if i == len(P)-1:
                u1 = alpha_1i
    ##Control algorithm design for the second leader group
    # weight : {edge: nx.get_edge_attribute(graph, "weight")for edge in graph.edges()}
    
    M = 4

    p2_M = p[dim:dim*M]
    v2_M = p[dim*(num+1):(dim)*(num + M)]
    a2_M = p[(dim*(2*num+1)):dim*(2*num + M)]
    p2_aug = np.vstack([p2_M,v2_M,a2_M])
    nodes = [i for i in range(2, M+1)]
    edges = []
    for i in range(2, M+1):
        for j in graph.neighbors(i):
            if (((i,j) in graph.edges()) and (j in range(2,M+1))):
                edges.append((i,j))
    nx.set_edge_attributes(graph,weight_dic, "weight")
    D = nx.incidence_matrix(graph, nodelist=nodes, edgelist=edges, oriented=True, weight="weight").toarray()
    A = nx.adjacency_matrix(graph, nodelist=nodes, weight="weight").toarray()
    L_s = nx.laplacian_matrix(graph, nodelist=nodes, weight="weight").toarray()
    
    u2 = -rho_j * np.sign(np.matmul(np.kron((L_s + np.diag(A[:,0])), np.eye(dim)), (P_aug - np.kron(np.array(np.ones(M-1)).reshape(3,1), np.array(p1).reshape(1,2))).reshape((M-1)*dim)))
    

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
    a_f = p[(dim*(2*num + M)):]
    pf_aug = np.vstack([p_f,v_f,a_f])

    p_cur = p[0:dim*num]
    v_cur = p[dim*num:2*dim*num]
    a_cur = p[2*dim*num:]
    pcur_aug = np.vstack([p_cur, v_cur, a_cur])
    eta_i = [56 for k in range(M+1, num+1)]

    u3 = np.zeros(dim*(num - M))
    K = np.kron(C.T @ P_riccati,np.eye(dim))
    dot_x_f = []
    for i in range(M + 1, num+1):
        s_i = 0
        u_temp = 0
        i_0 = i-M-1
        for j in graph.neighbors(i):
            x_i = pcur_aug[:,i:i+2].reshape(dim*order)
            x_j = pcur_aug[:,j:j+2].reshape(dim*order)
            w = 0
            if ((i,j) in weight_dic):
                w = weight_dic[(i,j)] 
            elif ((j,i) in weight_dic):
                w = weight_dic[(j,i)] 
            s_i +=  w* (x_i - x_j)
            u_temp+= w*K @(x_i - x_j).T
        u3[i_0:i_0+2] = -eta_i[i_0]*u_temp
        dot_eta_i = gamma_i * (s_i.T @np.kron(P_riccati @ C @ C.T @P_riccati, np.eye(dim))) @ s_i
        eta_i[i_0] -=dot_eta_i
        dot_x_f_i = (np.kron(B, np.eye(dim)) @ np.reshape(pf_aug[:,i_0:i_0+2], order * dim) + np.kron(C, np.eye(dim)) @ u3[i_0:i_0+2])
        if i == M+1:
            dot_x_f = dot_x_f_i
        else:
            dot_x_f = np.column_stack([dot_x_f, dot_x_f_i])
        # print(dot_x_f_i)
        # print(dot_eta_i)
    # print(dot_x_f)
    dot_x_f_pos = dot_x_f[0:2,:]
    dot_x_f_vel = dot_x_f[2:4,:]
    dot_x_f_acc = dot_x_f[4:6,:]
    # print(u1)
    # print(u2)
    # print(u3)
    x_l = pcur_aug[:,0:dim*M]
    dot_x_l = []
    for i in range(M):
            
        dot_x_i = np.kron(B , np.eye(2)) @ np.reshape(x_l[:,2*i:2*i+2], dim*order)
        if i ==0:
            dot_x_i += (list(np.zeros(len(dot_x_i)-2))) + list(u1)
            dot_x_l = dot_x_i
        else:
            dot_x_i += (list(np.zeros(len(dot_x_i)-2))) + list(u2[2*(i-1):2*(i)])
            dot_x_l =np.vstack([dot_x_l,dot_x_i])
    dot_x_l_pos = np.reshape(dot_x_l[:,0:2],dim*M)
    dot_x_l_vel = np.reshape(dot_x_l[:,2:4],dim*M)
    dot_x_l_accel = np.reshape(dot_x_l[:,4:6],dim*M)
    # print(dot_x_l_pos)
    # print(dot_x_f_pos.T.reshape(8))

    # print(dot_x_l_vel)
    # print(dot_x_f_vel.T.reshape(8))
    # print(dot_x_l_accel)
    # print(dot_x_f_acc.T.reshape(8))
    ret = list(dot_x_l_pos) + list(dot_x_f_pos.T.reshape(8)) +list(dot_x_l_vel) +list(dot_x_f_vel.T.reshape(8)) +list( dot_x_l_accel)+list( dot_x_f_acc.T.reshape(8))
    # print(len(ret))
    return ret


    D_D = nx.incidence_matrix(graph, oriented=True).toarray()
    # z_ref = np.kron(D_D.T, np.eye(2))@ P
    # z_t = np.kron(D_D.T, np.eye(2))@p
    # u = np.kron(D_D, np.eye(2)) @(z_ref - z_t)
    # print(u.shape)
    
    # return u
def create_formation(num, dim, P_bar):
    graph = nx.empty_graph()
    graph.add_nodes_from([i for i in range(1, num + 1)])
    graph.add_edges_from([(1,2), (1,4), (1,5), (1,6), (2,3), (2,5), (2,8), (3,4), (4, 6),(4,7),(5,6),(5,7), (6,8),(7,8)])
    nx.set_node_attributes(graph, {i: np.random.randint(0, 4, 2) for i in range(1, num+1)}, name="pos")
    nx.set_node_attributes(graph, {i: np.random.randint(0, 4, 2) for i in range(1, num+1)}, name="vel")
    nx.set_node_attributes(graph, {i: np.random.randint(0, 4, 2) for i in range(1, num+1)}, name="acc")
    number_of_edges = len(graph.edges)
    # r = np.array([np.array([0,0]).T,np.array([-1,1]).T,np.array([-1,0]).T,np.array([-1,-1]).T,np.array([-2,1]).T,np.array([-2,-1]).T,np.array([-3,1]).T,np.array([-3,-1]).T])
    # graph.add_edges_from([(1,1)])
    
    
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
    Omega = cp.Variable((z.shape[0], z.shape[0]), diag=True)
    c = cp.Variable(z.shape[1])
    for i in range(z.shape[1]):
        if i == 0:
            omega = cp.multiply(c[i], z[:,i])
            # M = cp.multiply(c[i],U_2.T @ D_D @np.diag(z[:,i]) @ D_D.T @ U_2)
        else:
            omega += cp.multiply(c[i], z[:,i])
            # M += cp.multiply(c[i],U_2.T @ D_D @np.diag(z[:,i]) @ D_D.T @ U_2)
    
    constraints = [cp.trace(cp.diag(omega)) == 1, U_2.T @ D_D @cp.diag(omega) @ D_D.T @ U_2 >>0]
    prob = cp.Problem(objective=cp.Minimize(0), constraints=constraints)
    result = prob.solve()
    edges = list(graph.edges())
    edge_labels = [ '%.4f' % elem for elem in omega.value]
    weight = {edges[i]: omega.value[i] for i in range(len(edges))}
    nx.set_edge_attributes(graph, edge_labels, "weight")
    pos = nx.get_node_attributes(graph,"pos")
    nx.draw(graph,pos = pos, with_labels = True)            
    nx.draw_networkx_edge_labels(graph, pos = pos, edge_labels=weight)
    plt.show()
    return graph, weight, pos

def simulate(graph, weight_dic, num, dim, order):
    delta = 0.1
    iteration = 12 #number of iteration to simulate limit
    time_lim = iteration *delta
    t = np.linspace(0, time_lim, iteration+1)
    p = np.reshape(list(nx.get_node_attributes(graph,"pos").values()) + list(nx.get_node_attributes(graph,"vel").values()) + list(nx.get_node_attributes(graph,"acc").values()), (num*dim*order))
    # print(len(p))
    # v = np.reshape(list(nx.get_node_attributes(graph,"vel").values()), (num*dim))
    # acc = np.reshape(list(nx.get_node_attributes(graph,"acc").values()), (num*dim))
    # p.append(v)
    # p.append(acc)
    trajectory_P = sp.integrate.odeint(P_dot,p, t, args = (graph, weight_dic, num, dim, order))
    # print(trajectory_P.shape)
    # plt.figure()
    labels = []
    for i in range(num):
        labels.append(f"x{i}")
    # t = np.linspace(0, time_lim, iteration)
    print(t)
    # pic_frame = w
    plt.figure()
    for i in range(num):
        # plt.figure()
        plt.plot(trajectory_P[iteration-1,2*i], trajectory_P[iteration-1,2*i +1], 'ko')
        plt.plot(trajectory_P[:,2*i], trajectory_P[:,2*i +1], label =labels[i])
        plt.xlabel("Position x")
        plt.ylabel("Position y")
    plt.legend()
    plt.show()
    L  = nx.laplacian_matrix(graph).toarray()
    # W = cp.Variable((number_of_edges,number_of_edges), symmetric = True)
    # constraints = [W >> 1]
    # objective = np.kron(D_D@W@D_D.T)@x==0
    # problem = cp.Problem(objective=objective, constraints=constraints)
    # result = problem.solve()
    


    # 
    # prob = cp.Problem(objective, constraints)
    # result = prob.solve()
    # print(D_D)

def main():
    num = 8
    dim = 2
    # P = [[0,0],[-1,1],[-1,0],[-1,-1],[-2,1],[-2,-1],[-3,1],[-3,-1]]
    # P = np.reshape(P, num*dim)
    P_bar = np.array(np.column_stack([[[0,0],[-1,1],[-1,0],[-1,-1],[-2,1],[-2,-1],[-3,1],[-3,-1]], np.ones(num)]))
    graph, weight_dic, pos_dic = create_formation(num, dim, P_bar)
    simulate(graph, weight_dic, num, dim, order =3)
    name = 'custom'
    

    
    # plt.figure()
    # plt.title(f"{name} graph weight distribution")
    # edges = list(graph.edges())
    # edge_labels = [ '%.4f' % elem for elem in x.value.data[0] ]
    # weight = {edges[i]: edge_labels[i] for i in range(len(edges))}
    # pos = nx.spring_layout(graph)
    # nx.draw(graph,pos = pos, with_labels = True)            
    # nx.draw_networkx_edge_labels(graph, pos = pos, edge_labels=weight)
    # k +=1
    # plt.show()

if __name__ == "__main__":
    main()

