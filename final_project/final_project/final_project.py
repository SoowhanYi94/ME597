import cvxpy as cp
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp



def P_dot(p, t, graph, P, num, dim):
    D_D = nx.incidence_matrix(graph, oriented=True).toarray()
    z_ref = np.kron(D_D.T, np.eye(2))@ P
    z_t = np.kron(D_D.T, np.eye(2))@p
    u = np.kron(D_D, np.eye(2)) @(z_ref - z_t)
    
    if (t>9) & (t < 9.2):
        p_reshape = np.reshape(p, (num,dim))
        pos = {i:(p_reshape[i-1]) for i in range(1, num+1)}
        nx.set_node_attributes(graph, pos, "pos")
        nx.draw(graph, pos = pos)
        plt.show()
    return u
def create_formation(num, dim, P, P_bar):
    graph = nx.empty_graph()
    graph.add_nodes_from([i for i in range(1, num + 1)])
    graph.add_edges_from([(1,2), (1,4), (1,5), (1,6), (2,3), (2,5), (2,8), (3,4), (4, 6),(4,7),(5,6),(5,7), (6,8),(7,8)])
    nx.set_node_attributes(graph, {i: np.random.randint(0, 10, 2) for i in range(1, num+1)}, name="pos")
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
    weight = {edges[i]: edge_labels[i] for i in range(len(edges))}
    pos = nx.get_node_attributes(graph,"pos")
    nx.draw(graph,pos = pos, with_labels = True)            
    nx.draw_networkx_edge_labels(graph, pos = pos, edge_labels=weight)
    plt.show()
    return graph, weight, pos

def simulate(graph, num, dim, P):
    delta = .1
    iteration = 1000 #number of iteration to simulate limit
    time_lim = iteration *delta
    t = np.linspace(0, time_lim, iteration)
    p = np.reshape(list(nx.get_node_attributes(graph,"pos").values()), (num*dim))
    trajectory_P = sp.integrate.odeint(P_dot,p, t, args = (graph, P , num, dim))
    print(trajectory_P.shape)
    plt.figure()
    labels = []
    for i in range(num):
        labels.append(f"x{i}")
    t = np.linspace(0, time_lim, iteration)
    # pic_frame = w
    for i in range(num):

        plt.plot(trajectory_P[990,2*i], trajectory_P[980,2*i +1], 'ko')
        plt.plot(trajectory_P[:,2*i], trajectory_P[:,2*i +1], label =labels[i])
        plt.xlabel("Time t")
        plt.ylabel("Position")
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
    P = [[0,0],[-1,1],[-1,0],[-1,-1],[-2,1],[-2,-1],[-3,1],[-3,-1]]
    P = np.reshape(P, num*dim)
    P_bar = np.array(np.column_stack([[[0,0],[-1,1],[-1,0],[-1,-1],[-2,1],[-2,-1],[-3,1],[-3,-1]], np.ones(num)]))
    graph, weight_dic, pos_dic = create_formation(num, dim, P, P_bar)
    simulate(graph, num, dim, P)
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

