import cvxpy as cp
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp

#https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
def gramschmidt(V):
    n, m = V.shape
    U = np.zeros_like(V)
    U[:, 0] = V[:, 0] / np.linalg.norm(V[:, 0])
    for i in range(1, m):
        U[:, i] = V[:, i]
        for j in range(i):
            U[:, i] -= np.dot(U[:, j].T, U[:, i]) * U[:, j]
        U[:, i] /= np.linalg.norm(U[:, i])
    return U[:,4:]
def create_formation(num):
    graph = nx.empty_graph()
    graph.add_nodes_from([i for i in range(1, num + 1)])
    graph.add_edges_from([(1,2), (1,4), (1,5), (1,6), (2,3), (2,5), (2,8), (3,4), (4, 6),(4,7),(5,6),(5,7), (6,8),(7,8)])
    nx.set_node_attributes(graph, {i: np.random.randint(0, 10, 3) for i in range(1, num+1)}, name="pos")
    number_of_edges = len(graph.edges)
    # r = np.array([np.array([0,0]).T,np.array([-1,1]).T,np.array([-1,0]).T,np.array([-1,-1]).T,np.array([-2,1]).T,np.array([-2,-1]).T,np.array([-3,1]).T,np.array([-3,-1]).T])
    # graph.add_edges_from([(1,1)])
    P_bar = np.array(np.column_stack([[[0,0],[-1,1],[-1,0],[-1,-1],[-2,1],[-2,-1],[-3,1],[-3,-1]], np.ones(num)]))
    D_D = nx.incidence_matrix(graph, oriented=True).toarray()
    E = [[]]
    
    for i in range(num):
        
        if i == 0:
            E = P_bar.T@ D_D @np.diag(D_D.T[:,i])
        else:
            E = np.row_stack([E,P_bar.T@ D_D @np.diag(D_D.T[:,i])])
    z = sp.linalg.null_space(E)
    U, s, VT = sp.linalg.svd(P_bar)
    d = 2
    U_2 = U[:,d+1:]
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
    print(cp.diag(omega))
    gamma = cp.Variable(1)
    objective = cp.Maximize(gamma)
    constraints = [cp.trace(cp.diag(omega)) == 1, U_2.T @ D_D @cp.diag(omega) @ D_D.T @ U_2 >>0]
    prob = cp.Problem(objective=cp.Minimize(0), constraints=constraints)
    result = prob.solve()
    print(omega.value)
    # 
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

    return graph
def main():
    num = 8
    graph = create_formation(num)
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

