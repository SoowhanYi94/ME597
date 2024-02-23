import cvxpy as cp
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
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
    return U[:,1:]
def create_formation(num):
    graph = nx.empty_graph()
    graph.add_nodes_from([i for i in range(1, num + 1)])
    graph.add_edges_from([(1,2), (1,4), (1,5), (1,6), (2,3), (2,5), (2,8), (3,4), (4, 6),(4,7),(5,6),(5,7), (6,8),(7,8)])
    return graph
def main():
    num = 8
    graph = create_formation(num)
    name = 'custom'
    k =0
    U = gramschmidt(np.column_stack([ np.ones(num),np.random.randn(num, num-1) ]))
    x = cp.Variable((len(graph.edges),len(graph.edges)), diag=True)
    print(U.shape)
    D_D = nx.incidence_matrix(graph, graph.nodes(),oriented=False).toarray()
    print(D_D.shape)

    gamma = cp.Variable(1)
    objective = cp.Maximize(gamma)
    constraints = [cp.trace(x) == 1,(U.T@D_D @ x @ D_D.T@U )>>gamma*np.eye(num-1)]
    prob = cp.Problem(objective, constraints)
    result = prob.solve()
    plt.figure()
    plt.title(f"{name} graph weight distribution")
    edges = list(graph.edges())
    edge_labels = [ '%.4f' % elem for elem in x.value.data[0] ]
    weight = {edges[i]: edge_labels[i] for i in range(len(edges))}
    pos = nx.spring_layout(graph)
    nx.draw(graph,pos = pos, with_labels = True)            
    nx.draw_networkx_edge_labels(graph, pos = pos, edge_labels=weight)
    k +=1
    plt.show()

if __name__ == "__main__":
    main()

