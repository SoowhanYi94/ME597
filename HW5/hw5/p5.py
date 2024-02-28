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

def main():
    nums = [9]
    custom_graph = nx.star_graph(nums[0]-2)
    custom_graph.add_node(nums[0] -1)
    custom_graph.add_edge(nums[0]-1, nums[0]-2)
    for num in nums:
        names = ['complete', 'path', 'star', 'custom' ]
        # graphs = [ custom_graph]
        graphs = [ nx.complete_graph(num), nx.path_graph(num), nx.star_graph(num - 1),custom_graph]
        k =0
        for graph in graphs:
            U = gramschmidt(np.column_stack([np.ones(num), np.random.randn(num, num-1)]))
            x = cp.Variable((len(graph.edges),len(graph.edges)), diag=True)
            D_D = nx.incidence_matrix(graph, graph.nodes(),oriented=True).toarray()
            gamma = cp.Variable(1)
            objective = cp.Maximize(gamma)
            constraints = [x >>0, cp.trace(x) == 1,(U.T@D_D @ x @ D_D.T@U )>>gamma*np.eye(num-1)]
            prob = cp.Problem(objective, constraints)
            result = prob.solve()
            plt.figure()
            plt.title(f"{names[k]} graph weight distribution")
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

