import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

def random_graphs_init(graph):
    x = np.zeros((len(graph), 2))
    for i in range(len(graph)):
        pos, vel = random.randint(0, len(graph)), random.randint(0, len(graph))
        graph.nodes[i]['pos_vel'] = (pos, vel)
        x[i][0] = pos
        x[i][1] = vel
    return graph, x

def get_input(x, G):
    k_p = 0.01
    k_v = 0.1
    u = np.zeros_like(x)

    for i in G.nodes():
        for j in G.neighbors(i):
            u[i] += k_p * (x[i, 0] - x[j, 0]) - k_v * (x[i, 1] - x[j, 1])
    return u

def get_xdot(x, t, G):
    num = len(x)
    x = x.reshape((num // 2, 2))

    A = np.array([[0, 1], [0, 0]])
    B = np.array([0, 1])
    L_G = nx.laplacian_matrix(G).toarray()
    
    # Use Kronecker product to define the new dynamics
    Kronecker_A = np.kron(A, np.eye(num // 2))
    Kronecker_B = np.kron(np.eye(num // 2), B)
    Kronecker_L = np.kron(np.eye(num // 2), L_G)

    u = get_input(x, G)

    # The dynamics equation using Kronecker product
    dxdt = np.matmul(Kronecker_A, x.T.flatten()) - np.matmul(Kronecker_B, u) - np.matmul(Kronecker_L, x.T.flatten())
    
    return dxdt.flatten()

def main():
    nums = [10]
    for num in nums:
        graphs = [nx.gnm_random_graph(num, 2 * num)]
        for graph in graphs:
            graph, x = random_graphs_init(graph)
            t = np.linspace(0, 10, 100)
            trajectory = odeint(get_xdot, x.flatten(), t, args=(graph,))

            plt.figure()
            plt.plot(t, trajectory)
            plt.xlabel("Time t")
            plt.ylabel("Position and Velocity of Nodes")
            plt.grid()
            plt.title(f"Convergence of Position and Velocity with {num} Nodes")
            plt.legend([f'Node {i + 1}' for i in range(num)])

    plt.show()

if __name__ == "__main__":
    main()