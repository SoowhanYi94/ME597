import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d
def random_graphs_init(graph):
    for i in range(len(graph.nodes())):
        graph.nodes[i]['pos_x'] = random.randint(0, i + 10)
        graph.nodes[i]['pos_y'] = random.randint(0, i + 10)
        graph.nodes[i]['pos_z'] = random.randint(0, i + 10)
        graph.nodes[i]['vel_x'] = random.randint(0, i + 10)
        graph.nodes[i]['vel_y'] = random.randint(0, i + 10)
        graph.nodes[i]['vel_z'] = random.randint(0, i + 10)
        

    return graph

def get_input(x, G):
    k_p = 1
    k_v = 1
    u = np.zeros(len(x)//2)
    L_D = list(nx.directed_laplacian_matrix(G))
    for i in G.nodes():
        for j in G.neighbors(i):
            u[i] += -(k_p * L_D[i][j] * (x[i] - x[j]) + k_v *L_D[i][j]* (x[len(x)//2 + i] - x[len(x)//2 + j]))
    return u

def get_xdot(x, t, G):
    num = len(x)

    A = np.array([[0, 1], [0, 0]])
    B = np.array([0, 1])
    
    # Use Kronecker product to define the new dynamics
    Kronecker_A = np.kron(A, np.eye(num//2))
    
    u = get_input(x, G)

    dxdt = np.matmul(Kronecker_A, x) - np.kron(B, u)
    
    return dxdt 

def main():
    nums = [4]
    for num in nums:
        graphs = [nx.gnm_random_graph(num, 2 * num, directed=True)]
        for graph in graphs:
            graph = random_graphs_init(graph)
            t = np.linspace(0, 30, 101)

            pos_vel_x = np.append(list(nx.get_node_attributes(graph, "pos_x").values()),list(nx.get_node_attributes(graph, "vel_x").values()) )
            pos_vel_y = np.append(list(nx.get_node_attributes(graph, "pos_y").values()),list(nx.get_node_attributes(graph, "vel_y").values()) )
            pos_vel_z = np.append(list(nx.get_node_attributes(graph, "pos_z").values()),list(nx.get_node_attributes(graph, "vel_z").values()) )
            
            
            trajectory_x = odeint(get_xdot, pos_vel_x, t, args=(graph,))
            trajectory_y = odeint(get_xdot, pos_vel_y, t, args=(graph,))
            trajectory_z = odeint(get_xdot, pos_vel_z, t, args=(graph,))

            plt.figure()
            plt.plot(t, trajectory_x[:,:num])
            plt.xlabel("Time t")
            plt.ylabel("Position x of Nodes ")
            plt.figure()
            plt.plot(t, trajectory_x[:,num:])
            plt.xlabel("Time t")
            plt.ylabel("Velocity x of Nodes ")
            plt.figure()
            plt.plot(t, trajectory_y[:,:num])
            plt.xlabel("Time t")
            plt.ylabel("Position y of Nodes ")
            plt.figure()
            plt.plot(t, trajectory_y[:,num:])
            plt.xlabel("Time t")
            plt.ylabel("Velocity x of Nodes ")
            plt.figure()
            plt.plot(t, trajectory_z[:,:num])
            plt.xlabel("Time t")
            plt.ylabel("Position z of Nodes ")
            plt.figure()
            plt.plot(t, trajectory_z[:,num:])
            plt.xlabel("Time t")
            plt.ylabel("Velocity x of Nodes ")
            plt.figure()
            ax = plt.axes(projection='3d')
            for i in range(num):
                ax.plot3D(trajectory_x[:,i], trajectory_y[:,i], trajectory_z[:,i])
                ax.scatter3D(trajectory_x[:,i], trajectory_y[:,i], trajectory_z[:,i])
            plt.xlabel
    plt.show()

if __name__ == "__main__":
    main()