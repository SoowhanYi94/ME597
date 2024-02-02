import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d
def random_graphs_init(graph):
    for i in range(len(graph.nodes())):
        graph.nodes[i]['pos_x'] = random.randint(0, i + 100000)
        graph.nodes[i]['pos_y'] = random.randint(0, i + 100000)
        graph.nodes[i]['pos_z'] = random.randint(0, i + 100000)
        graph.nodes[i]['vel_x'] = random.randint(0, i + 100000)
        graph.nodes[i]['vel_y'] = random.randint(0, i + 100000)
        graph.nodes[i]['vel_z'] = random.randint(0, i + 100000)
        graph.nodes[i]['theta'] = random.uniform(np.pi/2, 3*np.pi/2)

    return graph

def get_xdot(theta, t, G, omega):
    D = nx.incidence_matrix(G).toarray()


    dxdt = omega + np.matmul(D,np.matmul(D.T, np.sin(theta)))
    
    return dxdt 

def main():
    nums = [5]
    for num in nums:
        names = ['cycle','path', 'star', 'complete']
        graphs = [nx.cycle_graph(num), nx.path_graph(num), nx.star_graph(num - 1), nx.complete_graph(num)]
        omega = [5, 0 ,5, 0, 0]
        k =0
        for graph in graphs:
            graph = random_graphs_init(graph)
            t = np.linspace(0, 30, 101)
            trajectory_theta = odeint(get_xdot, list(nx.get_node_attributes(graph, "theta").values()), t, args=(graph, omega))
            plt.figure()
            plt.plot(t, trajectory_theta)
            plt.xlabel("Time t")
            plt.ylabel("Heading of Nodes ")
            plt.title(f"Heading of Nodes {names[k]} ")
            k +=1
    plt.show()

if __name__ == "__main__":
    main()