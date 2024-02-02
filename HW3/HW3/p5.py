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
def random_connected_graphs_init(graph):
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
def addEdge(G):
    # diameter = nx.diameter(G)
    start = G.nodes[0]
    end = G.nodes[0]
    max = 0    
    for node_i in G.nodes:
        for node_j in G.nodes:
            if node_i != node_j:
                path = longest_simple_paths(G, node_i, node_j)
                if max < len(path):
                    max = len(path)
                    start = node_i
                    end = node_j

    G.add_edge(start, end)
              
    return G

def longest_simple_paths(graph, source, target):
    longest_paths = []
    longest_path_length = 0
    if not nx.has_path(graph, source, target): 
        return None
    for path in nx.shortest_simple_paths(graph, source=source, target=target):
        if len(path) > longest_path_length:
            longest_path_length = len(path)
            longest_paths.clear()
            longest_paths.append(path)
        elif len(path) == longest_path_length:
            longest_paths.append(path)
    return longest_paths

def main():
    nums = [10]
    for num in nums:
        graphs = [nx.erdos_renyi_graph(500, 0.5, seed=123, directed=False)]
        # omega = [5, 0 ,5, 0, 0]
        k =0
        for graph in graphs:
            graph = random_graphs_init(graph)
            # graph = nx.random_spanning_tree(graph)
            t = np.linspace(0, 30, 101)
            graph = addEdge(graph)

            # trajectory_theta = odeint(get_xdot, list(nx.get_node_attributes(graph, "theta").values()), t, args=(graph, ))
            # plt.figure()
            # plt.plot(t, trajectory_theta)
            # plt.xlabel("Time t")
            # plt.ylabel("Heading of Nodes ")
            # plt.title(f"Heading of Nodes {names[k]} ")
            k +=1
    plt.show()

if __name__ == "__main__":
    main()