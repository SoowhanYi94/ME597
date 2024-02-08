import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d
def random_graphs_init(graph,num):
    poses = 0
    nx.set_node_attributes(graph,poses, "pos")
    # poses.append(0)
    graph.add_edges_from([(i, i+1) for i in range(len(graph.nodes()))])
    edges = 1
    nx.set_edge_attributes(graph, edges, "edge_length")
    print(nx.get_edge_attributes(graph, "edge_length"))
    # labels = []
    # nx.set_node_attributes(G, labels, "labels")
    # for edge in list(graph.edges()):
    #     p1, p2 = edge
    #     graph.edges[i]['length'] = np.abs(p1 - p2)
    return graph

def create_z_ref(num):
    #uniformly distributed
    z_ref = np.linspace(0, num-1, num)
    return z_ref

def single_xdot(x, t, D, z_ref,k):

    L_D_bar = nx.laplacian_matrix(D.to_undirected(reciprocal=False, as_view=False)).toarray()
    D_D = nx.incidence_matrix(D, oriented=True).toarray()
    # print(z_ref)

    return -k *np.matmul(L_D_bar, x) + k * np.matmul(D_D,z_ref)
def main():
    nums = [5]
    k = 1
    
    for num in nums:
        D =nx.empty_graph(num,create_using=nx.DiGraph())
        # D = nx.gnm_random_graph(num, (num -1)*(num-2)/2, directed=True)
        D = random_graphs_init(D,num)
        z_ref = create_z_ref(len(D.edges()))
        t = np.linspace(0, 100, 101)
       
        trajectory_x = odeint(single_xdot, list(nx.get_node_attributes(D, "pos").values()), t, args=(D, z_ref, k))
        print(D.edges())
        plt.figure()
        plt.plot(t, trajectory_x)
        plt.xlabel("Time t")
        plt.ylabel("Heading of Nodes ")
            
    plt.show()

if __name__ == "__main__":
    main()