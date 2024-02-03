import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d

def addEdge(G):
    # diameter = nx.diameter(G)
    start = G.nodes[0].copy()
    end = G.nodes[0].copy()
    max = 0    
    for node_i in G.nodes:
        G_temp = G.copy()
        for node_j in G.nodes:
            if node_i != node_j and ((node_i, node_j) not in G_temp.edges()):
                G_temp.add_edge(node_i, node_j)
                lambda2 = nx.laplacian_spectrum(G_temp)[1]
                if max < lambda2:
                    max = lambda2
                    start = node_i
                    end = node_j
    
    G.add_edge(start, end)
              
    return G


def main():
    nums = [10, 20, 30, 40, 50]#[10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    names = ['cycle','path', 'star']
    for num in nums:
        graphs = [nx.cycle_graph(num), nx.path_graph(num), nx.star_graph(num - 1),]#[nx.gnm_random_graph(num,  (num-1)*(num-2)/2)]
        # omega = [5, 0 ,5, 0, 0]
        k = 0
        for graph in graphs:
            lambdas = []
            num_edges = []
            print(len(graph.edges()))
            while(len(graph.edges()) < num*(num-1)/2):
                lambdas.append(nx.laplacian_spectrum(graph)[1])
                num_edges.append(len(graph.edges()))
                graph = addEdge(graph)
                print(len(graph.edges()))
            plt.figure()
            plt.plot(num_edges, lambdas)
            plt.xlabel("number of edges")
            plt.ylabel("$\lambda_2$")
            plt.title(f"number of nodes = {num} in {names[k]}")
            k +=1
    plt.show()

if __name__ == "__main__":
    main()