import random
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import animation
import numpy as np
from networkx.drawing.nx_agraph import graphviz_layout
import scipy as sp

def random_graphs_init(graph):
     
    positions = [[0 for x in range(2)] for y in range(len(graph))]
    for i in range(len(graph)):
        position = (random.randint(0,len(graph)), random.randint(0,len(graph)))
        graph.nodes[i]['pos'] = position
        positions[i][0] = position[0]
        positions[i][1] = position[1]
    return graph, positions

def get_xdot(positions, t, L_G):
    num = int(len(positions)/2)
    positions = positions.reshape(num, 2)
    return (-np.matmul(L_G,positions)).reshape(num*2)

def get_lambda2(G):
    eigenvalues = nx.laplacian_spectrum(G)
    return eigenvalues
def main():
    edges = [5,6,7,8,9,10]
    Gs = []
    num = 5
    for edge in edges:
        Gs.append(nx.gnm_random_graph(num, edge))
    eigvalues= [[] for y in range(len(edges))]
    k  = 0
    for graph in Gs:
        graph, positions = random_graphs_init(graph)
        t = np.linspace(0,10,101)
        L_G = nx.laplacian_matrix(graph).toarray()
        trajectory = sp.integrate.odeint(get_xdot, np.reshape(positions, 2*len(graph)), t, args=(L_G, ))
        eigvalues[k].append(nx.laplacian_spectrum(graph)[1]) 
        labels = []
        for i in range(num):
            labels.append(f"x{i+1}")
        num_edges = len(graph.edges())
        plt.figure()
        plt.plot(t, trajectory)
        plt.xlabel("time t")
        plt.ylabel(f"x and y position of nodes for the graph")
        plt.grid()
        plt.title(f"convergence of the graph with {num_edges} edges")
        plt.figure()
        for i in range(num):
            plt.plot(trajectory[:,2*i], trajectory[:,2*i+1],label = labels[i] )
        plt.plot()
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend()
        plt.grid()
        plt.title("x-y position")
        k+=1
    plt.figure()
    plt.plot(edges, eigvalues)
    plt.xlabel("probability of forming edge between 2 nodes")
    plt.ylabel("$\lambda_2$")
    plt.legend()
    plt.grid()
    plt.title("probabilty vs eigenvalues")
    ## 
    
    plt.show()

if __name__=="__main__": 
    main()  
 