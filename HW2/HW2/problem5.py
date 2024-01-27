import random
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import animation
import numpy as np
from networkx.drawing.nx_agraph import graphviz_layout
import scipy as sp
def random_graphs_init(graph):
     
    x = [[0 for x in range(2)] for y in range(len(graph))]
    for i in range(len(graph)):
        pos, vel = (random.randint(0,len(graph)), random.randint(0,len(graph)))
        graph.nodes[i]['pos_vel'] = (pos, vel)
        x[i][0] = pos
        x[i][1] = vel
    return graph, x
def get_input(x, G):
    k_p = 1  
    k_v = 1
    k_f = 1.0
    u = np.zeros_like(x)
    print(u.shape)

    for i in G.nodes():
        for j in G.neighbors(i):
            u[i] += -(k_p *(x[i,0] - x[j,0]) - k_v *(x[i,1] - x[j,1]))
    
    # u += k_f * np.sum(x[:,1] - x[:,1])
    return u
def get_xdot(x, t, G):
    num = int(len(x)/2)
    x = x.reshape(num, 2)
   
    A = [[0,1],[0,0]]
    B = [0,1]
    u = get_input(x, G)
    
    return (np.matmul(A,x.T) - np.matmul(B, u.T)).reshape(num*2)

def get_lambda2(G):
    eigenvalues = nx.laplacian_spectrum(G)
    return eigenvalues
def main():
    nums = [10]
    for num in nums : 
        graphs = [nx.dense_gnm_random_graph(num, random.randint(num//2, 2*num))]# [nx.cycle_graph(num),nx.path_graph(num), nx.star_graph(num), nx.complete_graph(num)]
        # k = 0
        for graph in graphs:

            graph, x = random_graphs_init(graph)
            t = np.linspace(0,10,101)
            L_G = nx.laplacian_matrix(graph).toarray()
            trajectory = sp.integrate.odeint(get_xdot, np.reshape(x, 2*len(graph)), t, args=(graph, ))
            # eigenvalues[k].append( np.sort(get_lambda2(graph))[1])
            print(len(trajectory[:,1]))
            labels = []


            for i in range(num):
                labels.append(f"x{i+1}")
            plt.figure()
            plt.plot(t, trajectory)

            plt.xlabel("time t")
            plt.ylabel(f"position and velocity of nodes")
            plt.grid()
            plt.title(f"convergence of position and velocity of graph with {num} nodes")

    
    plt.show()

if __name__=="__main__": 
    main()  
 