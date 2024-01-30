import random
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import animation
import numpy as np
from networkx.drawing.nx_agraph import graphviz_layout
import scipy as sp
def random_graphs_init(graph):
     
    x = np.array([[0 for x in range(2)] for y in range(len(graph))])
    
    for i in range(len(graph)):
        pos, vel = (random.randint(0,len(graph)), random.randint(0,len(graph)))
        graph.nodes[i]['pos_vel'] = (pos, vel)
        x[i][0] = pos
        x[i][1] = vel
    return graph, x
def get_input(x, G):
    k_p = 0.1
    k_v = 1
    k_f = 0
    u = np.array(np.zeros((x.shape[0],1)))
    
    for i in G.nodes():
            for j in G.neighbors(i):
                u[i] += k_p *(x[i,0] - x[j,0]) + k_v *(x[i, 1] - x[j, 1])
    
        
    return u
def get_xdot(x, t, G):
    num = int(len(x)/2)
    x = x.reshape(num, 2)
    A = [[0,1],[0,0]]
    B = np.array([[0],[1]])
    u = get_input(x, G)
    return (np.matmul(A,x.T) - B* u.T).reshape(num*2)

def get_lambda2(G):
    eigenvalues = nx.laplacian_spectrum(G)
    return eigenvalues
def main():
    plt.clf
    nums = [5]
    for num in nums : 
        graphs = [nx.dense_gnm_random_graph(num,2*num)] # random.randint(num//2, 2*num))
        # k = 0
        for graph in graphs:

            graph, x = random_graphs_init(graph)
            t = np.linspace(0,10,101)
            L_G = nx.laplacian_matrix(graph).toarray()
            trajectory = sp.integrate.odeint(get_xdot, np.reshape(x, 2*len(graph)), t, args=(graph, ))
            # eigenvalues[k].append( np.sort(get_lambda2(graph))[1])
            labels = []

           
            for i in range(2*num):
                if i%2 ==0:
                    plt.figure(1)
                    plt.plot(t, trajectory[:,i], label= f"$x{i//2+1}_p$")
                else:
                    plt.figure(2)
                    plt.plot(t, trajectory[:,i], label= f"$x{i//2+1}_v$")
            plt.figure(1)
            plt.xlabel("time t")
            plt.ylabel(f"position of nodes")
            plt.legend()
            plt.grid()
            plt.title(f"convergence of position of graph with {num} nodes")
            plt.figure(2)
            plt.xlabel("time t")
            plt.ylabel(f"velocity of nodes")
            plt.legend()
            plt.grid()
            plt.title(f"convergence of velocity of graph with {num} nodes")

    
    plt.show()

if __name__=="__main__": 
    main()  
 