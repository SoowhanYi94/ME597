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
    nums = [5, 10, 20 , 50]
    graph_names = ['cycle', 'path', 'star', 'complete']
    eigenvalues = [[] for y in range(len(graph_names))]
    for num in nums : 
        graphs = [nx.cycle_graph(num),nx.path_graph(num), nx.star_graph(num), nx.complete_graph(num)]
        k = 0
        for graph in graphs:

            graph, positions = random_graphs_init(graph)
            t = np.linspace(0,10,101)
            L_G = nx.laplacian_matrix(graph).toarray()
            trajectory = sp.integrate.odeint(get_xdot, np.reshape(positions, 2*len(graph)), t, args=(L_G, ))
            eigenvalues[k].append( np.sort(get_lambda2(graph))[1])
            labels = []
            for i in range(num):
                labels.append(f"x{i+1}")
            plt.figure()
            plt.plot(t, trajectory)
            plt.xlabel("time t")
            plt.ylabel(f"x and y position of nodes for {graph_names[k]}")
            plt.grid()
            plt.title(f"convergence of {graph_names[k]} graph with {num} nodes")
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
    ## 
    plt.xlabel("number_of_nodes")
    plt.ylabel('$\lambda_2$')
    plt.title("Convergence VS Number of nodes")
    plt.plot(nums, eigenvalues)
    plt.legend(graph_names)
    
    plt.show()

if __name__=="__main__": 
    main()  
 