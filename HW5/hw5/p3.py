import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d

def show(graph): 
    plt.figure()
    poses = nx.get_node_attributes(graph, 'pos')
    nx.draw_networkx_edges(graph, pos = poses,edgelist=graph.edges(),arrows=True)
    nx.draw_networkx_nodes(graph, pos = poses, nodelist=graph.nodes() ,label=True)
    nx.draw_networkx_labels(graph, pos=poses)
    plt.show()

def random_graphs_init(graph,num, Delta):
    poses = {i: np.random.randint(0,Delta) for i in range(num)} # starts inside Delta
    # poses = {i: np.random.randint(0,10*Delta) for i in range(num)} # starts outside of Delta
    # poses_= []
    # for i in range(num):
    #     if i == num -1:
    #         poses_.append(np.max(poses_) + Delta +10)
    #     else :
    #         poses_.append(np.random.randint(0,Delta))
    # poses = {i: poses_[i] for i in range(num)} # one element just outside of Delta
    nx.set_node_attributes(graph,poses, "pos")
    edges = {edge: np.abs(graph.nodes[edge[0]]["pos"]-graph.nodes[edge[1]]["pos"])  for edge in graph.edges()}
    desired_l = 0
    nx.set_edge_attributes(graph, edges, "edge_length")
    
    return graph, desired_l
num = 5
k = 1
Delta = 10
labels = []
D = nx.gnm_random_graph(num, (num -1)*(num-2)/2, directed=False)
D,desired_l = random_graphs_init(D,num, Delta)
D = nx.minimum_spanning_tree(D)

def xdot(x, t,desired_l):
    poses = {i: x[i] for i in range(num)}
    nx.set_node_attributes(D, poses, "pos")
    edges = {edge: np.abs(D.nodes[edge[1]]["pos"]-D.nodes[edge[0]]["pos"])  for edge in D.edges()}
    nx.set_edge_attributes(D, edges, "edge_length")
    ret  = []
    for i in range(num):
        dotx = 0
        for j in nx.neighbors(D, i):
            if ((i,j) in edges) :
                numerator = 2*(Delta - np.abs(desired_l )) - np.abs(edges[(i,j)] - desired_l )
                denominator = (Delta - np.abs(desired_l ) - np.abs(edges[(i,j)] - desired_l ))**2
                rest = D.nodes[i]["pos"] - D.nodes[j]["pos"] - desired_l 
                dotx += numerator/denominator *rest
            elif ((j,i) in edges):
                numerator = 2*(Delta - np.abs(desired_l )) - np.abs(edges[(j,i)] - desired_l )
                denominator = (Delta - np.abs(desired_l ) - np.abs(edges[(j,i)] - desired_l ))**2
                rest = D.nodes[i]["pos"] - D.nodes[j]["pos"] - desired_l 
                dotx += numerator/denominator *rest
        ret.append(-dotx)

    return ret

def main():
    
    for i in range(num):
        labels.append(f"x{i}")
    t = np.linspace(0, 40,num= 1001)
    
    l=[]
    [l.extend([v]) for k,v in nx.get_node_attributes(D, 'pos').items()]

    trajectory_x = odeint(xdot, l, t, args=(desired_l, ))
    plt.figure()
    plt.plot(t, trajectory_x, label = labels)

    plt.xlabel("Time t")
    plt.ylabel("Position")
    plt.title("one element just outside of Delta")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
