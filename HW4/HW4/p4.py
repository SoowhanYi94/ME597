import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d
import math



def show(graph): 
    plt.figure()
    poses = nx.get_node_attributes(graph, 'pos')
    nx.draw_networkx_edges(graph, pos = poses,edgelist=graph.edges(),arrows=True)
    nx.draw_networkx_nodes(graph, pos = poses, nodelist=graph.nodes() ,label=True)
    nx.draw_networkx_labels(graph, pos=poses)
    plt.show()
def random_graphs_init(graph,num):
    poses = {i: (np.random.randint(i+10),np.random.randint(i+10)) for i in range(num)}
    vels = {i: (0,0) for i in range(num)}
    nx.set_node_attributes(graph,poses, "pos")
    nx.set_node_attributes(graph, vels, "vel")
    graph.add_edges_from([(i, i+1) for i in range(len(graph.nodes())-1)])
    graph.add_edges_from([(i+1, i) for i in range(len(graph.nodes())-1)])
    graph.add_edges_from([(num-1, 0), (0, num -1)])

    return graph

def create_z_ref(num, int_type):
    #uniformly distributed
   
    match int_type:
        case 1:
             z_ref = np.linspace(0, num-1, 2*num)
        case 2:
             z_ref = [1,1,1,1,0,0,0,0]
        case _:
             z_ref = np.linspace(0, num-1, num)
    return z_ref

def single_xdot(x, t, D, z_ref,k):

    L_D_bar = nx.laplacian_matrix(D.to_undirected(reciprocal=False, as_view=False)).toarray()
    D_D = nx.incidence_matrix(D, oriented=True).toarray()
    return -k *np.matmul(L_D_bar, x) + k * np.matmul(D_D,z_ref)

num = 5
D =nx.empty_graph(num,create_using=nx.DiGraph())
# D = nx.gnm_random_graph(num, (num -1)*(num-2)/2, directed=True)
D = random_graphs_init(D,num)

def main():
    
    k = 1
    
    labels = []
    for i in range(num):
        labels.append(f"x{i}")
    
    t = np.linspace(0, 10, 101)
    ## Single
    z_ref = create_z_ref(len(D.nodes()),1)
    # np.append(list(nx.get_node_attributes(D, 'pos_x')),list(nx.get_node_attributes(D, 'pos_y')))
    l=[]
    [l.extend([v[0],v[1]]) for k,v in nx.get_node_attributes(D, 'pos').items()]
    print(f"input for odeint: {l}")
    trajectory_y = odeint(single_xdot,l, t, args=(D, z_ref, k))
    plt.figure()
    plt.plot(t, trajectory_y[:,:num], label = labels)
    plt.xlabel("Time t")
    plt.ylabel("Position")
    plt.title("Double Integrater")
    plt.figure()
    plt.plot(t, trajectory_y[:,num:], label = labels)
    plt.xlabel("Time t")
    plt.ylabel("Velocity")
    plt.title("Double Integrater")
    plt.figure()
    plt.plot( trajectory_y[:,num:],trajectory_y[:,:num], label = labels)
    plt.plot(z_ref[num:], z_ref[:num])
    plt.xlabel("Time t")
    plt.ylabel("Velocity")
    plt.title("Double Integrater")
    show(D)
    plt.show()
if __name__ == "__main__":
    main()