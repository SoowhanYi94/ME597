import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d
def random_graphs_init(graph,num):
    poses = 0
    vels = 1
    nx.set_node_attributes(graph,poses, "pos")
    nx.set_node_attributes(graph, vels, "vel")
    # poses.append(0)
    graph.add_edges_from([(i, i+1) for i in range(len(graph.nodes())-1)])

    edges = {(i, i+1):{graph.nodes[i+1]['pos'] - graph.nodes[i]['pos']}  for i in range(len(graph.nodes())-1) }
    nx.set_edge_attributes(graph, edges, "edge_length")
    return graph

def create_z_ref(num, int_type):
    #uniformly distributed
   
    match int_type:
        case 1:
             z_ref = np.linspace(0, num-1, num)
        case 2:
             z_ref = [1,1,1,1,0,0,0,0]
        case _:
             z_ref = np.linspace(0, num-1, num)
    return z_ref

def single_xdot(x, t, D, z_ref,k):

    L_D_bar = nx.laplacian_matrix(D.to_undirected(reciprocal=False, as_view=False)).toarray()
    D_D = nx.incidence_matrix(D, oriented=True).toarray()
    return -k *np.matmul(L_D_bar, x) + k * np.matmul(D_D,z_ref)

def double_xdot(x, t, D, z_ref,k):
    k = 0.1
    L_D_bar = nx.laplacian_matrix(D.to_undirected(reciprocal=False, as_view=False)).toarray()
    D_D = k*nx.incidence_matrix(D, oriented=True).toarray()
    
    L_D_bar = -k *np.kron([[0,0], [1,1]] ,L_D_bar) + np.kron([[0,1], [0,0]], np.eye(len(x)//2))
    
   
    D_D = k*np.kron([[0,0], [1,1]] ,D_D) 
    
    return np.matmul(L_D_bar, x) + np.matmul(D_D,z_ref)

def LTI_xdot(x, t, D, z_ref,k):
    a = .1
    b = 1
    z_t = np.matmul(nx.incidence_matrix(D, oriented=True).toarray().T, x)
    u = k*np.matmul(nx.incidence_matrix(D, oriented=True).toarray(), (z_ref - z_t)) 
    return a*x + b*u
def main():
    nums = [5]
    k = 1
    
    for num in nums:
        labels = []
        for i in range(num):
            labels.append(f"x{i}")
        D =nx.empty_graph(num,create_using=nx.DiGraph())
        # D = nx.gnm_random_graph(num, (num -1)*(num-2)/2, directed=True)
        D = random_graphs_init(D,num)
        # z_ref = create_z_ref(len(D.edges()),1)
        t = np.linspace(0, 30, 101)
       
        # trajectory_x = odeint(single_xdot, list(nx.get_node_attributes(D, "pos").values()), t, args=(D, z_ref, k))
        # plt.figure()
        # plt.plot(t, trajectory_x, label = labels)
        # plt.xlabel("Time t")
        # plt.ylabel("Position")
        # plt.title("Single Integrater")
        z_ref = create_z_ref(len(D.edges()),2)
        print(z_ref)
        pos_vel = []
        for i in range(len(D.nodes())):
            pos_vel.append(nx.get_node_attributes(D, "pos")[i])
            pos_vel.append(nx.get_node_attributes(D, "vel")[i])
        # pos_vel = np.append(list(nx.get_node_attributes(D, "pos").values()), list(nx.get_node_attributes(D, "vel").values()))
        trajectory_double = odeint(double_xdot, pos_vel , t, args=(D, z_ref, k))
        plt.figure()
        plt.plot(t, trajectory_double[:,:num], label = labels)
        plt.xlabel("Time t")
        plt.ylabel("Position")
        plt.title("Double Integrater")
        plt.figure()
        plt.plot(t, trajectory_double[:,num:], label = labels)
        plt.xlabel("Time t")
        plt.ylabel("Velocity")
        plt.title("Double Integrater")
        # z_ref = create_z_ref(len(D.edges()),3)
        # trajectory_LTI = odeint(LTI_xdot, list(nx.get_node_attributes(D, "pos").values()) , t, args=(D, z_ref, k))
        # plt.figure()
        # plt.plot(t, trajectory_LTI, label = labels)
        # plt.xlabel("Time t")
        # plt.ylabel("Position")
        # plt.title("LTI")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()