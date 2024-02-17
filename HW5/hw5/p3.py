import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d

def construct_W(graph, Delta, desired_l, num):
    Delta = [Delta]*((num -1)*(num-2)//2)
    desired_l = np.abs(desired_l)
    l_ij = []
    [l_ij.extend(v) for k,v in nx.get_edge_attributes(graph, 'edge_length').items()]
    W = np.diag((2*(Delta-desired_l+l_ij)/(Delta-desired_l+l_ij)**2))
    return W
def show(graph): 
    plt.figure()
    poses = nx.get_node_attributes(graph, 'pos')
    nx.draw_networkx_edges(graph, pos = poses,edgelist=graph.edges(),arrows=True)
    nx.draw_networkx_nodes(graph, pos = poses, nodelist=graph.nodes() ,label=True)
    nx.draw_networkx_labels(graph, pos=poses)
    plt.show()
def random_graphs_init(graph,num, Delta):
    poses = {i: (np.random.randint(0,i+Delta), np.random.randint(0,i+Delta)) for i in range(num)}
    # print(poses)
    # vels = 1
    nx.set_node_attributes(graph,poses, "pos")
    # print(nx.get_node_attributes(graph, 'pos'))
    # nx.set_node_attributes(graph, vels, "vel")
    # poses.append(0)
    # graph.add_edges_from([(i, i+1) for i in range(len(graph.nodes())-1)])

    edges = {edge:{np.sqrt((graph.nodes[edge[1]]["pos"][0] - graph.nodes[edge[0]]["pos"][0])**2 + (graph.nodes[edge[1]]["pos"][1] - graph.nodes[edge[0]]["pos"][1])**2)}  for edge in graph.edges()}
    # for edge in graph.edges():
    #     print(graph.nodes[edge[1]]['pos'])
        # print(edge[0])
    nx.set_edge_attributes(graph, edges, "edge_length")
    return graph
def xdot(x, t,desired_l):

    
    W = construct_W(D, Delta, desired_l, num)
    D_D = nx.incidence_matrix(D, oriented=True).toarray()
    # print(W)
    # print(D_D)
    # print(D_D@W@D_D.T)
    
    poses = {i: (x[2*i], x[2*i+1]) for i in range(num)}
    # print(poses)
    # poses = {i: {"pos": (x[2*i], x[2*i+1])} for i in range(len(x)//2)}
    nx.set_node_attributes(D, poses, "pos")
    # poses = nx.get_node_attributes(D, "pos")
    # print(poses[1])
    # print(list(D.nodes[1]["pos"].values())[0])
    edges = {edge:{np.sqrt((D.nodes[edge[1]]["pos"][0] - D.nodes[edge[0]]["pos"][0])**2 + (D.nodes[edge[1]]["pos"][1] - D.nodes[edge[0]]["pos"][1])**2)}  for edge in D.edges()}
# for edge in graph.edges():
#     print(graph.nodes[edge[1]]['pos'])
    # print(edge[0])
    nx.set_edge_attributes(D, edges, "edge_length")
    
    l_ij = []
    [l_ij.extend(v) for k,v in nx.get_edge_attributes(D, 'edge_length').items()]
    # print(D_D@(list(np.array(l_ij)- np.array(desired_l))))
    dxdt = ( D_D@W@D_D.T) @ (D_D@(list(np.array(l_ij)- np.array(desired_l))))
    # print(np.kron(dxdt, [1,1]))
    # print(np.kron(dxdt, np.eye(2)).reshape(len(dxdt)*4))
    return -(np.kron(dxdt, [1,1]))
num = 5
k = 1
Delta = 57
labels = []
D = nx.gnm_random_graph(num, (num -1)*(num-2)/2, directed=True)
D = random_graphs_init(D,num, Delta)
def main():
    
    for i in range(num):
        labels.append(f"x{i}")
    # D =nx.empty_graph(num,create_using=nx.Graph())
    
    # z_ref = create_z_ref(len(D.edges()),1)
    t = np.linspace(0, 10,num= 10)
    
    desired_l = [np.random.randint(0,i+1) for i in range((num -1)*(num-2)//2)]
    l=[]
    [l.extend([v[0],v[1]]) for k,v in nx.get_node_attributes(D, 'pos').items()]
    # print(l)
    # print(list(nx.get_node_attributes(D, "pos").values()))
    trajectory_x = odeint(xdot, l, t, args=(desired_l, ))
    for i in range(num):
        plt.figure(1)
        plt.plot(t, trajectory_x[:,2*i])#, label = labels)
        plt.figure(2)
        plt.plot(t, trajectory_x[:, 2*i + 1])
        plt.figure(3)
        plt.plot( trajectory_x[:,i], trajectory_x[:, i + 1])
    # plt.figure()
    # plt.plot(t, trajectory_x[:,num:])#, label = labels)
    # plt.figure()
    # plt.plot( trajectory_x[:,num:], trajectory_x[:,:num])
    # plt.plot(t, trajectory_x[:,2])#, label = labels)
    # plt.plot(t, trajectory_x[:,4])#, label = labels)
    # plt.plot(t, trajectory_x[:,6])#, label = labels)
    # plt.plot(t, trajectory_x[:,8])#, label = labels)

    plt.xlabel("Time t")
    plt.ylabel("Position")
    plt.title("Single Integrater")
        # z_ref = create_z_ref(len(D.edges()),2)
        # print(z_ref)
        # pos_vel = []
        # for i in range(len(D.nodes())):
        #     pos_vel.append(nx.get_node_attributes(D, "pos")[i])
        #     pos_vel.append(nx.get_node_attributes(D, "vel")[i])
        # pos_vel = np.append(list(nx.get_node_attributes(D, "pos").values()), list(nx.get_node_attributes(D, "vel").values()))
        # trajectory_double = odeint(double_xdot, pos_vel , t, args=(D, z_ref, k))
        # plt.figure()
        # plt.plot(t, trajectory_double[:,:num], label = labels)
        # plt.xlabel("Time t")
        # plt.ylabel("Position")
        # plt.title("Double Integrater")
        # plt.figure()
        # plt.plot(t, trajectory_double[:,num:], label = labels)
        # plt.xlabel("Time t")
        # plt.ylabel("Velocity")
        # plt.title("Double Integrater")
        # z_ref = create_z_ref(len(D.edges()),3)
        # trajectory_LTI = odeint(LTI_xdot, list(nx.get_node_attributes(D, "pos").values()) , t, args=(D, z_ref, k))
        # plt.figure()
        # plt.plot(t, trajectory_LTI, label = labels)
        # plt.xlabel("Time t")
        # plt.ylabel("Position")
        # plt.title("LTI")
    # plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
