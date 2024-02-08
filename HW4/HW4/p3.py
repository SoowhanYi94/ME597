import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from mpl_toolkits import mplot3d
def assess_x_1(U, x_0):
    delta_t = np.log(2/3)
    poses = np.matmul(np.exp(-delta_t *nx.laplacian_matrix(U).toarray()),x_0.T)\
    
    print(np.exp(-delta_t *nx.laplacian_matrix(U).toarray()))
    print(poses)
def main():

    n = 5
    G = nx.Graph()
    G.add_node(0)
    H = nx.complete_graph(n-1)
    U = nx.disjoint_union(G, H)
    pos = nx.spring_layout(U)
    U.add_edges_from([(0,1)])
    Delta = 1
    x_0 = np.zeros(n)
    x_0[0] = Delta *2
    x_0[1] = Delta
    
 
    assess_x_1(U, x_0)
    plt.figure()
    nx.draw_networkx_nodes(U, pos, cmap=plt.get_cmap('jet'), node_size = 500)
    nx.draw_networkx_labels(U, pos)
    nx.draw_networkx_edges(U, pos,U.edges(),width=2,  arrows=True)
    plt.show()
if __name__ == "__main__":
    main()


