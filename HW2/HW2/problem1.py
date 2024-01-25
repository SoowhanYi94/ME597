import random
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import animation
import numpy as np
from networkx.drawing.nx_agraph import graphviz_layout

def random_graphs_init(graph):
    positions = []
    for i in graph.nodes():
        position = (random.randint(0,len(graph)), random.randint(0,len(graph)))
        graph.nodes[i]['pos'] = position
        positions.append([position[0], position[1]])
    return graph, positions

def get_xdot(L_G, positions):
    return -np.matmul(L_G,positions)

def update(num, G, positions):
    plt.clf()
    nx.draw_networkx_nodes(G, pos = nx.get_node_attributes(G, 'pos'))
    plt.savefig('/home/syi/Wi 24 UW/ME597/HW2/pdf/img/frame{}.png'.format(num))
    # undraw = nx.draw_networkx_nodes(G, pos = nx.get_node_attributes(G, 'pos'), node_color= 'white')
    # undraw.set_edgecolor('white')
    # null_nodes = nx.draw_networkx_nodes(G, pos=pos, nodelist=set(G.nodes()) - set(path), node_color="white",  ax=ax)
    for i in graph.nodes():
        position = positions[i] + x_dot[i]
        graph.nodes[i]['pos'] = position
        positions[i] = positions[i] + x_dot[i]
num = 5
graphs = [nx.cycle_graph(num),nx.path_graph(num), nx.star_graph(num), nx.complete_graph(num)]
graphs = [nx.cycle_graph(num)]
for i in range(len(graphs)):
    graph = graphs[i]
    graph, positions = random_graphs_init(graph)
    fig  = plt.figure(figsize=(8,8))
    nc =    np.random.random(3)
    x_dot = get_xdot(nx.laplacian_matrix(graph).toarray(), positions)
    # positions = [[]]
    
    anim = FuncAnimation(fig, update, interval=60, save_count=30, fargs = (graph, positions))
#nx.draw(G, font_weight='bold', with_labels=True)
# plt.show()
    writervideo = animation.FFMpegWriter(fps=1) 
    anim.save('/home/syi/Wi 24 UW/ME597/HW2/pdf/img/sample{}.mp4'.format(i), writer=writervideo) 
    plt.close() 
    plt.show()


    # x_dot = -L_G * G.nodes()["position"]
    # print(x_dot)
    # e = np.linalg.eigvals(L_G)
    # e.sort()
    # print(e[1])
    # print(G.nodes(data=True))
    # print(G.edges)
    # print(L_G). Find the second smallest eigenvalues of these graphs and see if the convergence properties of consensus  can be matched to these second smallest eigenvalues. Then explore the convergence as a function the number  nodes in these graphs-
