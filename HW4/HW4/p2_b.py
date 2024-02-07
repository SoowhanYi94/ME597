import networkx as nx
import matplotlib.pyplot as plt

def main():
    n = 3
    G = nx.empty_graph(n, create_using=nx.DiGraph)
    G.add_edges_from([(0, 1), (0, 2)])
    pos = nx.spring_layout(G)
    plt.figure()
    nx.draw_networkx_nodes(G, pos, cmap=plt.get_cmap('jet'), node_size = 500)
    nx.draw_networkx_labels(G, pos)
    nx.draw_networkx_edges(G, pos,G.edges(),width=3,  arrows=True)
    G_reverse = nx.reverse(G)
    pos = nx.spring_layout(G_reverse)
    plt.figure()
    nx.draw_networkx_nodes(G_reverse, pos, cmap=plt.get_cmap('jet'), node_size = 500)
    nx.draw_networkx_labels(G_reverse, pos)
    nx.draw_networkx_edges(G_reverse, pos,G_reverse.edges(), width=3, arrows=True)
    plt.show()
if __name__ == "__main__":
    main()