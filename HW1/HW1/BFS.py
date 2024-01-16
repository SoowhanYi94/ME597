

##########
# 1. Write a code that implements breadth first search to determine if two nodes in a graph are connected by a walk. The input to the code is a list adjacency of the graph (so a dictionary that has the list of nodes and and their respective neighbors), and the desired pair of nodes. The output of the code should come back with the answer that lists either the sequence of adjacent nodes that allows a walk from one node to the next or a message that these two nodes are not connected.  Show how the algorithm works on a few generated graphs on five nodes.
##########
"""Breath First Search
    Args:
        G (nx.Digraph): an directed graph with n nodes. 

        desired_nodes ([nx.Digraph.node, nx.Digrapoh.node]):  
        [0]: source node, [1]: destination node

    Returns:
        Tuple(np.list, np.list):
        [0]: Either the sequence of walk to destination or the trial path with statement for failure if so. 
        [1]: list of nodes for BFS
    """
import util
def bfs_search(G, desired_nodes):
    source = desired_nodes[0]
    destination = desired_nodes[1]
    stateQueue = util.Queue()
    pathQueue= util.Queue()
    visitedSet = []
    visitedSet = set(visitedSet)
    pathList = []
    while source != destination:
        if source not in visitedSet:       
            visitedSet.add(source)
            successors = G[source]
            for successor in successors:
                stateQueue.push(successor)
                pathQueue.push(pathList + [[source, successor]])
        if stateQueue.isEmpty(): 
            print("state queue empty two nodes are not connected.")
            break
        source = stateQueue.pop()
        pathList = pathQueue.pop()
    nodeList = util.np.zeros(len(pathList)+1)
    nodeList[0] = pathList[0][0]
    for i in range(0, len(pathList)):
        nodeList[i+1] = pathList[i][1]
    return pathList, nodeList

def update(num, layout, G, desired_nodes):

    source = desired_nodes[0]
    destination = desired_nodes[1]
    stateQueue = util.Queue()
    pathQueue= util.Queue()
    visitedSet = []
    visitedSet = set(visitedSet)
    pathList = []
    while source != destination:
        if source not in visitedSet:       
            visitedSet.add(source)
            successors = G[source]
            for successor in successors:
                stateQueue.push(successor)
                pathQueue.push(pathList + [[source, successor]])
        if stateQueue.isEmpty(): 
            print("state queue empty two nodes are not connected.")
            break
        source = stateQueue.pop()
        pathList = pathQueue.pop()
        nodeList = util.np.zeros(len(pathList)+1)
        nodeList[0] = pathList[0][0]
        for i in range(0, len(pathList)):
            nodeList[i+1] = pathList[i][1]
        util.nx.draw_networkx_nodes(G, pos=layout, nodelist=nodeList[:num+1], node_color=['r'])
        # if source == destination:
        #     util.nx.draw_networkx_nodes(G, pos=layout, nodelist=[destination], node_color=['r'])
        util.plt.savefig('/home/syi/Wi 24 UW/ME597/pdf/img/frame{}.png'.format(num))
        
    # Draw the graph with random node colors
    # random_colors = util.np.random.randint(2, size=15)
    # get_colors = lambda n: ["#%06x" % util.random.randint(0, 0xFFFFFF) for _ in range(n)]
    colors = ['r', 'b', 'g', 'y', 'w', 'm']
    # path = [node_list[:i] for i in num]
    # query_nodes = util.nx.draw_networkx_nodes(G, pos=layout, nodelist=nodeList[:num+1], node_color=['r'])
    # util.plt.savefig('/home/syi/Wi 24 UW/ME597/pdf/img/frame{}.png'.format(num))
    # util.nx.draw_networkx_nodes(G, pos=layout, node_color=get_colors(len(node_list)))

    # Set the title
    # ax.set_title("Frame {}".format(num))

#   return nodes,

def main():
    n_nodes = 10
    edge_probability = 2/n_nodes
    G = util.random_graph(edge_probability, n_nodes)
    source = 2
    destination = 0
    
    desired_nodes =[source, destination]
    fig  = util.plt.figure(figsize=(8,8))
    nc =    util.np.random.random(3)
    nodes = util.nx.draw(G,pos= util.graphviz_layout(G),node_color=nc, with_labels=True)
    edges = util.nx.draw(G,pos= util.graphviz_layout(G), with_labels=True) 
    layout = util.graphviz_layout(G)
    path_list, node_list = bfs_search(G, desired_nodes)
    anim = util.FuncAnimation(fig, update, interval=60, save_count=len(node_list), fargs = (layout, G, desired_nodes))
    #util.nx.draw(G, font_weight='bold', with_labels=True)
    # util.plt.show()
    writervideo = util.animation.FFMpegWriter(fps=1) 
    anim.save('/home/syi/Wi 24 UW/ME597/pdf/img/increasingStraightLine.mp4', writer=writervideo) 
    util.plt.close() 
if __name__=="__main__": 
    main() 