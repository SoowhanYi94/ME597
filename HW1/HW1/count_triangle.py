import util
def main(): 
    n_nodes = 5
    edge_probability = 2/n_nodes
    G = util.random_graph(edge_probability, n_nodes)
    A_G = util.nx.to_numpy_array(G)
    m = range(len(A_G))
    count =0
    for i in m:
        for j in m:
            for k in m:
                if A_G[i][j] and A_G[j][k] and A_G[k][i]:
                    count +=1

    print("number of trianlge: ", count/3)
    util.nx.draw(G, font_weight='bold', with_labels=True)
    util.plt.show()
    util.plt.savefig('graph.png')

if __name__=="__main__": 
    main() 