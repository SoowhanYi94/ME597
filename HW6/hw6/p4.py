import cvxpy as cp
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

def main():
    num = 3
    graph = nx.path_graph(num)
    delta = .1
    for j in range(10):

        weight = {(0,1): 1+j, (1,2): 1}
        iteration = 1000 #number of iteration to simulate limit
        time_lim = iteration *delta
        nx.set_edge_attributes(graph, weight, 'weight')
        max_eigen_val = np.max(nx.laplacian_spectrum(graph, weight='weight'))
        L_w_G = nx.laplacian_matrix(graph, weight='weight').toarray()
        del_max_eig = round(delta*max_eigen_val,4)
        print(f"delta* max eigen value = {del_max_eig}")
        M_G = np.eye(num) - delta*L_w_G
        true_value = np.random.randint(0, 100, size = num)
        avg_true_value = np.average(true_value)
        H =  np.eye(num)
        estimates = []
        t = np.linspace(0, time_lim, iteration)
        print(list(weight.values()))
        for i in range(num):
            rand_noise = np.random.normal(0,1, size = num)
            z = H@true_value + rand_noise
            for k in range(iteration):
                z = M_G @z
                estimates.append(z)
            plt.figure(j)
            plt.plot(t, estimates)
            plt.plot(t, [avg_true_value]*iteration)
            plt.title(r"$\Delta  \rho (L(G)) =" f"{del_max_eig}$" ", weight = " f"{list(weight.values())}" f" $\Delta = {delta}$")
            estimates = []
        plt.show()

if __name__ == "__main__":
    main()

