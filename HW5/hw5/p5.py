import cvxpy as cp
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

num = 5

D = nx.gnm_random_graph(num, (num -1)*(num-2)/2, directed=False)

m = 30
n = 20
np.random.seed(1)
A = np.random.randn(m, n)
b = np.random.randn(m)

# Construct the problem.
x = cp.Variable(n)
objective = cp.Minimize(cp.sum_squares(A @ x - b))
constraints = [0 <= x, x <= 1]
prob = cp.Problem(objective, constraints)

# The optimal objective value is returned by `prob.solve()`.
result = prob.solve()
# The optimal value for x is stored in `x.value`.
print(x.value)
# The optimal Lagrange multiplier for a constraint is stored in
# `constraint.dual_value`.
print(constraints[0].dual_value)

def main():
    nums = [5]
    for num in nums:
        names = ['cycle','path', 'star', 'complete']
        graphs = [nx.cycle_graph(num), nx.path_graph(num), nx.star_graph(num - 1), nx.complete_graph(num)]

        k =0
        for graph in graphs:
            t = np.linspace(0, 30, 101)
            trajectory_theta = odeint(get_xdot, list(nx.get_node_attributes(graph, "theta").values()), t, args=(graph, omega))
            plt.figure()
            plt.plot(t, trajectory_theta)
            plt.xlabel("Time t")
            plt.ylabel("Heading of Nodes ")
            plt.title(f"Heading of Nodes {names[k]} ")
            k +=1
    plt.show()

if __name__ == "__main__":
    main()