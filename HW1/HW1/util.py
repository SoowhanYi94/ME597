import random
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import animation
import numpy as np
from networkx.drawing.nx_agraph import graphviz_layout
class Queue:
    "A container with a first-in-first-out (FIFO) queuing policy."
    def __init__(self):
        self.list = []

    def push(self,item):
        "Enqueue the 'item' into the queue"
        self.list.insert(0,item)

    def pop(self):
        """
          Dequeue the earliest enqueued item still in the queue. This
          operation removes the item from the queue.
        """
        return self.list.pop()

    def isEmpty(self):
        "Returns true if the queue is empty"
        return len(self.list) == 0


def random_graph(prob =0.3, n_nodes = 10):

    G = nx.DiGraph()

    G.add_nodes_from(range(n_nodes))

    for u in G.nodes:
        for v in G.nodes:
            if random.random() < prob and u!=v :
                G.add_edge(u, v)
    return G

