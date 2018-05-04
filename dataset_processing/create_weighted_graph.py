import networkx as nx
import numpy
from math import sqrt

def calc_weight(s, g):
    return sqrt(numpy.sum((s-g)**2))

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return numpy.array(val_list)    

def main():
    G = nx.read_graphml("graphs/herb_halton_1.graphml")
    for i, edge in enumerate(G.edges()):
        u,v = edge
        state1 = state_to_numpy(G.node[u]['state'])
        state2 = state_to_numpy(G.node[v]['state'])
        G[u][v]['weight'] = calc_weight(state1, state2)

    nx.write_graphml(G, "graphs/weighted_graph.graphml")    

if __name__=='__main__':
    main()