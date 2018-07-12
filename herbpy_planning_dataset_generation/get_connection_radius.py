import networkx as nx
import numpy as np
import argparse
from math import sqrt

def calc_weight(conf1, conf2):
    return sqrt(np.sum((conf2-conf1)**2))

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list) 

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)

    args = parser.parse_args()

    G = nx.read_graphml(args.graphfile)
    max_w = 0

    for edge in G.edges():
        u, v = edge
        conf1 = state_to_numpy(G.node[u]['state'])
        conf2 = state_to_numpy(G.node[v]['state'])
        w = calc_weight(conf1, conf2)
        if(w>max_w):
            max_w = w
    print("max_w = ", max_w)

if __name__ == '__main__':
    main()