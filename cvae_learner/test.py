import networkx as nx
import numpy as np
import argparse
import time
from math import sqrt

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def state_to_numpy(state):
#     print("state = ", state)
    strlist = state.split()
#     print("strlist = ", strlist)
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def connect_knn(G, K):
    print("no of nodes = ", len(list(G.nodes())))
    print("gotta connect knn")
    
    G1 = G.copy()
    nodes = []
    states = []
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)

        for k in range(K):
            w = 1000000
            sn = None
            for node1 in G1.nodes():
            	if(node == node1):
            		continue
                state1 = G1.node[node1]['state']
#                 print("node1 = ",node1)
#                 print("state1 = ",state1)
                
                conf1  = state_to_numpy(state1)
                w1 = calc_weight(conf, conf1)
                if(w1 < w):
                    w = w1
                    sn = node1

            # if(check_for_collision(node, sn)==1):
            G.add_edge(node, sn)
            nodes.append(sn)
            states.append(G1.node[sn]['state'])
            G1.remove_node(sn)

        for i in range(len(nodes)):
            G1.add_node(nodes[i], state = states[i])
        nodes = []
        states = []
    print("connected knn")
    return G

def main():
    k = 5

    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)

    args = parser.parse_args()

    G = nx.read_graphml(args.graphfile)

    t1 = time.time()
    G.remove_edges_from(list(G.edges()))
    t2 = time.time()
    print("removed "+`len(list(G.edges()))`+" edges in time t = "+`t2-t1`)

    t1 = time.time()
    G = connect_knn(G, k)
    t2 = time.time()
    print("connected "+`k`+" NN in time t = "+`t2-t1`)

if __name__ == '__main__':
    main()