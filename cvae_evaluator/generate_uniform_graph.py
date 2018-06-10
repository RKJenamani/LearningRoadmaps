import numpy as np 
import networkx as nx 
from math import sqrt

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def connect_knn(G, K):
    print("len(G) = ",len(list(G.nodes())))
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)
        G1 = G.copy()

        for k in range(K):
            # print("node = ",node)
            w = 1000000
            sn = None
            for node1 in G1.nodes():
                if(node == node1):
                    continue
                state1 = G1.node[node1]['state']
                conf1  = state_to_numpy(state1)
                if(calc_weight(conf, conf1) < w):
                    w = calc_weight(conf, conf1)
                    sn = node1

            # if(check_for_collision(node, sn)==1):
            G.add_edge(node, sn)
            # print("sn = ",sn)
            # print("connected edge from ",node, " to ",sn)
            G[node][sn]['weight'] = w
            G1.remove_node(sn)
    return G    

def main():
    n_samples = [200, 300, 400, 500]
    K = [5, 10, 15]

    for no_samples in n_samples:
            output_samples = []
            for i in range(no_samples):
                curr_sample = ""
                for j in range(7):
                    curr_sample += str(np.random.uniform(-1,1))+" "
                output_samples.append(curr_sample)
            G = nx.Graph()
            i = 0
            for sample in output_samples:
                G.add_node(str(i), state=sample)
                i += 1
            G = connect_knn(G, k)   
            print("Writing", no_samples, k)
            nx.write_graphml(G, "uniform_graph_"+str(no_samples)+".graphml")    

if __name__ == '__main__':
    main()