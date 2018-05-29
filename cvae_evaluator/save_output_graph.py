import networkx as nx 
import numpy as np
import argparse 
from math import sqrt

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def generate_graph(nodes): #nodes -> key, value pair, value => string with joint angles separated by ' '
    G = nx.Graph()
    for key in nodes:
        value = str(nodes[str(key)]) 
        G.add_node(str(key), state = value) 
    return G

def load_output_samples(file_addr, s_node_file_addr, g_node_file_addr, orig_G):
    nodes = {}
    i = 0
    with open(file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes["o"+str(i)] = line
            i += 1
    with open(s_node_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes[line] = orig_G.node[line]['state']
    with open(g_node_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes[line] = orig_G.node[line]['state']                
    print(nodes)
    return nodes

def connect_knn(G, K):
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)
        G1 = G.copy()

        for k in range(K):
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
            # print("connected edge from ",node, " to ",sn)
            G[node][sn]['weight'] = w
            G1.remove_node(sn)
    return G

if __name__ == '__main__':
    test_d = "_12"
    print("-----Loading "+test_d+"-----")
    parser = argparse.ArgumentParser(description='Evaluate Sample')
    parser.add_argument('--samplefile',type=str,required=True)
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    s_node_file_addr = "output_data/start_node"+test_d+".txt"
    g_node_file_addr = "output_data/goal_node"+test_d+".txt"

    orig_G = nx.read_graphml(args.graphfile)

    nodes = load_output_samples(args.samplefile, s_node_file_addr, g_node_file_addr, orig_G)
    G = generate_graph(nodes)
    G = connect_knn(G, 10)
    print("no of edges = ",len(list(G.edges())))
    nx.write_graphml(G, "output_graph"+test_d+".graphml")