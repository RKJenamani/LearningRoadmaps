import networkx as nx
import numpy as np
import argparse
from math import sqrt

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def get_path_length(G, path_nodes):
    config1 = state_to_numpy(G.node[path_nodes[0]]['state'])
    path_length = 0
    for i in range(1, len(path_nodes)):
        state = G.node[path_nodes[i]]['state']
        config2 = state_to_numpy(state)
        path_length += sqrt(np.sum((config2-config1)**2))
        config1 = config2
    return path_length

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    # parser.add_argument('--outputgraphfile',type=str,required=True)
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()
    orig_G = nx.read_graphml(args.graphfile)

    e_dir = ["test_data_11June/T1/4", "test_data_11June/T1/7", "test_data_11June/T2/0", "test_data_11June/T2/4", "test_data_11June/T2/7", "test_data_11June/T2/14"]

    for envdir in e_dir:
        p_node_file_addr = envdir + "/path_nodes.txt"
        path_nodes_all = []        
        with open(p_node_file_addr, 'r') as file:
            lines = file.readlines()
            for line in lines:
                node_nos = line.strip('\n')
                path_nodes_all.append(node_nos.split(","))
        path_length_halton = []
        for path_nodes in path_nodes_all:
            path_length_halton.append(get_path_length(orig_G, path_nodes))
        np.savetxt(envdir + "/halton_path_length.txt", path_length_halton)  

if __name__ == '__main__':
    main()