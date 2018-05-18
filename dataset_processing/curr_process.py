import os
import argparse
import networkx as nx
import math        
import numpy as np
from itertools import islice, chain

k = 1

#return k shortest path
def k_shortest_paths(G, source, target, weight=None):
    global k
    return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))

def remove_invalid_edges(G1, binary_vec):
    to_remove = []
    
    for i, edge in enumerate(G1.edges()):
        if(not binary_vec[i]):
            u,v = edge
            to_remove.append((u, v))

    for r in to_remove:
        G1.remove_edge(r[0], r[1])        
    return G1

def write_to_file(directory, all_paths):
    # print(all_paths)
    for i in range(len(all_paths)):
        print(i,all_paths[i])
    with open(directory + "/path_nodes_no.txt", 'w') as file:
        file.writelines(','.join(str(j) for j in i) + '\n' for i in all_paths)

def process_it(G, directory):
    print("processing data")
    G1 = G.copy()
    start = np.loadtxt(directory+"/start_node_no.txt")
    goal = np.loadtxt(directory+"/goal_node_no.txt")
    binary_vec = np.loadtxt(directory+"/binary_vec.txt")

    # G1 = remove_invalid_edges(G1, binary_vec)
    all_paths = []
    enum_nodes = list(G.nodes())
    print("enum_nodes = ", enum_nodes)
    for i in range(50):
        src = int(start[i])
        gl = int(goal[i])
        paths = []
        try:
            s_node = enum_nodes[src]
            g_node = enum_nodes[gl]
            print("searching path from ")
            paths = k_shortest_paths(G1, s_node, g_node)
            # print("paths = ",paths)
            paths_node = list(chain.from_iterable(paths))
            paths_node_no = [enum_nodes.index(n) for n in paths_node]
            all_paths.append(paths_node_no)
        except Exception as e:
            print(e)
            all_paths.append(['-1'])
            
    # print(all_paths) 
    # print("\n\n\n\n") 
    # print(len(all_paths))      
    write_to_file(directory, all_paths)  
    print("written to directory = ",directory)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--datadir',type=str,required=True)
    args = parser.parse_args()

    G = nx.read_graphml(args.graphfile)
    data_dir = args.datadir

    process_it(G, data_dir)    