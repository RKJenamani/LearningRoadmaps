import networkx as nx
import numpy as np
import argparse 

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def create_array(G):
    array = []
    for node in G.nodes():
        curr_node = G.node[node]['state']
        conf = state_to_numpy(curr_node)
        array.append(conf)
    return np.array(array)  

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    G = nx.read_graphml(args.graphfile)
    array = create_array(G)
    np.savetxt("DOF_Values_enum_nodes.txt", array, delimiter=" ", fmt="%s")

if __name__ == '__main__':
    main()  