import numpy as np 
from random import choice
import argparse
import networkx as nx

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def get_random_graph_nodes(G, no_samples):
    dofValues = []
    for i in range(no_samples):
        node = choice(list(G.nodes()))
        temp = G.node[node]['state']
        dv = state_to_numpy(temp)
        dofValues.append(dv)
    return dofValues

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()
    
    G = nx.read_graphml(args.graphfile) 

    e_dir = ["test_data/T1/4", "test_data/T1/12", "test_data/T1/16", "test_data/T1/18", "test_data/T2/4", "test_data/T2/12", "test_data/T2/16", "test_data/T2/18"]
    n_samples = [200, 300, 400, 500]

    for envdir in e_dir:
        for no_samples in n_samples:
            dofValues = get_random_graph_nodes(G, no_samples)
            np.savetxt(envdir + "/random_samples_"+str(no_samples)+".txt", np.array(dofValues), delimiter=" ", fmt="%s")

if __name__ == '__main__':
    main()