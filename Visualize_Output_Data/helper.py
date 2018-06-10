import numpy as np
import networkx as nx
from itertools import chain

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list) 

def get_eepositions(G,node_no_file_addr, pp_no):
    eepositions = {}
    array = []

    path_nodes_all = []        
    with open(node_no_file_addr, 'r') as file:
        lines = file.readlines()
        for line in lines:
            node_nos = line.strip('\n')
            path_nodes_all.append(node_nos.split(","))
    print("path_nodes_all = ",path_nodes_all)        
    if(pp_no == None):
        path_nodes = list(chain.from_iterable(list(path_nodes_all)))
    else:            
        path_nodes = path_nodes_all[pp_no]
    print("path_nodes = ",path_nodes)    
    for node in path_nodes:
        state = G.node[node]['eePos']
        array.append(state_to_numpy(state))        

    return array

def get_DOF_Values(G, node_no_file_addr, pp_no):
    array = []

    path_nodes_all = []        
    with open(node_no_file_addr, 'r') as file:
        lines = file.readlines()
        for line in lines:
            node_nos = line.strip('\n')
            path_nodes_all.append(node_nos.split(","))
    path_nodes = path_nodes_all[pp_no]
    print("path_nodes = ",path_nodes)
    for node in path_nodes:
        state = G.node[node]['state']
        array.append(state_to_numpy(state))

    return array    
