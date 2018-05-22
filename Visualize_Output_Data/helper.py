import numpy as np
import networkx as nx

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list) 

def get_eepositions(G, eePos_file_addr,node_no_file_addr, pp_no):
    eepositions = {}
    array = []
    i = 0
    with open(eePos_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            eepositions[str(i)] = np.float32(np.array(line.split(" ")))
            i += 1

    path_nodes_all = []        
    with open(node_no_file_addr, 'r') as file:
        lines = file.readlines()
        for line in lines:
            node_nos = line.strip('\n')
            path_nodes_all.append(node_nos.split(","))

    if(pp_no == None):
        path_nodes = list(np.ravel(np.array(path_nodes_all)))
    else:            
        path_nodes = path_nodes_all[pp_no]
    for node in path_nodes:
        state = G.node[node]['eePos']
        array.append(state_to_numpy(state))        

    return array

def get_DOF_Values(G, DOF_Values_file_addr,node_no_file_addr, pp_no):
    DOF_Values = {}
    array = []
    i = 0
    with open(DOF_Values_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            DOF_Values[str(i)] = np.float32(np.array(line.split(" ")))
            i += 1

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
