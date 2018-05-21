import numpy as np

def get_eepositions(eePos_file_addr,node_no_file_addr, pp_no):
    eepositions = {}
    array = []
    i = 0
    with open(eePos_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            eepositions[str(i)] = np.float32(np.array(line.split(",")))
            i += 1

    path_nodes_all = []        
    with open(node_no_file_addr, 'r') as file:
        lines = file.readlines()
        for line in lines:
            node_nos = line.strip('\n')
            path_nodes_all.append(node_nos.split(","))
    path_nodes = path_nodes_all[pp_no]
    for node in path_nodes:
        array.append(eepositions[node])        

    return array

def get_DOF_Values(DOF_Values_file_addr,node_no_file_addr, pp_no):
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
    for node in path_nodes:
        array.append(DOF_Values[node])

    return array    
