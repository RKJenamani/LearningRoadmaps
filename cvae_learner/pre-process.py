import os
import argparse
import networkx as nx
import math        
import numpy as np
FILE_NAME = "samplingdata.txt"

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def write_one_row(curr_node):
    print("curr_node = ",curr_node)
    with open(FILE_NAME, 'a') as file:
        file.writelines(','.join(str(j) for j in curr_node) + '\n')

def process_it(G, directory):
    start = np.loadtxt(directory+"/start_node.txt")
    goal = np.loadtxt(directory+"/goal_node.txt")
    # occ_grid = np.loadtxt(directory+"/occ_grid.txt")
    conditions = np.loadtxt(directory+"/conditions.txt")   
    # conditions = np.array([conditions[23]]) 
    # cond = cond.split(",")
    path_nodes = []
    i = 0
    with open(directory + "/path_nodes.txt", 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            print(line)
            print("\n\n")
            if(not line == '-1'):
                s = state_to_numpy(G.node[str(int(start[i]))]['state'])
                g = state_to_numpy(G.node[str(int(goal[i]))]['state'])
                path_nodes = str(line).split(",")
                # print(path_nodes)
                for path_node in path_nodes:
                    node_conf = state_to_numpy(G.node[path_node]['state'])
                    curr_node = np.array([])
                    # print("Data = ",node_conf, s, g, cond)
                    print("\n")
                    curr_node = np.concatenate((node_conf, s, g, conditions))
                    print("shape of curr_node = ", curr_node.shape)
                    write_one_row(curr_node)

def list_all_dir(data_dir):
    task_dirs = os.listdir(data_dir)

    list_dir = []
    for task_dir in task_dirs:
        env_dirs = os.listdir(data_dir+"/"+task_dir)
        for env_dir in env_dirs:
            list_dir.append(data_dir +"/"+ task_dir +"/"+ env_dir)
    return list_dir        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--datadir',type=str,required=True)
    args = parser.parse_args()

    G = nx.read_graphml(args.graphfile)
    data_dir = args.datadir

    directories = list_all_dir(data_dir)
    print(directories)

    for directory in directories:
        process_it(G, directory)