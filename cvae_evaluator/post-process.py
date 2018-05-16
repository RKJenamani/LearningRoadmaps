import networkx as nx
import math        
import numpy as np
from math import sqrt
import herbpy
import os
from prpy import serialization
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

EDGE_DISCRETIZATION = 7

from catkin.find_in_workspaces import find_in_workspaces

package_name = 'pr_ordata'
directory = 'data'
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project=package_name,
    path=directory,
    first_match_only=True)

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)           

def get_actual_samples(G, directory):
    nodes = {}
    i = 0
    with open(directory + "/path_nodes.txt", 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            print(line)
            print("\n\n")
            if(not line == '-1'):
                path_nodes = str(line).split(",")
                # print(path_nodes)
                for path_node in path_nodes:
                    node_conf = str(G.node[path_node]['state'])
                    nodes[str(i)] = node_conf
                    i += 1
    return nodes            

def generate_graph(nodes): #nodes -> key, value pair, value => string with joint angles separated by ' '
    G = nx.Graph()
    for key in nodes:
        value = str(nodes[str(key)]) 
        G.add_node(str(key), state = value) 
    return G

def get_shortest_path_length(G, start, goal):
    start_node, goal_node = find_closet_node(start, goal)
    path_length = nx.dijkstra_path_length(g, str(start_node), str(goal_node), 'weight')
    return path_length

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))


def edge_to_configs(state1, state2):
    config1 = state_to_numpy(state1)
    config2 = state_to_numpy(state2)

    diff = config2 - config1
    step = diff/EDGE_DISCRETIZATION

    to_check = list()

    for i in xrange(EDGE_DISCRETIZATION - 1):
        conf = config1 + step*(i+1)
        to_check.append(conf)

    return to_check
    
def create_weighted_graph(G):
    for i, edge in enumerate(G.edges()):
        u,v = edge
        state1 = state_to_numpy(G.node[u]['state'])
        state2 = state_to_numpy(G.node[v]['state'])
        G[u][v]['weight'] = calc_weight(state1, state2)
    nx.write_graphml(G, "graphs/weighted_graph.graphml")
    return G

def check_for_collision(G, env, robot, u, v):
    state1 = G.node[u]['state']
    state2 = G.node[v]['state']
    configs_to_check = edge_to_configs(state1,state2)

    edge_free = 1

    for cc in configs_to_check:
        robot.SetActiveDOFValues(cc)

        if env.CheckCollision(robot) == True:
            edge_free = 0
            break
    return edge_free        

def load_table(table_pos, env): # 4X4 numpy array
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)
    table.SetTransform(table_pose)

def connect_knn(G, K):
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)
        G1 = G.copy()

        for k in range(K):
            w = 1000000
            sn = None
            for node1 in G1.nodes():
                state1 = G1.node[node1]['state']
                conf1  = state_to_numpy(state1)
                if(calc_weight(conf, conf1) < w):
                    w = calc_weight(conf, conf1)
                    sn = node1

            if(check_for_collision(node, sn)==1):
                G.add_edge(node, sn)
            G1.remove_node(node1)
    return G        

def load_output_samples(file_addr):
    nodes = {}
    i = 0
    with open(file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes[str(i)] = line
            i += 1
    print(nodes)
    return nodes   

def get_eepositions(env, robot, nodes):
    eepositions = {}
    for key in nodes:
        value = nodes[key]
        conf = state_to_numpy(value)
        robot.SetActiveDOFValues(conf)

        ee_trans = robot.right_arm.GetEndEffectorTransform()
        trans = ee_trans[0:3,3]
        eepos = trans.tolist()

        eepositions[key] = eepos
    return eepositions    

#take ee_transform and plot it
def plot_end_effector_positions(eepositions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = []
    y = []
    z = []
    for key in eepositions:
        value = eepositions[key]
        x.append(value[0])
        y.append(value[1])
        z.append(value[2])
    ax.scatter(x, y, z) 
    plt.show()   

def main():
    K = 5
    file_addr = "output_sample_node_e9.txt"
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    nodes = load_output_samples(file_addr)
    G = generate_graph(nodes)
    G1 = nx.read_graphml("graphs/herb_halton_1.graphml")
    eepositions = get_eepositions(env, robot, nodes)
    # plot_end_effector_positions(eepositions)

    nodes_1 = get_actual_samples(G1, "data/T1/9") 
    eepositions_1 = get_eepositions(env, robot, nodes_1)
    plot_end_effector_positions(eepositions_1)
    # G = create_weighted_graph(G)
    # G = connect_knn(G, K)
    # path_length = get_shortest_path_length(G, start_conf, goal_conf)


if __name__ == '__main__':
            main()      
