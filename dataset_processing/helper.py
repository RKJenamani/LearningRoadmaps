import numpy as np
import networkx as nx
import random
import math

step_size = 0.1
stick_len = 0.4 

rx1 = [0, 6]
rx2 = [None, 9]
rx3 = [None, None]

ry1 = [0, 7]
ry2 = [None, 9]
EDGE_DISCRETIZATION_T = 40
EDGE_DISCRETIZATION = 11 
THRESHOLD = 4.0

def write_to_file(directory, all_paths):
    with open(directory + "/path_nodes.txt", 'a') as file:
        file.writelines(','.join(str(j) for j in i) + '\n' for i in all_paths)
        
def calc_weight(config1, config2):
    return math.sqrt(float(np.sum((config2-config1)**2)))

def state_to_numpy(state):
    # print("state = ", state)
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

#check for existense of a feasible path
def path_exists(G, src, goal):
    try:
        paths = nx.dijkstra_path(G, src, goal, weight='weight')
        return 1
    except Exception as e:
        return 0

#to check if a node is free
def is_free(node_conf, env, robot, stick):
    robot.SetActiveDOFValues(node_conf[:-1])
    ee_trans = robot.right_arm.GetEndEffectorTransform()

    push_dir = ee_trans[:3,2]
    parr_dir = ee_trans[:3,1]
    stick_pose = ee_trans
    stick_pose[:3,3] += push_dir*step_size
    stick_pose[:3,3] += parr_dir*stick_len*node_conf[-1]
    stick.SetTransform(stick_pose)
    if env.CheckCollision(robot) or env.CheckCollision(stick) or robot.CheckSelfCollision():
        return 0

    return 1

#check for the planning  problem being trivial
def valid_start_goal(start, goal, env, robot, stick):
    start = np.array(start)
    goal = np.array(goal)

    if(not (is_free(start, env, robot, stick) and is_free(goal, env, robot, stick))):
        # print("start_free = ",is_free(start, obstacles))
        # print("goal_free = ",is_free(goal, obstacles))
        return 0

    diff = goal - start
    step = diff/EDGE_DISCRETIZATION_T

    for i in range(EDGE_DISCRETIZATION_T+1):
        nodepos = start + step*i
        if(not (is_free(nodepos, env, robot, stick))):
            return 1
    # print("trivial")        
    return 0

#to check if two nodes are within threshold and can be connected
def satisfy_condition(node1_pos, node2_pos, env, robot, stick):
    node1_pos, node2_pos = np.array(node1_pos), np.array(node2_pos)
    if(calc_weight(node1_pos, node2_pos)>THRESHOLD):
        return 0
    
    diff = node2_pos - node1_pos
    step = diff/EDGE_DISCRETIZATION

    for i in range(EDGE_DISCRETIZATION+1):
        nodepos = node1_pos + step*i
        if(not (is_free(nodepos, env, robot, stick))):
            return 0

    return 1

#connect knn 
def connect_knn_for_one_node(G, K, node):
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
        if(w<THRESHOLD):
            G.add_edge(node, sn)
            G[node][sn]['weight'] = w
            G1.remove_node(sn)
        else:
            break    
    return G

#closest node to a point
def find_closest_node(shallow_G1, node_posn):
    dist = 10000
    c_node = None

    for node in list(shallow_G1.nodes()):
        pos = state_to_numpy(shallow_G1.node[node]['state'])
        if(calc_weight(pos, node_posn)<dist):
            dist = calc_weight(pos, node_posn)
            c_node = node
    
    return c_node

def remove_invalid_edges(G, env, robot, stick):
    to_remove = []
    for i,edge in enumerate(G.edges()):
        u,v = edge
        node1_pos = state_to_numpy(G.node[u]['state'])
        node2_pos = state_to_numpy(G.node[v]['state'])

        diff = node2_pos - node1_pos
        step = diff/EDGE_DISCRETIZATION

        for i in range(EDGE_DISCRETIZATION+1):
            nodepos = node1_pos + step*i
            if(not (is_free(nodepos, env, robot, stick))):
                to_remove.append((u,v))
                break
    
    for edge in to_remove:
        u, v = edge
        G.remove_edge(u, v) 
    return G