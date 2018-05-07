import herbpy
import os
import numpy
import math
import openravepy
import argparse
import sys
from prpy import serialization
import json
import networkx as nx
from itertools import islice                

#project src and goal in graph
def find_closest_node(SRC, GOAL):
    SRC_C = state_to_numpy(G.node['110']['state'])
    GOAL_C = SRC_C
    SRC_CN = GOAL_CN = '110'
    for node in G.nodes():
        state = G.node[node]['state']
        config = state_to_numpy(state)
        if(numpy.sum((SRC - config)**2)< numpy.sum((SRC - SRC_C))**2):
            SRC_C = config
            SRC_CN = node
        if(numpy.sum((GOAL - config)**2)< numpy.sum((GOAL - GOAL_C)**2)):
            GOAL_C = config   
            GOAL_CN = node
    return SRC_CN, GOAL_CN        

#return k shortest path
def k_shortest_paths(G, source, target, k, weight=None):
        return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))

#save the processed data
def save_data(start, goal, feature_vec, binary_vec):
    assert len(binary_vec)==K,"No of binary vectors doesn't match"
    print("size of vector = ",binary_vec.shape)
    binary_vec = binary_vec.ravel()
    data = np.concatenate((start, goal, feature_vec, binary_vec))
    np.savetxt("processed_data.txt", data, delimiter = ",")


if __name__ == '__main__':
    global SRC, GOAL
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

    G = nx.read_graphml(args.graphfile)
    G = check_for_collisions(G)

    set_source_and_goal(SRC, GOAL)
    SRC, GOAL = find_closest_node(SRC, GOAL)
    # SRC, GOAL = '2398', '8467'
    print("SRC = ",SRC, "GOAL = ", GOAL)

    paths = k_shortest_paths(G, SRC, GOAL, K)
    print("paths = ",paths)

    binary_vec = convert_to_vectors(paths, G)