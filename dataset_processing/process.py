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

table_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])


XMIN = 0.638
XMAX = 1.2
YMIN = -1.17
YMAX = 0.74
EDGE_DISCRETIZATION = 7
K = 10
SRC = numpy.array((0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7))
GOAL = numpy.array((5, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7))
NUM_FILES = 10000

from catkin.find_in_workspaces import find_in_workspaces

package_name = 'pr_ordata'
directory = 'data'
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project=package_name,
    path=directory,
    first_match_only=True)

if len(objects_path) == 0:
    print('Can\'t find directory %s/%s' % (package_name, directory))
    sys.exit()
else:
    print objects_path # for me this is '/home/USERNAME/catkin_workspaces/herb_ws/src/pr-ordata/data/objects'
    objects_path = objects_path[0]

def set_source_and_goal(s, g):
    global SRC, GOAL
    SRC = s
    GOAL = g
    assert (len(s)==len(g))and(len(s)==7),"Length of DOF Values doesn't match!"    

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return numpy.array(val_list)

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

#remove all the invalid edges from the graph
def check_for_collisions(G):
    to_remove = []
    for i,edge in enumerate(G.edges()):
            u,v = edge
            state1 = G.node[u]['state']
            state2 = G.node[v]['state']
            configs_to_check = edge_to_configs(state1,state2)

            edge_free = 1

            for cc in configs_to_check:
                robot.SetActiveDOFValues(cc)

                if env.CheckCollision(robot) == True:
                    edge_free = 0
                    break
            if(not edge_free):
                to_remove.append((u, v))
    for e in to_remove:
        G.remove_edge(e[0], e[1])            
    return G                    

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