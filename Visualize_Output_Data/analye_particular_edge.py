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
import helper

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

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return numpy.array(val_list) 

def get_table_pose(condnsfile):
    t = numpy.loadtxt(condnsfile)
    print("t = ", t)
    return t[3], t[7]

def get_ee_pos(env, robot, state):
    robot.SetActiveDOFValues(state)

    ee_trans = robot.right_arm.GetEndEffectorTransform()
    trans = ee_trans[0:3,3]
    eepos = trans.tolist()

    return eepos    

def analyse_edge(config1, config2, env, robot):
    diff = config2 - config1
    step = diff/EDGE_DISCRETIZATION

    to_check = list()
    to_check.append(config1)

    for i in xrange(EDGE_DISCRETIZATION - 1):
        conf = config1 + step*(i+1)
        to_check.append(conf)

    isFree = 1
    for conf in to_check:
      robot.SetActiveDOFValues(conf)
      print("collision_check = ",env.CheckCollision(robot)) 
      y = raw_input("Press K")
      if(env.CheckCollision(robot)):
        isFree = 0

    if(not isFree):      
      print("Invalid Edge!!!!!!............................")         

    return to_check        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--condnsfile',type=str,required=True)
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--edge_no',type=str,required=True)
    args = parser.parse_args()
    G = nx.read_graphml(args.graphfile) 

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

    xpos, ypos = get_table_pose(args.condnsfile)
    pp_no = 3
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)

    edge_no = args.edge_no

    l = list(G.edges())

    while(True):
      edge_no = raw_input("Enter Edge no")
      u, v = l[int(edge_no)]
      print("edge from ", u, " to ", v)
      config1 = state_to_numpy(G.node[u]['state'])
      config2 = state_to_numpy(G.node[v]['state'])
      analyse_edge(config1, config2, env, robot)

    analyse_edge(state_to_numpy(G.node[u]['state']), state_to_numpy(G.node[v]['state']), env, robot)
    raw_input(".............Press Enter")
