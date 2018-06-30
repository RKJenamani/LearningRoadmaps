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

box_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
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

def get_table_pose(condnsfile):
    t = numpy.loadtxt(condnsfile)
    print("t = ", t)
    return t[3], t[7], t[11]

def get_box_pose(condnsfile):
    t = numpy.loadtxt(condnsfile)
    print("t = ", t)
    return t[19], t[23], t[27]

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
      robot.SetActiveDOFValues(conf[:-1])
      ee_trans = robot.right_arm.GetEndEffectorTransform()
      # box_pose = get_box_pose("box_pose.txt")
      step_size = 0.1
      stick_len = 0.4
      prop = conf[-1]

      push_dir = ee_trans[:3,2]
      parr_dir = ee_trans[:3,1]
      box_pose = ee_trans
      box_pose[:3,3] += push_dir*step_size
      box_pose[:3,3] += parr_dir*stick_len*prop
      # box_pose[0,3], box_pose[1,3], box_pose[2,3] = eepos 

      stick.SetTransform(box_pose)
      # print(" C = ", robot.CheckCollision(robot))
      print("collision_check = ",env.CheckCollision(robot) or env.CheckCollision(stick)) 
      y = raw_input("Press K")
      if(env.CheckCollision(robot)):
        isFree = 0

    if(not isFree):      
      print("Invalid Edge!!!!!!............................")         

    return to_check        

def wait_for_user():
  x = raw_input("Press Enter")

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--envdir',type=str,required=True)
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()
    G = nx.read_graphml(args.graphfile)

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

    box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    box = env.ReadKinBodyXMLFile(box_file)
    env.AddKinBody(box)

    stick_file = os.path.join(objects_path,'objects/stick.kinbody.xml')
    stick = env.ReadKinBodyXMLFile(stick_file)
    env.AddKinBody(stick)

    xpos, ypos, zpos = get_table_pose(args.envdir+"/conditions.txt")
    pp_no = 5
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)

    flat_base_file = os.path.join(objects_path,'objects/flat_base.kinbody.xml')
    flat_base = env.ReadKinBodyXMLFile(flat_base_file)
    env.AddKinBody(flat_base)
    base_pose = table_pose.copy()
    base_pose[:3,3] = 0,0,-0.04
    flat_base.SetTransform(base_pose)

    xpos1, ypos1, zpos1 = get_box_pose(args.envdir+"/conditions.txt")
    box_pose[:3,3] = xpos1, ypos1, zpos1
    box.SetTransform(box_pose)

    path_nodes_no_addr = args.envdir + "/path_nodes.txt"

    path_conf_values = helper.get_DOF_Values(G, path_nodes_no_addr, pp_no) 
    # path_eePos_values = helper.get_DOF_Values()

    print("len(path_conf_values) = ", len(path_conf_values))
    i = 0  
    for conf in path_conf_values:
      robot.SetActiveDOFValues(conf[:-1])
      ee_trans = robot.right_arm.GetEndEffectorTransform()
      # box_pose = get_box_pose("box_pose.txt")
      step_size = 0.1
      stick_len = 0.4
      prop = conf[-1]

      push_dir = ee_trans[:3,2]
      parr_dir = ee_trans[:3,1]
      box_pose = ee_trans
      box_pose[:3,3] += push_dir*step_size
      box_pose[:3,3] += parr_dir*stick_len*prop
      # box_pose[0,3], box_pose[1,3], box_pose[2,3] = eepos 

      stick.SetTransform(box_pose)

      print("env(Collision) = ",env.CheckCollision(robot) and env.CheckCollision(stick))
      if(i < len(path_conf_values)-1):
        print("i = ",i)
        print("edge from ", conf, " to ", path_conf_values[i+1])
        analyse_edge(conf, path_conf_values[i+1], env, robot)
      wait_for_user()
      i += 1
    wait_for_user()