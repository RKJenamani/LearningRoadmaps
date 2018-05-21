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

def wait_for_user():
  x = raw_input("Press Enter")



if __name__=='__main__':

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

    xpos, ypos = 1, 0
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)

    path_nodes_no_addr = "data_vis/path_node_no.txt"
    DOF_file_addr = "data_vis/DOF_Values_enum_nodes.txt"

    path_conf_values = helper.get_DOF_Values(DOF_file_addr, path_nodes_no_addr, 1) 

    print("len(path_conf_values) = ", len(path_conf_values))
    for conf in path_conf_values:
      robot.SetActiveDOFValues(conf)
      print("env(Collision) = ",env.CheckCollision(robot))
      wait_for_user()
    
    wait_for_user()