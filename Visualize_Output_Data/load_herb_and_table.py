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

def get_box_pose(file_addr):
  b_pose = numpy.ones((4,4), dtype = float)
  i = 0
  with open(file_addr, 'r') as file:
        lines = file.readlines()
        for line in lines:
          values = line.split(",")
          for j in range(4):
            b_pose[i,j] = float(values[j])
          i += 1
  return b_pose

def get_conf(file_addr):
  conf = numpy.loadtxt(file_addr)
  return list(conf)

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

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--condnsfile',type=str,required=True)
    args = parser.parse_args()
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)
    stick_file = os.path.join(objects_path,'objects/stick.kinbody.xml')
    stick = env.ReadKinBodyXMLFile(stick_file)
    env.AddKinBody(stick)

    xpos, ypos, zpos = get_table_pose(args.condnsfile)
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)

    conf = [2, -0.615896, -0.84775500000000004, 0.34049499999999999, -1.22523, 1.38107, 0]
    while(True):
      try:
        conf = get_conf("herb_conf.txt")
      except Exception as e:
        print(e)  
      robot.SetActiveDOFValues(conf)
      ee_trans = robot.right_arm.GetEndEffectorTransform()
      # box_pose = get_box_pose("box_pose.txt")
      step_size = 0.1
      stick_len = 0.4
      prop = -0.5

      push_dir = ee_trans[:3,2]
      parr_dir = ee_trans[:3,1]
      box_pose = ee_trans
      box_pose[:3,3] += push_dir*step_size
      box_pose[:3,3] += parr_dir*stick_len*prop
      # box_pose[0,3], box_pose[1,3], box_pose[2,3] = eepos 

      stick.SetTransform(box_pose)

      print("stick_collis = ", env.CheckCollision(stick))
      # print("box_transform = ", stick.GetTransform())

      # r = float(raw_input("Enter Last Index Value"))