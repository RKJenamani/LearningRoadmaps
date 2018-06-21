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

tall_box_pose_1 = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
      7.83268307e-01],
   [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
     -2.58088849e-03],
   [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
      1.19378528e-07],
   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
      1.00000000e+00]])

tall_box_pose_2 = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
      7.83268307e-01],
   [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
     -2.58088849e-03],
   [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
      1.19378528e-07],
   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
      1.00000000e+00]])

small_box_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
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
    return t[0], t[1], t[2], t[9], t[10], t[11]

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
    tall_box_1_file = os.path.join(objects_path,'objects/tall_white_box1.kinbody.xml')
    tall_box_1 = env.ReadKinBodyXMLFile(tall_box_1_file)
    env.AddKinBody(tall_box_1)

    tall_box_2_file = os.path.join(objects_path,'objects/tall_white_box2.kinbody.xml')
    tall_box_2 = env.ReadKinBodyXMLFile(tall_box_2_file)
    env.AddKinBody(tall_box_2)

    small_box_file = os.path.join(objects_path,'objects/small_white_box.kinbody.xml')
    print(small_box_file)
    small_box = env.ReadKinBodyXMLFile(small_box_file)
    print("small_box = ", small_box)
    env.AddKinBody(small_box)

    xpos, ypos, zpos, xpos1, ypos1, zpos1 = get_table_pose(args.condnsfile)
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)

    small_box_pose[0,3], tall_box_pose_1[0,3], tall_box_pose_2[0,3] = xpos1, xpos1, xpos1
    small_box_pose[1,3], tall_box_pose_1[1,3], tall_box_pose_2[1,3] = ypos1, ypos1-0.3, ypos1+0.3
    small_box_pose[2,3], tall_box_pose_1[2,3], tall_box_pose_2[2,3] = zpos1, zpos1, zpos1
 
    small_box.SetTransform(small_box_pose)
    tall_box_1.SetTransform(tall_box_pose_1)
    tall_box_2.SetTransform(tall_box_pose_2)
    r = 0

    conf = numpy.array([0.62132900000000002, -0.615896, -0.84775500000000004, 0.34049499999999999, -1.22523, 1.38107,r])
    robot.SetActiveDOFValues(conf)

    x = raw_input("Press Enter")