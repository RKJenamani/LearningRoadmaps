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

dish_washer_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
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
    return t[3], t[7], t[19], t[23], t[27]

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='Generate environments')
    # parser.add_argument('--condnsfile',type=str,required=True)
    args = parser.parse_args()
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    # table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    # tall_white_box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    # table = env.ReadKinBodyXMLFile(table_file)
    # env.AddKinBody(table)
    # tall_white_box = env.ReadKinBodyXMLFile(tall_white_box_file)
    # env.AddKinBody(tall_white_box)

    dish_washer_file = os.path.join(objects_path,'furniture/bookcase.kinbody.xml')
    dish_washer = env.ReadKinBodyXMLFile(dish_washer_file)
    env.AddKinBody(dish_washer)

    dish_washer_pose[0,3], dish_washer_pose[1,3], dish_washer_pose[2, 3] = 0.75, 0, 0.75
    dish_washer.SetTransform(dish_washer_pose) 

    # xpos, ypos, xpos1, ypos1, zpos1 = get_table_pose(args.condnsfile)
    # table_pose[0,3] = xpos
    # table_pose[1,3] = ypos

    # table.SetTransform(table_pose)

    # box_pose[0,3] = xpos1
    # box_pose[1,3] = ypos1
    # box_pose[2,3] = zpos1
    # tall_white_box.SetTransform(box_pose)
    x = raw_input("Press Enter")