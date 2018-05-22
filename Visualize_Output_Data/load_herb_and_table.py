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
    return t[3], t[7]    

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--condnsfile',type=str,required=True)
    args = parser.parse_args()
    table_pose[0][3], table_pose[1][3] = get_table_pose(args.condnsfile)

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

    xpos, ypos = 1.18872327738, -0.724415406951
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)
    
    x = raw_input("Press Enter")