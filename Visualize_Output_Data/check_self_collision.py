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

def main():
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()

    conf = [2,-3,0,0,0,0,0]

    flat_base_file = os.path.join(objects_path,'objects/flat_base.kinbody.xml')
    flat_base = env.ReadKinBodyXMLFile(flat_base_file)
    env.AddKinBody(flat_base)

    table_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

    base_pose = table_pose.copy()
    base_pose[:3,3] = 0,0,-0.04
    flat_base.SetTransform(base_pose)

    while(True):
        robot.SetActiveDOFValues(conf)

        print("C = ", env.CheckCollision(robot))
        print("C1 = ",robot.CheckSelfCollision())
        x = float(raw_input("Enter 1st Index Value"))
        conf[0] = x

if __name__ == '__main__':
    main()