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
import matplotlib.pyplot as plt

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

cube_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

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

def create_occu_grid(occ_grid, env, cube):

  for i in range(-10,20):
    for j in range(-10,20):
      for k in range(0,14,2):
        cube_pose[0,3], cube_pose[1,3], cube_pose[2,3] = i/10.0, j/10.0, k/10.0
        cube.SetTransform(cube_pose)
        if(env.CheckCollision(cube)==True):
          occ_grid[i+10,j+10,k/2] = 0
          print("In Collision")
        else:
          occ_grid[i+10,j+10,k/2] = 1
          print("Free Space")
  return occ_grid

def list_all_dir(data_dir):
  task_dirs = os.listdir(data_dir)

  list_dir = []
  for task_dir in task_dirs:
      env_dirs = os.listdir(data_dir+"/"+task_dir)
      for env_dir in env_dirs:
          list_dir.append(data_dir +"/"+ task_dir +"/"+ env_dir)
  return list_dir

def main():
  parser = argparse.ArgumentParser(description='Generate environments')
  parser.add_argument('--datadir',type=str,required=True)
  args = parser.parse_args()

  env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
  robot.right_arm.SetActive()
  # Load table from pr_ordata
  table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
  tall_white_box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
  table = env.ReadKinBodyXMLFile(table_file)
  env.AddKinBody(table)
  tall_white_box = env.ReadKinBodyXMLFile(tall_white_box_file)
  env.AddKinBody(tall_white_box)
  cube_file = os.path.join(objects_path,'objects/cube.kinbody.xml')
  cube = env.ReadKinBodyXMLFile(cube_file)
  env.AddKinBody(cube)

  direc = list_all_dir(args.datadir)

  for envdir in direc:
    cond = envdir + "/conditions.txt"
    xpos, ypos, xpos1, ypos1, zpos1 = get_table_pose(cond)
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)

    box_pose[0,3] = xpos1
    box_pose[1,3] = ypos1
    box_pose[2,3] = zpos1
    tall_white_box.SetTransform(box_pose)

    occ_grid = numpy.ones((30,30,7))

    occ_grid = create_occu_grid(occ_grid, env, cube)

    occ_grid = numpy.ones((30,30,14))

    occ_grid = create_occu_grid(occ_grid, env, cube)

    front = numpy.zeros((14,30))
    for j in range(30):
      for k in range(14):
        for i in range(30):
          if(occ_grid[i][j][k]==0):
            front[13-k][j] = 1
            break
    top = numpy.zeros((30, 30))
    for i in range(30):
      for j in range(30):
        for k in range(14):
          if(occ_grid[i][j][k]==0):
            top[i][j] = 1
    right = numpy.zeros((14,30))
    for i in range(30):
      for k in range(14):
        for j in range(30):
          if(occ_grid[i][j][k]==0):
            right[13-k][i] = 1

    projections = numpy.concatenate((front.ravel(), top.ravel(), right.ravel()))
    for i in range(len(projections)):
      if(projections[i]==0):
        projections[i]=-1
    numpy.savetxt(envdir+"/projections.txt", projections, delimiter=" ", fmt="%s")

if __name__ == '__main__':
  main()