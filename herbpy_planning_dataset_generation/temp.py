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

bookcase_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
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

def create_occu_grid(occ_grid, env, cube):

    for i in range(-10,20):
      for j in range(-10,20):
        for k in range(0,14):
          cube_pose[0,3], cube_pose[1,3], cube_pose[2,3] = i/10.0, j/10.0, k/10.0
          cube.SetTransform(cube_pose)
          if(env.CheckCollision(cube)==True):
            occ_grid[i+10,j+10,k] = 0
            print("In Collision")
          else:
            print("Free Space") 
          # raw_input("Press Enter"+str(i)+","+str(j)+","+str(k))   
    return occ_grid         

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='Generate environments')

    # direc = ["data/T1/", "data/T2/"]

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

    bookcase_file = os.path.join(objects_path,'furniture/bookcase.kinbody.xml')
    bookcase = env.ReadKinBodyXMLFile(bookcase_file)
    # env.AddKinBody(bookcase)

    env.RemoveKinBody(robot)
    cube_file = os.path.join(objects_path,'objects/cube.kinbody.xml')
    cube = env.ReadKinBodyXMLFile(cube_file)
    env.AddKinBody(cube)   

    table_pose[0,3] = 1
    table_pose[1,3] = 0

    table.SetTransform(table_pose)

    box_pose[0,3] = 0.75
    box_pose[1,3] = 0
    box_pose[2,3] = 1.1
    tall_white_box.SetTransform(box_pose)

    bookcase_pose[0,3], bookcase_pose[1,3], bookcase_pose[2,3] = 1, 0, 0.75
    bookcase.SetTransform(bookcase_pose)

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

    plt.imshow(front,cmap='Greys',  interpolation='nearest')
    plt.savefig("front_table_box.jpg")
    print("front = ",front)
    plt.show()       

    plt.imshow(top,cmap='Greys',  interpolation='nearest')
    plt.savefig("top_table_box.jpg")
    print("top = ",top)
    plt.show()        

    plt.imshow(right,cmap='Greys',  interpolation='nearest')   
    plt.savefig("right_table_box.jpg")     
    print("right = ",right)
    plt.show()        

    # numpy.savetxt(d + str(i) + "/occ_grid.txt", occ_grid.ravel(), delimiter=" ", fmt="%s")

    x = raw_input("Press Enter")