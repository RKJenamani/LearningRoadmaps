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
from itertools import islice
import random
import copy
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

def write_one_dir(task_id, datadir, env_no,start_node, goal_node, condition_vectors, binary_vec):
    dir_path = datadir+"/"+task_id+"/"+str(env_no)
    try:
        os.mkdir(datadir+"/"+task_id, 0755)
    except:
        print(task_id,"Directory Exists")
    
    try:
        os.mkdir(dir_path, 0755)
    except:
        print(dir_path,"Directory Exists")        
    binary_vec = numpy.array(binary_vec)    
    print("binary_vec = ",binary_vec)
    start_node = numpy.array(start_node)
    goal_node = numpy.array(goal_node)
    numpy.savetxt(dir_path+"/start_node.txt", start_node, delimiter=" ", fmt="%s")
    numpy.savetxt(dir_path+"/goal_node.txt", goal_node, delimiter=" ", fmt="%s")
    numpy.savetxt(dir_path+"/binary_vec.txt", binary_vec, delimiter=" ", fmt="%s")
    numpy.savetxt(dir_path+"/conditions.txt", condition_vectors, delimiter=" ", fmt="%s") 

def path_exists(G1, source, target):
    try:
      nx.dijkstra_path(G1, source, target, weight="weight")
      return True
    except Exception as e:
      return False  

def table_box_below_above(task_id, datadir, robot, env, G):
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

    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

    box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    box = env.ReadKinBodyXMLFile(box_file)
    env.AddKinBody(box)    

    stick_file = os.path.join(objects_path,'objects/stick.kinbody.xml')
    stick = env.ReadKinBodyXMLFile(stick_file)
    env.AddKinBody(stick)

    table_pose[0,3], table_pose[1,3], table_pose[2,3] = 0.8, 0, 0
    box_pose[0,3], box_pose[2,3] = 0.8, 1.0

    env_no = 0
    shallow_G = nx.read_graphml("graphs/halton8D3000_0.graphml")

    for i in range(0,21):
        if(i%2==0):
          continue
        print(" debug: env_no = ",env_no)
        table_pose[1,3] = -0.5 + random.random()
        box_pose[1,3] = -0.3 + random.random()/3

        table.SetTransform(table_pose)
        box.SetTransform(box_pose)
        # stick.SetTransform(stick_pose)

        cond = numpy.concatenate((numpy.array(table_pose.ravel()), numpy.array(box_pose.ravel())))
        
        G1 = G.copy()
        shallow_G1 = shallow_G.copy()

        print(" debug: generating binary_vec")
        binary_vec = [1]
        binary_vec, G1 = helper.get_binary_vector(G, G1, robot, stick, env)
        b_v, shallow_G1 = helper.get_binary_vector(shallow_G, shallow_G1, robot, stick, env)
        print(" debug: got binary_vec")

        start = []
        goal = []

        s_temp, g_temp = None, None
        while(len(start)<50 or len(goal)<50):
            print(" debug: in while loop, start = ", len(start), " goal = ", len(goal))
            random_node, status = helper.get_valid_node(task_id, G, robot, stick, env, table_pose, box_pose)
            
            if(status=="s" and s_temp==None):
                s_temp = random_node
            elif(status=="g" and g_temp==None):
                g_temp = random_node


            if(not (s_temp==None or g_temp==None)):
              print(" debug: checking if shallow_fails")  
              if(not helper.shallow_fails(shallow_G1, G.node[s_temp]['state'], G.node[g_temp]['state'], env, robot, stick)):
                print(" debug: shallow_passed")
                continue

              print(" debug: shallow_fails!! :D ")  
              if(path_exists(G1, s_temp, g_temp)):
                start.append(s_temp)
                goal.append(g_temp)
              s_temp = None
              g_temp = None        

        if(len(start) == 50 and len(goal) == 50):    
            print("write_one_dir")    
            write_one_dir(task_id, datadir, env_no, start, goal, cond, binary_vec)
        else:
            print("no of start or goal posiitons is under limit")

        env_no += 1  

def list_all_dir(data_dir):
    task_dirs = os.listdir(data_dir)

    list_dir = []
    for task_dir in task_dirs:
        env_dirs = os.listdir(data_dir+"/"+task_dir)
        for env_dir in env_dirs:
            list_dir.append(data_dir +"/"+ task_dir +"/"+ env_dir)
    return list_dir

def get_parameters(directories):
  binary_vecs = []
  table_posns = []
  box_posns = []

  for directory in directories:
    binary_vec = numpy.loadtxt(directory+"/binary_vec.txt")
    binary_vecs.append(binary_vec)
    cond = numpy.loadtxt(directory+"/conditions.txt")
    table_pose = cond[:16].reshape(4,4)
    box_pose = cond[16:].reshape(4,4)
    table_posns.append(table_pose)
    box_posns.append(box_pose)

  return binary_vecs, table_posns, box_posns

def table_box_left_right(task_id, datadir, robot, env, G):
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

    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)
    box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    box = env.ReadKinBodyXMLFile(box_file)
    env.AddKinBody(box)

    stick_file = os.path.join(objects_path,'objects/stick.kinbody.xml')
    stick = env.ReadKinBodyXMLFile(stick_file)
    env.AddKinBody(stick)

    table_pose[0,3], table_pose[1,3], table_pose[2,3] = 0.8, 0, 0
    box_pose[0,3], box_pose[2,3] = 0.8, 1.0

    b_xmin, b_xmax = -0.6, 0.2

    table_yp = [0.1, -0.1]

    env_no = 0
    shallow_G = nx.read_graphml("graphs/halton8D3000_0.graphml")

    # directories = list_all_dir(data_dir)

    # binary_vecs, table_posns, box_posns = get_parameters(directories)
    
    for i in range(len(binary_vecs)):
        table_pose[1,3] = -0.5 + random.random()
        box_pose[1,3] = -0.3 + random.random()/3

        table.SetTransform(table_pose)
        box.SetTransform(box_pose)
        # stick.SetTransform(stick_pose)

        print("wokring on ", env_no)  

        cond = numpy.concatenate((numpy.array(table_pose.ravel()), numpy.array(box_pose.ravel())))
        G1 = G.copy()
        shallow_G1 = shallow_G.copy()

        # G1 = helper.remove_invalid_edges(G1, binary_vec)
        print("Generating binary_vec")
        # binary_vec = []
        binary_vec, G1 = helper.get_binary_vector(G, G1, robot, stick, env)
        b_v, shallow_G1 = helper.get_binary_vector(shallow_G, shallow_G1, robot, stick, env)
        print("Generated binary_vec")

        s_temp, g_temp = None, None
        start = []
        goal = []

        while(len(start)<50 or len(goal)<50):
            print("in while loop, start = ", len(start), " goal = ", len(goal))
            random_node, status = helper.get_valid_node(task_id, G, robot, stick, env, table_pose, box_pose)
            
            if(status=="s" and s_temp==None):
                s_temp = random_node
            elif(status=="g" and g_temp==None):
                g_temp = random_node  
            
            if(not (s_temp==None or g_temp==None)):
              print(" debug: checking if shallow_fails")  
              if(not helper.shallow_fails(shallow_G1, G.node[s_temp]['state'], G.node[g_temp]['state'], env, robot, stick)):
                print(" debug: shallow_passed")
                continue
              if(path_exists(G1, s_temp, g_temp)):
                if(len(start)<25):
                  start.append(s_temp)
                  goal.append(g_temp)
                else:
                  goal.append(s_temp)
                  start.append(g_temp)  
              s_temp = None
              g_temp = None


        if(len(start) == 50 and len(goal) == 50):    
            print("write_one_dir")    
            write_one_dir(task_id, datadir, env_no, start, goal, cond, binary_vec)
        else:
            print("no of start or goal posiitons is under limit")

        env_no += 1

def bookcase_front(task_id, datadir, robot, env, G):
    bookcase_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

    bookcase_file = os.path.join(objects_path,'furniture/bookcase.kinbody.xml')
    bookcase = env.ReadKinBodyXMLFile(bookcase_file)
    env.AddKinBody(bookcase)

    bookcase_pose[0,3], bookcase_pose[1,3], bookcase_pose[2,3] = 0.8, 0, 0.75

    bookcase_yp = [-0.4, -0.35, 0.3]

    bookcase_xp = [-0.65, -0.63, -0.67]
    # bookcase_xp = [0.55, 0.6, 0.65]

    env_no = 0
    for b_y in bookcase_yp:
      for b_x in bookcase_xp:

        bookcase_pose[0,3], bookcase_pose[1,3] = b_x, b_y
        bookcase.SetTransform(bookcase_pose)

        Tz = openravepy.matrixFromAxisAngle([0,0,numpy.pi/2])
        bookcase.SetTransform(numpy.dot(Tz,bookcase.GetTransform()))

        cond = numpy.array(bookcase_pose.ravel())

        binary_vec = numpy.array([1])
        # binary_vec = helper.get_binary_vector(G, robot, env)
        start = []
        goal = []

        while(len(start)<50 or len(goal)<50):
            print("in while loop, start = ", len(start), " goal = ", len(goal))
            print("calling get_valid_node, task_id = ", task_id)
            random_node_1, status = helper.get_valid_node(task_id, G, robot, env, bookcase_pose, None)
            random_node_2, status = helper.get_valid_node(task_id, G, robot, env, bookcase_pose, None)
            
            if(not helper.is_trivial(G.node[random_node_1]['state'], G.node[random_node_2]['state'], env, robot)):
              start.append(random_node_1)
              goal.append(random_node_2)   

        if(len(start) == 50 and len(goal) == 50):    
            print("write_one_dir-----------------------------------------------------------------")    
            write_one_dir(task_id, env_no, start, goal, cond, binary_vec)
        else:
            print("no of start or goal posiitons is under limit")

        env_no += 1




def generate_data(task_id, datadir, robot, env, G):
    if(task_id=="T1"):
        table_box_below_above(task_id, datadir, robot, env, G)  
    elif(task_id=="T2"):
        table_box_left_right(task_id, datadir, robot, env, G)
    else:
        print("InValid Task ID, HELP:")
        print("T1 : Table_Box Problem, moving stick below to above the table")
        print("T2 : Table_Box Problem, moving stick across the block")
        return 
    # elif(task_id=="T3"):
    #     bookcase_front(task_id, robot, env, G)
    # elif(task_id=="T4"):
    #     bookcase_table(task_id, robot, env, G)          #bookcase on right, table in front                    

def main():
    print("start main")
    
    table_pose = numpy.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--datadir',type=str,required=True)
    parser.add_argument('--taskid',type=str,required=True)
    # parser.add_argument('--datadir',type=str,required=True)
    args = parser.parse_args()

    args.datadir = args.datadir.strip('/')
    if(not os.path.isdir(args.datadir)):
      raise Exception(" Datadir Not Found!! args : ", args.datadir)

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    
    G = nx.read_graphml(args.graphfile)

    flat_base_file = os.path.join(objects_path,'objects/flat_base.kinbody.xml')
    flat_base = env.ReadKinBodyXMLFile(flat_base_file)
    env.AddKinBody(flat_base)
    base_pose = table_pose.copy()
    base_pose[:3,3] = 0,0,-0.04
    flat_base.SetTransform(base_pose)

    print("pre generate_data")
    # generate_data("T2", args.datadir, robot, env, G) 
      
    generate_data(args.taskid, args.datadir, robot, env, G)   
    # print("post generate_data") 

if __name__ == '__main__':
  main()