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
from random import choice

EDGE_DISCRETIZATION = 7

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

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return numpy.array(val_list) 

def edge_to_configs(state1, state2):

    config1 = state_to_numpy(state1)
    config2 = state_to_numpy(state2)

    diff = config2 - config1
    step = diff/EDGE_DISCRETIZATION

    to_check = list()

    for i in xrange(EDGE_DISCRETIZATION - 1):
        conf = config1 + step*(i+1)
        to_check.append(conf)

    return to_check    

def check_for_collisions(G, robot, env):
    binary_vec = []
    for i,edge in enumerate(G.edges()):
            u,v = edge
            state1 = G.node[u]['state']
            state2 = G.node[v]['state']
            configs_to_check = edge_to_configs(state1,state2)

            edge_free = 1

            for cc in configs_to_check:
                robot.SetActiveDOFValues(cc)

                if env.CheckCollision(robot) == True:
                    edge_free = 0
                    break
            if(not edge_free):
                binary_vec.append('0')
            else:
                binary_vec.append('1')    
    return binary_vec     

# to classify ee_pos as invalid or valid_start or valid_goal
def classify_eepos(eepos, table_pose):
    return 2 if eepos[2]>0.5 else 1

#to append each output in the corresponding file
# start and goal node have valid end effectors.
# binary vectors of enumerated edges of the roadmap
def write_one_dir(task_id, env_no,start_node, goal_node, condition_vectors, binary_vec):
    dir_path = "data/"+task_id+"/"+str(env_no)
    try:
        os.mkdir("data/"+task_id, 0755)
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
    numpy.savetxt(dir_path+"/conditions.txt", condition_vectors, delimiter=" ", fmt="%s") #here only table_pose
def object_around_table(task_id, robot, env, G):
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

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

    # n environment setup => n sub directory inside task directory
    for env_no in range(10):
        xpos = XMIN + numpy.random.rand()*(XMAX - XMIN)
        ypos = YMIN + numpy.random.rand()*(YMAX - YMIN)

        print '{0} ; {1}'.format(xpos,ypos)

        table_pose[0,3] = xpos
        table_pose[1,3] = ypos

        table.SetTransform(table_pose)

        binary_vec = check_for_collisions(G, robot, env)

        cond = ""
        cond = table_pose.ravel()
        # TODO: create occupancy grid ###################
        # cond = (table_pose.ravel()).astype(str)
        # cond = ",".join(condn)
        #################################################

        start = []
        goal  = []

        while(len(start)<50 or len(goal)<50):
            random_node = choice(list(G.nodes()))
            state = state_to_numpy(G.node[random_node]['state'])
            robot.SetActiveDOFValues(state)

            ee_trans = robot.right_arm.GetEndEffectorTransform()
            trans = ee_trans[0:3,3]
            eepos = trans.tolist()
            
            check = classify_eepos(eepos, table_pose)

            # 1 -> valid start_pos, 2 -> valid goal_pos, 0 -> invalid_pos
            if(check==1 and len(start)<50):
                start.append(random_node)
            elif (check==2 and len(goal)<50):
                goal.append(random_node)

        if(len(start) == 50 and len(goal) == 50):        
            write_one_dir(task_id, env_no, start, goal, cond, binary_vec)
        else:
            print("no of start or goal posiitons is under limit")            


def generate_data(task_id, robot, env, G):
    if(task_id=="T1"):
        object_around_table(task_id, robot, env, G)                    

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    
    G = nx.read_graphml(args.graphfile)

    generate_data("T1", robot, env, G)    
    nx.write_graphml(G, "graphs/weighted_graph.graphml")