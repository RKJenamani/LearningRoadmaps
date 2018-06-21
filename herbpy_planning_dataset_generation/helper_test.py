import herbpy
import numpy
from random import choice
import networkx as nx
import math

EDGE_DISCRETIZATION = 7
TABLE_X, TABLE_Y, TABLE_Z = 1, 0, 0
TABLE_XW, TABLE_YW, TABLE_ZH = 0.75, 1.82, 0.8

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
    to_check.append(config1)

    for i in xrange(EDGE_DISCRETIZATION - 1):
        conf = config1 + step*(i+1)
        to_check.append(conf)

    return to_check    

def get_valid_node(task_id, G, robot, env, obj1_pose, obj2_pose):
    valid_node = 0

    if(task_id=="T1"):
        t_x, t_y, t_z = obj1_pose[0,3], obj1_pose[1,3], obj1_pose[2,3]
        sb_x, sb_y, sb_z = obj2_pose[0,3], obj2_pose[1,3], obj2_pose[2,3]
        while(valid_node==0):
            random_node = choice(list(G.nodes()))
            config = state_to_numpy(G.node[random_node]['state'])
            robot.SetActiveDOFValues(config)
            if(env.CheckCollision(robot)==True):
                continue

            ee_trans = robot.right_arm.GetEndEffectorTransform()
            trans = ee_trans[0:3,3]
            eepos = trans.tolist()
            if( math.fabs(eepos[0] - t_x) < TABLE_XW/2 and math.fabs(eepos[1] - t_y) < TABLE_YW/2 ):
                if( eepos[2]<TABLE_ZH-0.2 ):
                    return random_node, "s"
                if(eepos[2]>sb_z and eepos[2]<sb_z+0.3):
                    if(eepos[1] > sb_y-0.2 and eepos[1]< sb_y+0.2):
                        return random_node, "g"

    elif(task_id=="T2"):
        t_x, t_y, t_z = obj1_pose[0,3], obj1_pose[1,3], obj1_pose[2,3]
        b_x, b_y, b_z = obj2_pose[0,3], obj2_pose[1,3], obj2_pose[2,3]
        while(valid_node==0):
            random_node = choice(list(G.nodes()))
            config = state_to_numpy(G.node[random_node]['state'])
            robot.SetActiveDOFValues(config)
            if(env.CheckCollision(robot)==True):
                continue

            ee_trans = robot.right_arm.GetEndEffectorTransform()
            trans = ee_trans[0:3,3]
            eepos = trans.tolist()
            if( math.fabs(eepos[0] - t_x) < TABLE_XW/2 and math.fabs(eepos[1] - t_y) < TABLE_YW/2 + 0.2 and eepos[2]>TABLE_ZH and eepos[2]<TABLE_ZH+0.4):
                if( eepos[1]<b_y and eepos[1] > b_y-0.3 ):
                    return random_node, "s"
                if(eepos[1]>b_y and eepos[1]<b_y+0.3):
                    return random_node, "g"   
    elif(task_id=="T3"):
        bookcase_pose = obj1_pose
        xl = 0.4
        yl = 0.75
        zl = 1.45
        valid_node = 0
        print("Entering while loop")
        max_x = 0
        min_y = 10
        while(valid_node==0):
            random_node = choice(list(G.nodes()))
            config = state_to_numpy(G.node[random_node]['state'])
            robot.SetActiveDOFValues(config)
            if(env.CheckCollision(robot)==True):
                continue

            ee_trans = robot.right_arm.GetEndEffectorTransform()
            trans = ee_trans[0:3,3]
            eepos = trans.tolist()

            # if(eepos[0]>max_x):
            #     max_x = eepos[0]
            if(eepos[1]<min_y):
                min_y = eepos[1]    
            # print("max_x = ", max_x, "min_y = ", min_y, "eepos = ",eepos)
            # raw_input("Press Enter")   

            # print(eepos[1], bookcase_pose[0,3]+0.1, min_y)
            cond1 =  eepos[1]<bookcase_pose[0,3]+0.1
            cond2 =  math.fabs(eepos[0]-bookcase_pose[1,3])<yl/2
            cond3 =  eepos[2]<zl

            # print("conditions = ",cond1, cond2, cond3)

            # if(eepos[0]>bookcase_pose[0,3]-0.1 and math.fabs(eepos[1]-bookcase_pose[1,3])<yl/2 and eepos[2]<zl):
            #     print("valid_node found!")
            #     print("\n\n\n\n")
            #     return random_node, "v"

            if(eepos[1]<bookcase_pose[0,3]+0.1 and math.fabs(eepos[0]-bookcase_pose[1,3])<yl/2 and eepos[2]<zl):
                print("valid_node found!")
                print("\n\n\n\n")
                return random_node, "v"                                     

def is_trivial(state1, state2, env, robot):
    configs_to_check = edge_to_configs(state1,state2)

    edge_free = 1
   
    for cc in configs_to_check:
        robot.SetActiveDOFValues(cc)
        if env.CheckCollision(robot) == True:
            edge_free = 0
            break

    return edge_free

def get_binary_vector(G, G1, robot, env):
    print("Creating Binary Vector")
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
                binary_vec.append(['0', u, v])
                G1.remove_edge(u, v)
            else:
                binary_vec.append(['1', u, v])   
    print("Returning Binary Vector")             
    return binary_vec, G1   