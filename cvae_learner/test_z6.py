import openravepy
import rospy
import os
import herbpy
from prpy import serialization

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
import csv
from random import randint, random
import time
import argparse

env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')

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
    
robot.right_arm.SetActive()
# Load table from pr_ordata
table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
tall_white_box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
table = env.ReadKinBodyXMLFile(table_file)
env.AddKinBody(table)
tall_white_box = env.ReadKinBodyXMLFile(tall_white_box_file)
env.AddKinBody(tall_white_box)
stick_file = os.path.join(objects_path,'objects/stick.kinbody.xml')
stick = env.ReadKinBodyXMLFile(stick_file)
env.AddKinBody(stick)

# Workspace problem with several narrow gaps

# (restrict tensorflow memory growth)
os.environ["CUDA_VISIBLE_DEVICES"]="1"
config = tf.ConfigProto()
config.gpu_options.allow_growth=True

# neural network parameters
mb_size = 256
h_Q_dim = 512
h_P_dim = 512

c = 0
# learning rate
lr = 1e-4

# problem dimensions
dim = 8
dataElements = dim*3 + 32 # sample (7D), init (7D), goal (7D), cond (16+16 points (table + box)) //total = 37

z_dim = 6 # latent
X_dim = dim # samples
y_dim = dim # reconstruction of the original point
c_dim = dataElements - dim # dimension of conditioning variable

#Pre-Processing

import os
import argparse
import networkx as nx
import math        
import numpy as np

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

#load test cases
from random import randint
step_size = 0.1
stick_len = 0.4


# define networks
X = tf.placeholder(tf.float32, shape=[None, X_dim])
c = tf.placeholder(tf.float32, shape=[None, c_dim])

# Q
inputs_Q = tf.concat(axis=1, values=[X,c])

dense_Q1 = tf.layers.dense(inputs=inputs_Q, units=h_Q_dim, activation=tf.nn.relu)
dropout_Q1 = tf.layers.dropout(inputs=dense_Q1, rate=0.5)
dense_Q2 = tf.layers.dense(inputs=dropout_Q1, units=h_Q_dim, activation=tf.nn.relu)

z_mu = tf.layers.dense(inputs=dense_Q2, units=z_dim) # output here is z_mu
z_logvar = tf.layers.dense(inputs=dense_Q2, units=z_dim) # output here is z_logvar

# P
eps = tf.random_normal(shape=tf.shape(z_mu))
z = z_mu + tf.exp(z_logvar / 2) * eps
# z_sigma = tf.exp(z_logvar /2)
inputs_P = tf.concat(axis=1, values=[z,c])

dense_P1 = tf.layers.dense(inputs=inputs_P, units=h_P_dim, activation=tf.nn.relu)
dropout_P1 = tf.layers.dropout(inputs=dense_P1, rate=0.5)
dense_P2 = tf.layers.dense(inputs=dropout_P1, units=h_P_dim, activation=tf.nn.relu)

y = tf.layers.dense(inputs=dense_P2, units=X_dim) # fix to also output y

conditions = np.loadtxt("Results/conditions.txt", delimiter = " ")
print("conditions = ", conditions)

# training
########### comment in the one with 0 weight and uncomment the other ###########
w = [[1, 1, 1, 1, 1, 1, 1, 1]];
# w = [[1, 1, 1, 0, 0, 0]];
recon_loss = tf.losses.mean_squared_error(labels=X, predictions=y, weights=w)

# TODO: fix loss function for angles going around
kl_loss = 10**-4 * 2 * tf.reduce_sum(tf.exp(z_logvar) + z_mu**2 - 1. - z_logvar, 1)

cvae_loss = tf.reduce_mean(kl_loss + recon_loss)

train_step = tf.train.AdamOptimizer(lr).minimize(cvae_loss)

sess = tf.Session(config=config)
sess.run(tf.global_variables_initializer())
it = 0;

def restore_model(model_file):
    global sess
    saver = tf.train.Saver()
    path_ = model_file
    print("path = ",path_)
    # print("numTrain = ",numTrain)
    try:
        saver.restore(sess, path_)
        print("Model Restored!!")
        return 1
    except Exception as e:
        print("Could not restore checkpoint!")
        return 0

def edge_to_configs(state1, state2):
    EDGE_DISCRETIZATION = 7
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

#get output node posns
def get_o_node_posns(c_sample_seed, num_viz, count, env, robot, type_):
    if(type_=="Halton"):
        return []
    print("in get_o_node_posns")
    init = c_sample_seed[:2]
    goal = c_sample_seed[2:4]
    c_sample = np.repeat([c_sample_seed],num_viz,axis=0)
    y_viz, z_viz = sess.run([y, z], feed_dict={z: np.random.randn(num_viz, z_dim), c: c_sample})
    
    free_nodes = []
    y_viz = np.array(y_viz)
    print("output y_viz.shape = ",y_viz.shape)
    y_viz = list(y_viz)
    c_ = 0
    for cc in y_viz:
        robot.SetActiveDOFValues(cc[:-1])
        ee_trans = robot.right_arm.GetEndEffectorTransform()

        push_dir = ee_trans[:3,2]
        parr_dir = ee_trans[:3,1]
        stick_pose = ee_trans
        stick_pose[:3,3] += push_dir*step_size
        stick_pose[:3,3] += parr_dir*stick_len*cc[-1]
        stick.SetTransform(stick_pose)
        if env.CheckCollision(robot) or env.CheckCollision(stick) or robot.CheckSelfCollision():
#             print("in collision count = ", c_)
#             raw_input("Press Enter")
            c_ += 1
            continue
        else:
            free_nodes.append(cc)
        c_ += 1
    y_viz = np.array(free_nodes)
    print("no of nodes in collision = ", num_viz-y_viz.shape[0])
    print("returning o_node_posns of shape = ", y_viz.shape)
    return y_viz

#load shallow_graph
def load_halton_samples(num_viz, env, robot, stick):
    print("loading halton with "+str(num_viz)+"samples")
    shallow_G = nx.read_graphml("graphs/halton8D"+str(num_viz)+"_0.graphml")
    shallow_G.remove_edges_from(list(shallow_G.edges()))
    
    for node in list(shallow_G.nodes()):
        cc = state_to_numpy(shallow_G.node[node]['state'])
        robot.SetActiveDOFValues(cc[:-1])
        ee_trans = robot.right_arm.GetEndEffectorTransform()

        push_dir = ee_trans[:3,2]
        parr_dir = ee_trans[:3,1]
        stick_pose = ee_trans
        stick_pose[:3,3] += push_dir*step_size
        stick_pose[:3,3] += parr_dir*stick_len*cc[-1]
        stick.SetTransform(stick_pose)
        if env.CheckCollision(robot) or env.CheckCollision(stick) or robot.CheckSelfCollision():
             shallow_G.remove_node(node)
    print("after removing invalid nodes n = ", len(list(shallow_G.nodes())))
    return shallow_G

def remove_invalid_edges(G, env, robot, stick):
    print("gotta remove invalid edges")
    print("total no of edges = ", len(list(G.edges())))
    to_remove = []
    for edge in G.edges():
        u, v = edge
        state1 = G.node[u]['state']
        state2 = G.node[v]['state']
        configs_to_check = edge_to_configs(state1,state2)

        edge_free = 1
        for cc in configs_to_check:
            robot.SetActiveDOFValues(cc[:-1])
            ee_trans = robot.right_arm.GetEndEffectorTransform()
    
            push_dir = ee_trans[:3,2]
            parr_dir = ee_trans[:3,1]
            stick_pose = ee_trans
            stick_pose[:3,3] += push_dir*step_size
            stick_pose[:3,3] += parr_dir*stick_len*cc[-1]
            stick.SetTransform(stick_pose)
            if env.CheckCollision(robot) or env.CheckCollision(stick) or robot.CheckSelfCollision():
                edge_free = 0
        if(not edge_free):
            to_remove.append((u, v))

    for r in to_remove:
        G.remove_edge(r[0], r[1])

    print("no of edges in collision = ", len(to_remove))    
    print("removed inavlid edges")
    return G

import time
THRESHOLD = 2.54
def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def connect_within_thresh(G, lmbda, env, robot, stick):
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)
        G1 = G.copy()
        
        for node1 in G1.nodes():
            if(node == node1):
                continue
            state1 = G1.node[node1]['state']
            conf1  = state_to_numpy(state1)
            w = calc_weight(conf, conf1)
            if(w < lmbda*THRESHOLD):
                G.add_edge(node, node1)
                G[node][node1]['weight'] = w
    return G

def connect_knn(G, K, env, robot, stick):
    print("no of nodes = ", len(list(G.nodes())))
    print("gotta connect knn")
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)
        G1 = G.copy()

        for k in range(K):
            w = 1000000
            sn = None
            for node1 in G1.nodes():
                if(node == node1):
                    continue
                state1 = G1.node[node1]['state']
#                 print("node1 = ",node1)
#                 print("state1 = ",state1)
                
                conf1  = state_to_numpy(state1)
                if(calc_weight(conf, conf1) < w):
                    w = calc_weight(conf, conf1)
                    sn = node1

            # if(check_for_collision(node, sn)==1):
            G.add_edge(node, sn)
            # print("connected edge from ",node, " to ",sn)
            G[node][sn]['weight'] = w
            G1.remove_node(sn)
    print("connected knn")
    return G

def get_path_length(shallow_G, o_node_posns, src_conf, goal_conf, lmbda, env, robot, stick):
    G = shallow_G.copy()
    s_state = " ".join(str(src_conf[i]) for i in range(len(src_conf)))
    g_state = " ".join(str(goal_conf[i]) for i in range(len(goal_conf)))
    
    G.add_node('s', state = s_state)
    G.add_node('g', state = g_state)
    for i in range(len(o_node_posns)):
        curr_state = " ".join(str(o_node_posns[i,j]) for j in range(len(o_node_posns[i])))
#         print("adding curr_state i = ", i, " state = ", curr_state)
        G.add_node('o'+str(i), state = curr_state)
        
#     for node in list(G.nodes()):
#         print("node = ", node," state = ", G.node[node]['state'])
#     print("connecting knn")    
#     G = connect_knn(G, k, env, robot, stick)
    
    G = connect_within_thresh(G, lmbda, env, robot, stick)  
#     print("connected knn")
    print("removing invalid edges")
    t1 = time.time()
    G = remove_invalid_edges(G, env, robot, stick)
    t2 = time.time()
    print("removed invalid edges in time t = ", t2-t1," secs")
    path_length = None
    path_node_posns = []
    try:
        path_length = nx.dijkstra_path_length(G,'s','g', weight = 'weight')
        path_nodes = nx.dijkstra_path(G, 's', 'g', weight = 'weight')
        for node in path_nodes:
            path_node_posns.append(state_to_numpy(G.node[node]['state']))
    except Exception as e:
        pass
    return path_length, path_node_posns

from random import randint
from math import sqrt

def main(type_, model_file):
    if(not type_=="Halton"):
        check = restore_model(model_file)
        if(check==0):
            raise Exception("Could not Restore Model!!")

    num_v = []

    if(type_=="Halton"):
        num_v = [200, 400, 600, 800, 1000]
    elif(type_=="SP"):
        num_v = [100, 200, 300, 400, 500]
    elif(type_=="RF"):
        num_v = [100, 200, 300, 400, 500]
    elif(type_=="RF_Robust"):
        num_v = [100, 200, 300, 400, 500]
    else:
        raise Exception("Invalid Type, Options: Halton, SP, RF, RF_Robust ")
 
    for num_viz in num_v:
        shallow_G = load_halton_samples(num_viz, env, robot, stick)
        path_lengths = []
        all_path_nodes_posns = []
        lmbda_ = [1]
        
        for lmbda in lmbda_:
            count = 0
            # print(test_cases)
            # print(conditions[0])
            # return
            failed_c = 0
            for cond in conditions:
                # print("---------------------------------------cond = ",cond)
            #     if(count<7):
            #         count+=1
            #         continue
                print("cond.shape = ", cond.shape)
                table_pose = cond[16:32].reshape(4,4)
                box_pose = cond[32:].reshape(4,4)
                table.SetTransform(table_pose)
                tall_white_box.SetTransform(box_pose)
            #     print(cond)
                print("count = ", count)
                o_node_posns = get_o_node_posns(cond, num_viz, count, env, robot, type_)
                print("len(o_node_posns) = ", len(o_node_posns))
                print("got o_node posns")
                path_length, path_node_posns = get_path_length(shallow_G, o_node_posns, cond[:8], cond[8:16], lmbda, env, robot, stick)
                
                if(path_length==None):
                    path_lengths.append(-1)
                    failed_c += 1
                else:
                    path_lengths.append(path_length)
                print("path_length = ", path_length)
                all_path_nodes_posns.append(path_node_posns)
                count += 1
        print(path_lengths)
        print(failed_c)
        if(type_=="Halton"):
            np.savetxt("Results/"+type_+"_num_viz+"+str(num_viz)+"_lambda"+str(lmbda)+".txt", path_lengths, delimiter = " ")
        elif(type_=="SP"):
            np.savetxt("Results/"+type_+"_num_viz+"+str(2*num_viz)+"_lambda"+str(lmbda)+".txt", path_lengths, delimiter = " ")
        elif(type_=="RF"):
            np.savetxt("Results/"+type_+"_num_viz+"+str(2*num_viz)+"_lambda"+str(lmbda)+".txt", path_lengths, delimiter = " ")
        elif(type_=="RF_Robust"):
            np.savetxt("Results/"+type_+"_num_viz+"+str(2*num_viz)+"_lambda"+str(lmbda)+".txt", path_lengths, delimiter = " ")
        else:
            print(" Invalid Type!! ")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--type',type=str,required=True)
    parser.add_argument('--modelfile',type=str,required=True)
    args = parser.parse_args()

    main(args.type, args.modelfile)