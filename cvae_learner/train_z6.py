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

def list_all_dir(data_dir):
    task_dirs = os.listdir(data_dir)

    list_dir = []
    for task_dir in task_dirs:
        env_dirs = os.listdir(data_dir+"/"+task_dir)
        for env_dir in env_dirs:
            list_dir.append(data_dir +"/"+ task_dir +"/"+ env_dir)
    return list_dir  

def process_it(G, directory):
    start = np.loadtxt(directory+"/start_node.txt")
    goal = np.loadtxt(directory+"/goal_node.txt")
    cond = np.loadtxt(directory+"/conditions.txt")
    # occ_grid = occ_grid.split(",")
    path_nodes = []
    i = 0
    all_data = []
    with open(directory + "/path_nodes.txt", 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
#             print(line)
#             print("\n\n")
            
            s = state_to_numpy(G.node[str(int(start[i]))]['state'])
            g = state_to_numpy(G.node[str(int(goal[i]))]['state'])
#             og = occ_grid[i]
            path_nodes = str(line).split(",")
            # print(path_nodes)
            for path_node in path_nodes:
                if(path_node=='-1'):
                    continue
                node_conf = state_to_numpy(G.node[path_node]['state'])
                curr_node = np.array([])
                # print("Data = ",node_conf, s, g, occ_grid)
#                     print("\n")
#                     print("node_conf = ", node_conf, " s = ", s, " g = ",g)

                curr_node = np.concatenate((node_conf, s, g, cond))
#                     print("shape of curr_node = ", curr_node.shape)
                all_data.append(curr_node)
            i+=1
    return all_data

G = nx.read_graphml("graphs/modified_graph_8D30000_3lacs.graphml")
data_dir = "data_21June"

directories = list_all_dir(data_dir)
final_data = []
flag = 0

for directory in directories:
    data = np.array(process_it(G, directory))
    if(flag==0):
        flag=1
        final_data = data
    else:
        final_data = np.concatenate((final_data, data))


data = final_data
np.random.shuffle(data)
print("shape of array: ",data.shape)

numEntries = data.shape[0]

# split the inputs and conditions into test train (to be processed in the next step into an occupancy grid representation)
ratioTestTrain = 0.8;
numTrain = int(numEntries*ratioTestTrain)

X_train = data[0:numTrain,0:dim] # state: x, y, z, xdot, ydot, zdot
c_train = data[0:numTrain,dim:dataElements] # conditions: gaps, init (6), goal (6)

X_test = data[numTrain:numEntries,0:dim]
c_test = data[numTrain:numEntries,dim:dataElements]

#########################################################
c_train1 = []
c_test1 = []
c_train1 = c_train
c_test1 = c_test
#########################################################
numTest = X_test.shape[0]

# print("shape of final obstacle = ",obs.shape)
print("shape of c_train1 = ", c_train1.shape)
print("shape of c_test1 = ",c_test1.shape)

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

saver = tf.train.Saver()
path_ = os.getcwd() + "/checkpoints_z6_21Junedata_less/model.ckpt"
print("path = ",path_)
print("numTrain = ",numTrain)
try:
    saver.restore(sess, path_)
    print("Model Restored!!")
except Exception as e:
    print("Could not restore checkpoint!")
    print(e)

loss_ = []  
mu = []
sigma = []
n_ = []

def main():
	global it
	for it in range(it,it+500001):
	    # randomly generate batches
	    batch_elements = [randint(0,numTrain-1) for n in range(0,mb_size)]
	    X_mb = X_train[batch_elements,:]
	    c_mb = c_train1[batch_elements,:]
	    
	    _, loss, r = sess.run([train_step, cvae_loss, recon_loss], feed_dict={X: X_mb, c: c_mb})

	#     _, loss, m, s = sess.run([train_step, cvae_loss, z_mu, z_sigma], feed_dict={X: X_mb, c: c_mb})

	    if it % 1000 == 0:
	        print('Iter: {}'.format(it))
	        print('Loss: {:.7}'. format(loss))
	        print('Recon_Loss: {:.7}'.format(r))
	        print("kl_loss = ", loss-r)
	#         mu.append(m)
	#         sigma.append(s)
	        n_.append(it)
	        loss_.append(loss)
	        saver.save(sess, path_)
	        print("saved session to ", path_)

if __name__ == '__main__':
	try:
		main()	
	except KeyboardInterrupt:
		print("Interrupted")
		hfont = {'fontname':'Helvetica'}

		plt.plot(n_, loss_, color = "blue", linewidth = 2)
		plt.xlabel("No of iterations ", **hfont)
		plt.ylabel("Total Loss", **hfont)
		plt.show()