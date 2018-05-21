import herbpy
import os
import numpy as np
import math
import openravepy
import argparse
import sys
from prpy import serialization
import json
import networkx as nx

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list) 

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    
    G = nx.read_graphml(args.graphfile)

    confns = []
    eePosns = []
    for n in G.nodes():
    	print(n)
    	state = state_to_numpy(G.node[n]['state'])
        robot.SetActiveDOFValues(state)

        ee_trans = robot.right_arm.GetEndEffectorTransform()
        trans = ee_trans[0:3,3]
        eepos = trans.tolist()
        confns.append(state)
        eePosns.append(eepos)

    confns = np.array(confns)
    eePosns = np.array(eePosns) 

    np.savetxt("data/DOF_Values_enum_nodes.txt", confns, delimiter=" ", fmt="%s")
    np.savetxt("data/eePosns_enum_nodes.txt", eePosns, delimiter=" ", fmt="%s")   
        
