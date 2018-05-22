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
import herbpy
import os

def calc_weight(config1, config2):
    return float(np.sum((config2-config1)**2))

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list) 

def save_modified_graph(G, env, robot):
    file_addr = "graphs/modified_graph.graphml"

    i = 0
    for node in G.nodes():
        state = state_to_numpy(G.node[node]['state'])
        robot.SetActiveDOFValues(state)

        ee_trans = robot.right_arm.GetEndEffectorTransform()
        trans = ee_trans[0:3,3]
        eepos = trans.tolist()
        G.node[node]['eePos'] = str(eepos[0])+" "+str(eepos[1])+" "+str(eepos[2])
        i += 1

    for i, edge in enumerate(G.edges()):
        u, v = edge
        G[u][v]['weight'] = calc_weight(state_to_numpy(G.node[u]['state']), state_to_numpy(G.node[v]['state']))
    nx.write_graphml(G, file_addr)    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    
    G = nx.read_graphml(args.graphfile)

    save_modified_graph(G, env, robot)

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

    # np.savetxt("data/DOF_Values_enum_nodes.txt", confns, delimiter=" ", fmt="%s")
    # np.savetxt("data/eePosns_enum_nodes.txt", eePosns, delimiter=" ", fmt="%s")   
        
