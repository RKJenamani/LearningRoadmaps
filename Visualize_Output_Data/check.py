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

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    # env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    # robot.right_arm.SetActive()
    
    G = nx.read_graphml(args.graphfile)

    random_node = choice(list(G.nodes()))
    print("random_node = ", random_node)
    print("list(G.nodes())[list(G.nodes()).index(random_node)] = ", list(G.nodes())[list(G.nodes()).index(random_node)])

    print(list(G.nodes())[0:10])

    i = 0
    for i, node in enumerate(G.nodes()):
    	print(node)
    	if(i>10):
    		break
    	