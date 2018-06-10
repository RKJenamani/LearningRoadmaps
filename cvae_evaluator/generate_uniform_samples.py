import numpy as np 
import networkx as nx 
from math import sqrt

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def connect_knn(G, K):
    print("len(G) = ",len(list(G.nodes())))
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)
        G1 = G.copy()

        for k in range(K):
            # print("node = ",node)
            w = 1000000
            sn = None
            for node1 in G1.nodes():
                if(node == node1):
                    continue
                state1 = G1.node[node1]['state']
                conf1  = state_to_numpy(state1)
                if(calc_weight(conf, conf1) < w):
                    w = calc_weight(conf, conf1)
                    sn = node1

            # if(check_for_collision(node, sn)==1):
            G.add_edge(node, sn)
            # print("sn = ",sn)
            # print("connected edge from ",node, " to ",sn)
            G[node][sn]['weight'] = w
            G1.remove_node(sn)
    return G    

def main():
    n_samples = [200, 300, 400, 500]
    K = [5, 10, 15]

    for no_samples in n_samples:
            output_samples = []
            for i in range(no_samples):
                curr_sample = ""
                for j in range(7):
                    if(j==0):
                        curr_sample += str(np.random.uniform(0,6))+" "
                    elif(j==1):
                        curr_sample += str(np.random.uniform(-2,2))+" "
                    elif(j==2):
                        curr_sample += str(np.random.uniform(-3,3))+" "
                    elif(j==3):
                        curr_sample += str(np.random.uniform(-4,4))+" "
                    elif(j==4):
                        curr_sample += str(np.random.uniform(-5,2))+" "
                    elif(j==5):
                        curr_sample += str(np.random.uniform(-2,2))+" "
                    elif(j==6):
                        curr_sample += str(np.random.uniform(-3,3))+" "                        
                output_samples.append(curr_sample)
            np.savetxt("uniform_samples/uniform_samples_"+str(no_samples)+".txt", np.array(output_samples), delimiter=" ", fmt="%s")    

if __name__ == '__main__':
    main()