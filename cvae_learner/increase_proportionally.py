import networkx as nx
import numpy as np
import argparse

lower = [0, -2, -3, -4, -5, -2, -3, -0.5]
upper = [6, 2, 3, 4, 2, 2, 3, 0.5]

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()
    G = nx.read_graphml(args.graphfile)

    for node in list(G.nodes()):
        state = G.node[node]['state']
        conf = state_to_numpy(state)

        st = ""
        for i in range(len(conf)):
            conf[i] = lower[i] + conf[i]*(upper[i]-lower[i])
            st += str(conf[i])+" "
        G.node[node]['state'] = st.strip()
    nx.write_graphml(G, args.graphfile)

if __name__ == '__main__':
    main()