import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

TABLE_X_MIN = 0.5
TABLE_X_MAX = 1.5
TABLE_Y_MIN = -1
TABLE_Y_MAX = 1
TABLE_Z = 1
TABLE_ZT = 0.4*100

def load_array(fileaddr, flag = ","):
    array = []
    with open(fileaddr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            l = np.array(line.split(flag))
            l = l.astype(np.float)
            array.append(l)
    return np.array(array)  

def plot_surface(ax):
    X = np.arange(TABLE_X_MIN, TABLE_X_MAX, 0.01)
    Y = np.arange(TABLE_Y_MIN, TABLE_Y_MAX, 0.01)
    X, Y = np.meshgrid(X, Y)
    Z = np.ones(X.shape)*TABLE_Z
    surf = ax.plot_surface(X, Y, Z, color = 'g', linewidth =TABLE_ZT)
    return ax

def load_eepositions(path_node_no_addr, eepositions_path_addr, index):
    eepositions = load_array(eepositions_path_addr)
    path_no_nodes = load_array(path_node_no_addr)
    nodes = path_no_nodes[index]
    nodes = nodes.astype(int)
    print(nodes)
    path_eepos = {}
    i = 0
    for node in nodes:
        path_eepos[i] = (eepositions[node])
        i += 1
    print(path_eepos)
    return path_eepos    

def plot_end_effector_positions(eepositions_path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = []
    y = []
    z = []
    for key in eepositions_path:
        value = eepositions_path[key]
        x.append(value[0])
        y.append(value[1])
        z.append(value[2])
    ax.scatter(x, y, z, c = 'b') 
    print("no of points = ",len(x))

    ax = plot_surface(ax)

    ax.set_xlabel('X Pos')
    ax.set_ylabel('Y Pos')
    ax.set_zlabel('Z Pos')

    plt.show()

def main():
    path_node_no_addr = "path_nodes_no.txt"
    eepositions_path_addr = "eePosns_enum_nodes.txt"
    eepositions_path = load_eepositions(path_node_no_addr, eepositions_path_addr, 2)

    plot_end_effector_positions(eepositions_path)

if __name__ == '__main__':
    main()