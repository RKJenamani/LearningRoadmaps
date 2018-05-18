import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

TABLE_X_MIN = 0.5
TABLE_X_MAX = 1.5
TABLE_Y_MIN = -1
TABLE_Y_MAX = 1
TABLE_Z = 1
TABLE_ZT = 0.4*100

def plot_surface(ax):
    X = np.arange(TABLE_X_MIN, TABLE_X_MAX, 0.01)
    Y = np.arange(TABLE_Y_MIN, TABLE_Y_MAX, 0.01)
    X, Y = np.meshgrid(X, Y)
    Z = np.ones(X.shape)*TABLE_Z
    surf = ax.plot_surface(X, Y, Z, color = 'g', linewidth =TABLE_ZT)
    return ax


def load_eepositions(file_addr):
    eepositions = {}
    i = 0
    with open(file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            eepositions[str(i)] = np.float32(np.array(line.split(" ")))
            i += 1
    return eepositions

def plot_end_effector_positions(eepositions_start, eepositions_goal):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = []
    y = []
    z = []
    for key in eepositions_start:
        value = eepositions_start[key]
        x.append(value[0])
        y.append(value[1])
        z.append(value[2])
    ax.scatter(x, y, z, c = 'r') 

    x = []
    y = []
    z = []

    for key in eepositions_goal:
        value = eepositions_goal[key]
        x.append(value[0])
        y.append(value[1])
        z.append(value[2])
    ax.scatter(x, y, z, c = 'b') 

    ax.set_xlabel('X Pos')
    ax.set_ylabel('Y Pos')
    ax.set_zlabel('Z Pos')

    ax = plot_surface(ax)

    plt.show()

def main():
    file_addr = "start_node.txt"
    eepositions_start = load_eepositions(file_addr)

    file_addr = "goal_node.txt"
    eepositions_goal = load_eepositions(file_addr)

    plot_end_effector_positions(eepositions_start, eepositions_goal)

if __name__ == '__main__':
    main()