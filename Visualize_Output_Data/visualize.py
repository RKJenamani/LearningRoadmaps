import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

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

    plt.show()

def main():
    file_addr = "start_node.txt"
    eepositions_start = load_eepositions(file_addr)

    file_addr = "goal_node.txt"
    eepositions_goal = load_eepositions(file_addr)

    plot_end_effector_positions(eepositions_start, eepositions_goal)

if __name__ == '__main__':
    main()