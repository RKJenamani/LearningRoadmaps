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
            eepositions[str(i)] = np.float32(np.array(line.split(",")))
            i += 1
    return eepositions

def plot_end_effector_positions(eepositions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = []
    y = []
    z = []
    for key in eepositions:
        value = eepositions[key]
        x.append(value[0])
        y.append(value[1])
        z.append(value[2])
    ax.scatter(x, y, z) 
    plt.show()

def main():
	file_addr = "output_eepos.txt"
	eepositions = load_eepositions(file_addr)

	plot_end_effector_positions(eepositions)

if __name__ == '__main__':
	main()