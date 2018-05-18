import numpy as np
import random 
from math import fabs

TABLE_X, TABLE_Y, TABLE_Z = 1, 0, 0
TABLE_XW, TABLE_YW, TABLE_ZH = 0.75, 1.82, 1.2

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

def classify_ee_pos(eepos):
    global TABLE_X, TABLE_Y, TABLE_Z
    if( fabs(TABLE_X - eepos[0]) < TABLE_XW/2 + 0.2 and fabs(eepos[1] - TABLE_Y) < TABLE_YW/2 + 0.2 ):
        if(eepos[2] > TABLE_Z and eepos[2] < TABLE_Z + 0.4):
            return 1
        elif(eepos[2] > 1.2 and eepos[2] < 1.8):
            return 2
        else:
            return 0
    else:
        return 0

def get_start_and_goal_pos(eeposns, conf_values):
    start = []
    goal = []
    node_nos = []       # start node no
    node_nog = []       # goal node no
    s = eeposns.shape[0]   

    try_n = 100000
    while((len(start)<50 or len(goal)<50) and try_n>0):
        r = random.randint(0, s-1)
        
        check = classify_ee_pos(eeposns[r])

        # 1 -> valid start_pos, 2 -> valid goal_pos, 0 -> invalid_pos
        if(check==1 and len(start)<50):
            start.append(eeposns[r])
            node_nos.append(r)
        elif (check==2 and len(goal)<50):
            goal.append(eeposns[r])
            node_nog.append(r)
        try_n -= 1    

    if(len(start) == 50 and len(goal) == 50):        
        return np.array(start), np.array(goal), np.array(node_nos), np.array(node_nog)
    else:
        print("no of start or goal posiitons is under limit")
        return None, None

            
def main():
    ee_posFile = "eePosns_enum_nodes.txt"
    DOFvaluesFile = "DOF_Values_enum_nodes.txt"

    ee_posns = load_array(ee_posFile)
    conf_values = load_array(DOFvaluesFile, flag = " ")

    print("ee_pos.shape = ",ee_posns.shape)
    print("conf_values.shape = ",conf_values.shape)

    start, goal, node_nos, node_nog = get_start_and_goal_pos(ee_posns, conf_values)

    print("start = ",start)
    print("goal = ",goal)

    np.savetxt("start_node.txt", start, delimiter=" ", fmt="%s")
    np.savetxt("goal_node.txt", goal, delimiter=" ", fmt="%s")
    np.savetxt("start_node_no.txt", node_nos, delimiter=" ", fmt="%s")
    np.savetxt("goal_node_no.txt", node_nog, delimiter=" ", fmt="%s")    

if __name__ == '__main__':
    main()