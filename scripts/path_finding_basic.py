##############################################################################

# import packages

##############################################################################


import numpy as np

import heapq

import matplotlib.pyplot as plt

import serial

import time

from matplotlib.pyplot import figure

from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk1

##############################################################################

# plot grid

##############################################################################


grid = np.array([

    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

     [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

 [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

 [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

 [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

 [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

[[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

[[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],

     [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]



])

#
# fig = plt.figure(figsize=(12, 12))
#
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlim(0,7)
# ax.set_ylim(0,13)
# ax.set_zlim(0,16)
# # ax.scatter3D(goal[0], goal[1], goal[2], marker="*", color="red", s=100)
# for punkt, wartosc in np.ndenumerate(grid):
#  # print('wartosc'+ str(int(wartosc)) + 'punkt = ('+ str(int(punkt[0])) + ','+ str(int(punkt[1])) + ','+ str(punkt[2])+ ')')
#  if wartosc == 1:
#   ax.scatter3D(int(punkt[0]), int(punkt[1]), int(punkt[2]), marker="o", color="black", s=100)
#  # elif wartosc == 0:
#  #  ax.scatter3D(int(punkt[0]), int(punkt[1]), int(punkt[2]), marker="x", color="green", s=100)
# plt.show()

start = (5, 0, 2)

goal = (5, 6, 2)


##############################################################################

# heuristic function for path scoring

##############################################################################


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


##############################################################################

# path finding function

##############################################################################


def astar(array, start, goal):
    neighbors = [(0, 0, -1), (0, 1, -1), (1, 1, -1), (1, 0, -1), (1, -1, -1), (0, -1, -1), (-1, -1, -1), (-1, 0, -1),
                 (0, 1, 0), (1, 1, 0), (1, 0, 0), (1, -1, 0), (0, -1, 0), (-1, -1, 0), (-1, 0, 0),
                 (0, 0, 1), (0, 1, 1), (1, 1, 1), (1, 0, 1), (1, -1, 1), (0, -1, 1), (-1, -1, 1), (-1, 0, 1)]

    close_set = set()

    came_from = {}

    gscore = {start: 0}

    fscore = {start: heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:

        current = heapq.heappop(oheap)[1]

        if current == goal:

            data = []

            while current in came_from:
                data.append(current)

                current = came_from[current]

            return data

        close_set.add(current)

        for x, y, z in neighbors:

            neighbor = current[0] + x, current[1] + y, current[2] + z #sprawdzamy po kolei każdego sąsiada

            tentative_g_score = gscore[current] + heuristic(current, neighbor) #liczenie T

            if 0 <= neighbor[0] < array.shape[0]: #sprawdzanie czy w osi x sąsiad się mieści

                if 0 <= neighbor[1] < array.shape[1]: #sprawdzanie czy w osi y sąsiad się mieści

                    if 0 <= neighbor[2] < array.shape[2]: #sprawdzanie czy w osi z sąsiad się mieści

                        if array[neighbor[0]][neighbor[1]][neighbor[2]] == 1: #sprawdzanie czy sąsiad nie jest barierą
                            continue #continue oznacza przejscie od razu do kolejnej pętli
                    else:

                     # array bound z walls

                     continue

                else:

                    # array bound y walls

                    continue

            else:

                # array bound x walls

                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue #jeśli sąsiad jest w zbiorze zamkniętym i jego punkty są większe bądź równe poprzednim to przejsc do
                #kolejnej petli

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False


route = astar(grid, start, goal)

route = route + [start]

route = route[::-1]

print(route)
for index,value in enumerate(route):
    qModel=myVirtualRobotArm.inverseKinematics(value[0], value[1], z_coords[index])
##############################################################################

# plot the path

##############################################################################


# extract x and y coordinates from route list

x_coords = []

y_coords = []

z_coords = []

for i in (range(0, len(route))):

    x = route[i][0]

    y = route[i][1]

    z = route[i][2]

    x_coords.append(x)

    y_coords.append(y)

    z_coords.append(z)

x_coords = np.array(x_coords)

y_coords = np.array(y_coords)

z_coords = np.array(z_coords)

# plot map and path

fig, ax = plt.subplots(figsize=(12, 12))

# ax.imshow(grid, cmap=plt.cm.Dark2)
ax = fig.add_subplot(111, projection='3d')

ax.scatter3D(goal[0], goal[1], goal[2], marker="*", color="red", s=100)

for punkt, wartosc in np.ndenumerate(grid):
 # print('wartosc'+ str(int(wartosc)) + 'punkt = ('+ str(int(punkt[0])) + ','+ str(int(punkt[1])) + ','+ str(punkt[2])+ ')')
 if wartosc == 1:
  ax.scatter3D(int(punkt[0]), int(punkt[1]), int(punkt[2]), marker="o", color="black", s=100)

ax.plot3D(x_coords, y_coords, z_coords, color="pink")
print('x_coords: ' + str(x_coords) + ' y_cords:' + str(y_coords) + ' z_coords:' + str(z_coords))

plt.show()


y_coords +=13
print(y_coords)
x_coords -=5

y_coords = y_coords*10
x_coords = x_coords*10
z_coords = z_coords*10
print(x_coords)
print(y_coords)
print(z_coords)


myVirtualRobotArm = EEZYbotARM_Mk1(
    initial_q1=0, initial_q2=90, initial_q3=-90)

# Define end effector open and closed angle
servoAngle_EE_closed = 10
servoAngle_EE_open = 180
q=[]
# Compute inverse kinematics
for index,value in enumerate(z_coords):
    qModel=myVirtualRobotArm.inverseKinematics(y_coords[index], x_coords[index], z_coords[index])
    myVirtualRobotArm.updateJointAngles(qModel[0],qModel[1],qModel[2])
    q.append(myVirtualRobotArm.map_kinematicsToServoAngles())
print(q)
ser1 = serial.Serial('COM6', baudrate=9600, timeout=1)
time.sleep(3)
for index in range(0,len(q)):
    wiadomoscFB = '<'+'F,'+ str(int(q[index][1]))+'>'
    wiadomoscUD = '<' + 'U,' + str(int(q[index][2])) + '>'
    wiadomoscRo = '<' + 'R,' + str(int(q[index][0])) + '>'
    wiadomoscGr = '<' + 'G,' + str(int(servoAngle_EE_open)) + '>'
    # print(type(wiadomosc))
    # print(wiadomosc)
    ser1.write(bytes(wiadomoscFB, 'utf-8'))
    ser1.write(bytes(wiadomoscUD, 'utf-8'))
    ser1.write(bytes(wiadomoscRo, 'utf-8'))
    ser1.write(bytes(wiadomoscGr, 'utf-8'))