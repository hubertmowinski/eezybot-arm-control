##############################################################################

# import packages

##############################################################################

from easyEEZYbotARM.serial_communication import arduinoController
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
def dodajOgraniczenia(siatka):
    np.asarray(np.where(siatka) == 1).T # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople,
    searchval = [1, 0] #jakiej kombinacji szukamy
    N = len(searchval)
    possibles = zip(*np.where(siatka == searchval[0])) # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople,
    # natomiast zip zamienia to na list-of-lists https://stackoverflow.com/questions/27175400/how-to-find-the-index-of-a-value-in-2d-array-in-python

    solns = []
    for p in possibles: #przeszukiwanie wszystkich otrzymanych 1 w celu znalezienia kombinacji

        check = grid[p[0], p[1], p[2]:p[2] + N]
        if np.all(check == searchval):
            solns.append(p)
            siatka[p[0]][p[1]][p[2]+N-1] = 1
    return siatka

def dodajOgraniczeniaX(siatka): #ograniczenie w osi x 6 cm za przeszkoda nie moze znaleźć się ramie robota bo zachaczy

    punktOgraniczenia = []
    np.asarray(np.where(siatka) == 1).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople, asarray działa jak zip tylko zwraca np array
    searchval = [1, 0]  # jakiej kombinacji szukamy
    searchval2 = [0, 1] #kombinacja górna
    N = len(searchval)
    possibles = np.vstack(np.where(siatka == searchval[0])).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople,
    a = np.sort([i[0] for i in possibles])
    i = a[-1] + 1
    j = a[0]  - 1
    while i <= len(siatka) - 1 and i <= a[-1] + 6:
        siatka[i] = siatka[a[-1]]
        i+=1
    if j >= 0:
        siatka[j] = siatka[a[0]]
    return siatka


def dodajOgraniczeniaY(siatka):  # ograniczenie w osi x 6 cm za przeszkoda nie moze znaleźć się ramie robota bo zachaczy
    # funkcja zwracająca pozycję 2 skrajnych punktów na górnej tej dalszej od robota krawędzi przeszkody
    punktOgraniczenia = []
    np.asarray(np.where(siatka) == 1).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople, asarray działa jak zip tylko zwraca np array
    searchval = [1, 0]  # jakiej kombinacji szukamy
    searchval2 = [0, 1]  # kombinacja górna
    N = len(searchval)
    possibles = np.vstack(
        np.where(siatka == searchval[0])).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople,

    # sortowanie w celu znalezienia współrzędnej x krawędzi dalszej
    for p in possibles:  # przeszukiwanie wszystkich otrzymanych 1 w celu znalezienia kombinacji
        check2 = siatka[p[0], p[1] - 1:p[1] + 1, p[2]]
        check3 = siatka[p[0], p[1]:p[1] + 2, p[2]]
        if np.all(check2 == searchval2):  # współrzędna x krawędzi dalszej
            siatka[p[0], p[1] - 1, p[2]] = 1  # insert żeby zawsze na początek wrzucał
            siatka[p[0], p[1] - 2, p[2]] = 1
        elif np.all(check3 == searchval):
            siatka[p[0], p[1] + 1, p[2]] = 1  #
            siatka[p[0], p[1] + 2, p[2]] = 1
    return siatka

def dodajOgraniczeniaZ(siatka):  # ograniczenie w osi x 6 cm za przeszkoda nie moze znaleźć się ramie robota bo zachaczy
    # funkcja zwracająca pozycję 2 skrajnych punktów na górnej tej dalszej od robota krawędzi przeszkody
    punktOgraniczenia = []
    np.asarray(np.where(siatka) == 1).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople, asarray działa jak zip tylko zwraca np array
    searchval = [1, 0]  # jakiej kombinacji szukamy
    N = len(searchval)
    possibles = np.vstack(
        np.where(siatka == searchval[0])).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople,

    # sortowanie w celu znalezienia współrzędnej x krawędzi dalszej
    for p in possibles:  # przeszukiwanie wszystkich otrzymanych 1 w celu znalezienia kombinacji
        check = siatka[p[0], p[1], p[2]:p[2] + N]
        if np.all(check == searchval):  # współrzędna x krawędzi dalszej
            siatka[p[0], p[1], p[2] + N-1] = 1

    return siatka

def wyszukajSkrajne(p7,p8):
    #funkcja zwracająca pozycję 2 skrajnych punktów na górnej tej dalszej od robota krawędzi przeszkody
    # punktOgraniczenia = []
    # np.asarray(np.where(siatka) == 1).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople, asarray działa jak zip tylko zwraca np array
    # searchval = [1, 0]  # jakiej kombinacji szukamy
    # searchval2 = [0, 1] #kombinacja górna
    # N = len(searchval)
    # possibles = np.vstack(
    #     np.where(siatka == searchval[0])).T  # where wyszukuje wszystkie 1 w siatka i zwraca to jako tople,
    #
    # a = np.sort([i[0] for i in possibles])
    # # sorted_array = an_array[np.argsort(possibles2[:, 1])]
    # # natomiast zip zamienia to na list-of-lists https://stackoverflow.com/questions/27175400/how-to-find-the-index-of-a-value-in-2d-array-in-python
    # # sortowanie w celu znalezienia współrzędnej x krawędzi dalszej
    # for p in possibles:  # przeszukiwanie wszystkich otrzymanych 1 w celu znalezienia kombinacji
    #     check1 = siatka[p[0], p[1], p[2]:p[2] + N]
    #     check2 = siatka[p[0], p[1] - 1:p[1] + 1, p[2]]
    #     check3 = siatka[p[0], p[1]:p[1] + 2, p[2]]
    #     if p[0] == a[-1] and np.all(check1 == searchval) and np.all(
    #             check2 == searchval2):  # współrzędna x krawędzi dalszej
    #         punktOgraniczenia.insert(0, p) #insert żeby zawsze na początek wrzucał
    #     elif p[0] == a[-1] and np.all(check1 == searchval) and np.all(check3 == searchval):
    #         punktOgraniczenia.append(p)
    punktOgraniczenia = []
    punktOgraniczenia.append(p7)
    punktOgraniczenia.append(p8)
    punktOgraniczenia[0][0] = 10 * (punktOgraniczenia[0][0] - 6)  # podziałki osi są przesuniete wzgledem siebie
    # zatem trzeba je tu przesunac to jest przesuniecie dla osi X
    punktOgraniczenia[1][0] = 10 * (punktOgraniczenia[1][0] - 6)
    # Use **kwargs if provided, otherwise use current values
    punktOgraniczenia[1][1] = 10 * (punktOgraniczenia[1][1] - 6)
    punktOgraniczenia[0][1] = 10 * (punktOgraniczenia[0][1] - 6)
    punktOgraniczenia[1][2] = 10 * punktOgraniczenia[1][2]
    punktOgraniczenia[0][2] = 10 * punktOgraniczenia[0][2]
    return punktOgraniczenia

def dodajPrzeszkode(siatka, przeszkoda):

    #
    x1y1z1 = np.mgrid[przeszkoda[0][0]:przeszkoda[5][0] + 1:1, przeszkoda[0][1]:przeszkoda[5][1] + 1:1,
             przeszkoda[0][2]:przeszkoda[5][2] + 1:1].reshape(3, -1).T
    x2y2z2 = np.mgrid[przeszkoda[0][0]:przeszkoda[6][0] + 1:1, przeszkoda[0][1]:przeszkoda[6][1] + 1:1,
             przeszkoda[0][2]:przeszkoda[6][2] + 1:1].reshape(3, -1).T
    x3y3z3 = np.mgrid[przeszkoda[0][0]:przeszkoda[3][0] + 1:1, przeszkoda[0][1]:przeszkoda[3][1] + 1:1,
             przeszkoda[0][2]:przeszkoda[3][2] + 1:1].reshape(3, -1).T
    x4y4z4 = np.mgrid[przeszkoda[2][0]:przeszkoda[7][0] + 1:1, przeszkoda[2][1]:przeszkoda[7][1] + 1:1,
             przeszkoda[2][2]:przeszkoda[7][2] + 1:1].reshape(3, -1).T
    x5y5z5 = np.mgrid[przeszkoda[1][0]:przeszkoda[7][0] + 1:1, przeszkoda[1][1]:przeszkoda[7][1] + 1:1,
             przeszkoda[1][2]:przeszkoda[7][2] + 1:1].reshape(3, -1).T
    x6y6z6 = np.mgrid[przeszkoda[4][0]:przeszkoda[7][0] + 1:1, przeszkoda[4][1]:przeszkoda[7][1] + 1:1,
             przeszkoda[4][2]:przeszkoda[7][2] + 1:1].reshape(3, -1).T
    # X,Y = np.mgrid[-5:5.1:0.5, -5:5.1:0.5]
    # xy = np.vstack((X.flatten(), Y.flatten())).T

    for index in x1y1z1:
        siatka[index[0]][index[1]][index[2]] = 1
    for index in x2y2z2:
        siatka[index[0]][index[1]][index[2]] = 1
    for index in x3y3z3:
        siatka[index[0]][index[1]][index[2]] = 1
    for index in x4y4z4:
        siatka[index[0]][index[1]][index[2]] = 1
    for index in x5y5z5:
        siatka[index[0]][index[1]][index[2]] = 1
    for index in x6y6z6:
        siatka[index[0]][index[1]][index[2]] = 1
    return siatka




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




##############################################################################

# heuristic function for path scoring

##############################################################################


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


##############################################################################

# path finding function

##############################################################################


def astar(array, start, goal, petla, sprawdzam):

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
        petla = petla + 1
        print('petla:' + str(petla))
        sprawdzam = 0

        current = heapq.heappop(oheap)[1]

        if current == goal:

            data = []

            while current in came_from:
                data.append(current)

                current = came_from[current]

            return data

        close_set.add(current)

        for x, y, z in neighbors:
            sprawdzam = sprawdzam + 1
            print('sprawdzam' + str(sprawdzam))
            neighbor = current[0] + x, current[1] + y, current[2] + z #sprawdzamy po kolei każdego sąsiada

            tentative_g_score = gscore[current] + heuristic(current, neighbor) #liczenie T

            x_sprawdz = 10*(neighbor[0] - 6)
            y_sprawdz = 10 * (neighbor[1] - 6)
            z_sprawdz = 10 * (neighbor[2])
            qModel = myVirtualRobotArm.inverseKinematics(x_sprawdz, y_sprawdz, z_sprawdz )
            myVirtualRobotArm.updateJointAngles(qModel[0], qModel[1], qModel[2])
            if myVirtualRobotArm.funkcjaOgraniczen(Ograniczenia) == 1:
                continue
            if 6 <= neighbor[0] < array.shape[0]: #sprawdzanie czy w osi x sąsiad się mieści

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
                print('DODANO PUNKT JEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!1')
                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False
liczbaIteracji = 0
znaleziono = False
# start = (10, 12, 6) #COLGATE
# goal = (19, 13, 15)#COLGATE
start = (15, 1, 1) #POWERBANK
goal = (15, 12, 0)#POWERBANK
# start = (17, 1, 1) #dwie przeszkody
# goal = (15, 13, 6)#dwie przeszkody
# goal = (19, 7, 2)
# start = (9, 12, 5)
home = (13,7,14)
# start = (14, 0, 2) #Arduino
# goal = (14, 12, 2) #Arduino
# goal = (17, 0, 2) #Arduino
# start = (17, 12, 1) #Arduino
myVirtualRobotArm = EEZYbotARM_Mk1(
        initial_q1=0, initial_q2=90, initial_q3=-90)
# przeszkoda = [[6, 4, 0], [6, 8, 0], [11, 4, 0], [11, 8, 0], [6, 4, 4], [6, 8, 4], [11, 4, 4], [11, 8, 4]]#COLGATE #zmienić wprowadzanie przeszkody na oś bazową robota
przeszkoda = [[5, 5, 0], [5, 8, 0], [19, 5, 0], [19, 8, 0], [5, 5, 9], [5, 8, 9], [19, 5, 9], [19, 8, 9]]#POWERBANK #zmienić wprowadzanie przeszkody na oś bazową robota
przeszkoda2 = [[5, 5, 0], [5, 7, 0], [19, 5, 0], [19, 7, 0], [5, 5, 9], [5, 7, 9], [19, 5, 9], [19, 7, 9]]#POWERBANK #zmienić wprowadzanie przeszkody na oś bazową robota
# przeszkoda1 = [[16, 6, 0], [16, 9, 0], [19, 6, 0], [19, 9, 0], [16, 6, 10], [16, 9, 10], [19, 6, 10], [19, 9, 10]]#dwie przeszkody  #zmienić wprowadzanie przeszkody na oś bazową robota
# przeszkoda2 = [[5, 9, 0], [5, 13, 0], [19, 9, 0], [19, 13, 5], [5, 9, 5], [5, 13, 5], [19, 9, 5], [19, 13, 5]]#dwie przeszkody #zmienić wprowadzanie przeszkody na oś bazową robota
# przeszkoda = [[5, 6, 0], [5, 8, 0], [11, 6, 0], [11, 8, 0], [5, 6, 7], [5, 8, 7], [11, 6, 7], [11, 8, 7]]
# przeszkoda = [[5, 6, 0], [5, 8, 0], [16, 6, 0], [16, 8, 0], [5, 6, 7], [5, 8, 7], [16, 6, 7], [16, 8, 7]] #Arduino
# przeszkoda = [[16, 6, 0], [16, 9, 0], [19, 9, 0], [19, 9, 0], [16, 6, 9], [16, 9, 9], [19, 6, 9], [19, 9, 9]] #Kropelka
gridPodstawowy = np.zeros((20,14,16))
gridPodstawowy2 = np.zeros((20,14,16))
gridPodstawowy = dodajPrzeszkode(gridPodstawowy, przeszkoda)
gridPodstawowy2 = dodajPrzeszkode(gridPodstawowy2, przeszkoda2)
grid = np.copy(gridPodstawowy)
grid = dodajOgraniczeniaZ(grid)
grid = dodajOgraniczeniaX(grid)
grid = dodajOgraniczeniaY(grid)
grid[15, 13, 6] = 0
Ograniczenia = wyszukajSkrajne(przeszkoda[6],przeszkoda[7])
print(Ograniczenia)
# while(znaleziono == False):
sprawdzam = 0
petla = 0
route2 = astar(grid, start, goal,petla,sprawdzam)

route2 = route2 + [start]

route2 = route2[::-1]

route1 = astar(grid, home, start,petla,sprawdzam)

route1 = route1 + [home]

route1 = route1[::-1]

route3 = astar(grid, goal, home,petla,sprawdzam)

route3 = route3 + [goal]

route3 = route3[::-1]



#     myVirtualRobotArm = EEZYbotARM_Mk1(
#         initial_q1=0, initial_q2=90, initial_q3=-90)
#     znaleziono = True
#     print(route)
#     for value in route:
#         qModel = myVirtualRobotArm.inverseKinematics(value[0], value[1], value[2])
#         myVirtualRobotArm.updateJointAngles(qModel[0], qModel[1], qModel[2])
#         if myVirtualRobotArm.funkcjaOgraniczen(Ograniczenia) == 1:
#             znaleziono = False
#             grid = dodajOgraniczenia(grid)
#             liczbaIteracji = liczbaIteracji + 1
#             break
# print(grid[2])
# print("liczba iteracji:" + str(liczbaIteracji))
##############################################################################

# plot the path

##############################################################################


# extract x and y coordinates from route list

x_coords1 = []

y_coords1 = []

z_coords1 = []

x_coords2 = []

y_coords2 = []

z_coords2 = []

x_coords3 = []

y_coords3 = []

z_coords3 = []

for i in (range(0, len(route2))):

    x = route2[i][0]

    y = route2[i][1]

    z = route2[i][2]

    x_coords2.append(x)

    y_coords2.append(y)

    z_coords2.append(z)

x_coords2 = np.array(x_coords2)

y_coords2 = np.array(y_coords2)

z_coords2 = np.array(z_coords2)

for i in (range(0, len(route1))):

    x = route1[i][0]

    y = route1[i][1]

    z = route1[i][2]

    x_coords1.append(x)

    y_coords1.append(y)

    z_coords1.append(z)

x_coords1 = np.array(x_coords1)

y_coords1 = np.array(y_coords1)

z_coords1 = np.array(z_coords1)

for i in (range(0, len(route3))):

    x = route3[i][0]

    y = route3[i][1]

    z = route3[i][2]

    x_coords3.append(x)

    y_coords3.append(y)

    z_coords3.append(z)

x_coords3 = np.array(x_coords3)

y_coords3 = np.array(y_coords3)

z_coords3 = np.array(z_coords3)

# plot map and path

fig, ax = plt.subplots(figsize=(12, 12))

 #ax.imshow(grid, cmap=plt.cm.Dark2)
ax = fig.add_subplot(111, projection='3d')

ax.scatter3D(goal[0], goal[1], goal[2], marker="*", color="grey", s=30)

# for punkt, wartosc in np.ndenumerate(grid):
#  # print('wartosc'+ str(int(wartosc)) + 'punkt = ('+ str(int(punkt[0])) + ','+ str(int(punkt[1])) + ','+ str(punkt[2])+ ')')
#  if wartosc == 1:
#     ax.scatter3D(int(punkt[0]), int(punkt[1]), int(punkt[2]), marker="o", color="grey", s=15)

for punkt, wartosc in np.ndenumerate(gridPodstawowy2):
 # print('wartosc'+ str(int(wartosc)) + 'punkt = ('+ str(int(punkt[0])) + ','+ str(int(punkt[1])) + ','+ str(punkt[2])+ ')')
 if wartosc == 1:
    ax.scatter3D(int(punkt[0]), int(punkt[1]), int(punkt[2]), marker="o", color="blue", s=30)


ax.plot3D(x_coords1, y_coords1, z_coords1, color="pink")
print('x_coords: ' + str(x_coords1) + ' y_cords:' + str(y_coords1) + ' z_coords:' + str(z_coords1))

ax.plot3D(x_coords2, y_coords2, z_coords2, color="green")
print('x_coords: ' + str(x_coords2) + ' y_cords:' + str(y_coords2) + ' z_coords:' + str(z_coords2))

ax.plot3D(x_coords3, y_coords3, z_coords3, color="red")
print('x_coords: ' + str(x_coords3) + ' y_cords:' + str(y_coords3) + ' z_coords:' + str(z_coords3))

plt.show()


y_coords1 -= 7


y_coords1 = y_coords1 * 10
x_coords1 = x_coords1 * 10
z_coords1 = z_coords1 * 10

y_coords2 -= 7


y_coords2 = y_coords2 * 10
x_coords2 = x_coords2 * 10
z_coords2 = z_coords2 * 10

y_coords3 -= 7


y_coords3 = y_coords3 * 10
x_coords3 = x_coords3 * 10
z_coords3 = z_coords3 * 10

print(x_coords1)
print(y_coords1)
print(z_coords1)

print(x_coords2)
print(y_coords2)
print(z_coords2)

print(x_coords3)
print(y_coords3)
print(z_coords3)


myVirtualRobotArm = EEZYbotARM_Mk1(
    initial_q1=0, initial_q2=90, initial_q3=-90)

# Define end effector open and closed angle
servoAngle_EE_closed = 10
servoAngle_EE_open = 180
q1=[]
q2=[]
q3=[]
# Compute inverse kinematics
for index,value in enumerate(z_coords2):
    qModel=myVirtualRobotArm.inverseKinematics(x_coords2[index], y_coords2[index], z_coords2[index])
    myVirtualRobotArm.updateJointAngles(qModel[0],qModel[1],qModel[2])
    q2.append(myVirtualRobotArm.map_kinematicsToServoAngles())
for index,value in enumerate(z_coords1):
    qModel=myVirtualRobotArm.inverseKinematics(x_coords1[index], y_coords1[index], z_coords1[index])
    myVirtualRobotArm.updateJointAngles(qModel[0],qModel[1],qModel[2])
    q1.append(myVirtualRobotArm.map_kinematicsToServoAngles())
for index,value in enumerate(z_coords3):
    qModel=myVirtualRobotArm.inverseKinematics(x_coords3[index], y_coords3[index], z_coords3[index])
    myVirtualRobotArm.updateJointAngles(qModel[0],qModel[1],qModel[2])
    q3.append(myVirtualRobotArm.map_kinematicsToServoAngles())
# print(q)
# ser1 = serial.Serial('COM6', baudrate=9600, timeout=1)
# time.sleep(3)
# print(len(q))
# wiadomoscLen = '<' + 'L,' + str(len(q)) + '>'
myArduino = arduinoController(port="COM6")
myArduino.openSerialPort()
servoAngle_EE_open = 110
servoAngle_EE_closed = 50
# ser1.write(bytes(wiadomoscLen, 'utf-8'))
for index in range(0,len(q1)):
    # wiadomoscFB = '<'+'F,' + str(int(q[index][1]))+'>'
    # wiadomoscUD = '<' + 'U,' + str(int(q[index][2])) + '>'
    # wiadomoscRo = '<' + 'R,' + str(int(q[index][0])) + '>'
    # wiadomoscGr = '<' + 'G,' + str(int(servoAngle_EE_open)) + '>'
    # # print(type(wiadomosc))
    # # print(wiadomosc)
    # ser1.write(bytes(wiadomoscFB, 'utf-8'))
    # ser1.write(bytes(wiadomoscUD, 'utf-8'))
    # ser1.write(bytes(wiadomoscRo, 'utf-8'))
    # ser1.write(bytes(wiadomoscGr, 'utf-8'))

    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=q1[index][0],
                                                        servoAngle_q2=q1[index][1],
                                                        servoAngle_q3=q1[index][2],
                                                        servoAngle_EE=servoAngle_EE_open))
time.sleep(0.5)

for index in range(0,len(q2)):
    # wiadomoscFB = '<'+'F,' + str(int(q[index][1]))+'>'
    # wiadomoscUD = '<' + 'U,' + str(int(q[index][2])) + '>'
    # wiadomoscRo = '<' + 'R,' + str(int(q[index][0])) + '>'
    # wiadomoscGr = '<' + 'G,' + str(int(servoAngle_EE_open)) + '>'
    # # print(type(wiadomosc))
    # # print(wiadomosc)
    # ser1.write(bytes(wiadomoscFB, 'utf-8'))
    # ser1.write(bytes(wiadomoscUD, 'utf-8'))
    # ser1.write(bytes(wiadomoscRo, 'utf-8'))
    # ser1.write(bytes(wiadomoscGr, 'utf-8'))

    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=q2[index][0],
                                                        servoAngle_q2=q2[index][1],
                                                        servoAngle_q3=q2[index][2],
                                                        servoAngle_EE=servoAngle_EE_closed))
time.sleep(0.5)
for index in range(0,len(q3)):
    # wiadomoscFB = '<'+'F,' + str(int(q[index][1]))+'>'
    # wiadomoscUD = '<' + 'U,' + str(int(q[index][2])) + '>'
    # wiadomoscRo = '<' + 'R,' + str(int(q[index][0])) + '>'
    # wiadomoscGr = '<' + 'G,' + str(int(servoAngle_EE_open)) + '>'
    # # print(type(wiadomosc))
    # # print(wiadomosc)
    # ser1.write(bytes(wiadomoscFB, 'utf-8'))
    # ser1.write(bytes(wiadomoscUD, 'utf-8'))
    # ser1.write(bytes(wiadomoscRo, 'utf-8'))
    # ser1.write(bytes(wiadomoscGr, 'utf-8'))

    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=q3[index][0],
                                                        servoAngle_q2=q3[index][1],
                                                        servoAngle_q3=q3[index][2],
                                                        servoAngle_EE=servoAngle_EE_open))


