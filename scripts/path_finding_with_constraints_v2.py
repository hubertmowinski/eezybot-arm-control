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
    while i <= len(siatka) - 1 and i <= a[-1] + 6:
        siatka[i] = siatka[a[-1]]
        i+=1
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
        elif np.all(check3 == searchval):
            siatka[p[0], p[1] + 1, p[2]] = 1  #
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
start = (10, 12, 5) #COLGATE
goal = (19, 7, 2)
# start = (14, 0, 3)
# goal = (14, 12, 3)
myVirtualRobotArm = EEZYbotARM_Mk1(
        initial_q1=0, initial_q2=90, initial_q3=-90)
przeszkoda = [[6, 0, 0], [6, 12, 0], [11, 0, 0], [11, 12, 0], [6, 0, 4], [6, 12, 4], [11, 0, 4], [11, 12, 4]]#COLGATE #zmienić wprowadzanie przeszkody na oś bazową robota
# przeszkoda = [[5, 6, 0], [5, 8, 0], [11, 6, 0], [11, 8, 0], [5, 6, 7], [5, 8, 7], [11, 6, 7], [11, 8, 7]]
gridPodstawowy = np.zeros((20,13,16))
gridPodstawowy = dodajPrzeszkode(gridPodstawowy, przeszkoda)
grid = np.copy(gridPodstawowy)
grid = dodajOgraniczeniaX(grid)
grid = dodajOgraniczeniaY(grid)
Ograniczenia = wyszukajSkrajne(przeszkoda[6],przeszkoda[7])
print(Ograniczenia)
# while(znaleziono == False):
sprawdzam = 0
petla = 0
route = astar(grid, start, goal,petla,sprawdzam)

route = route + [start]

route = route[::-1]


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

for punkt, wartosc in np.ndenumerate(gridPodstawowy):
 # print('wartosc'+ str(int(wartosc)) + 'punkt = ('+ str(int(punkt[0])) + ','+ str(int(punkt[1])) + ','+ str(punkt[2])+ ')')
 if wartosc == 1:
    ax.scatter3D(int(punkt[0]), int(punkt[1]), int(punkt[2]), marker="o", color="blue", s=100)

ax.plot3D(x_coords, y_coords, z_coords, color="pink")
print('x_coords: ' + str(x_coords) + ' y_cords:' + str(y_coords) + ' z_coords:' + str(z_coords))

plt.show()


y_coords -= 7


y_coords = y_coords * 10
x_coords = x_coords * 10
z_coords = z_coords * 10
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
    qModel=myVirtualRobotArm.inverseKinematics(x_coords[index], y_coords[index], z_coords[index])
    myVirtualRobotArm.updateJointAngles(qModel[0],qModel[1],qModel[2])
    q.append(myVirtualRobotArm.map_kinematicsToServoAngles())

# print(q)
# ser1 = serial.Serial('COM6', baudrate=9600, timeout=1)
# time.sleep(3)
# print(len(q))
# wiadomoscLen = '<' + 'L,' + str(len(q)) + '>'
myArduino = arduinoController(port="COM6")
myArduino.openSerialPort()
servoAngle_EE_open = 180
# ser1.write(bytes(wiadomoscLen, 'utf-8'))
for index in range(0,len(q)):
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

    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=q[index][0],
                                                        servoAngle_q2=q[index][1],
                                                        servoAngle_q3=q[index][2],
                                                        servoAngle_EE=servoAngle_EE_open))


