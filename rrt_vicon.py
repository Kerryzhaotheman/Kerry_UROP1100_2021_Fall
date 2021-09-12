'''
MIT License
Copyright (c) 2019 Fanjin Zeng(Original), 2021 Liangyawei Kuang, Xiaoqi Zhao
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.
'''

import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque

import PyKDL
import rospy
from geometry_msgs.msg import *
from vicon_bridge.msg import *
from socket import *
import math
import json
import time


class Line():
    ''' Define line '''
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn


def Intersection(line, center, radius):
    ''' Check line-sphere (circle) intersection '''
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True



def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))


def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex


def window(startpos, endpos):
    ''' Define search window - 2 times of start to end rectangle'''
    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    return winx, winy, width, height


def isInWindow(pos, winx, winy, width, height):
    ''' Restrict new vertex insides search window'''
    if winx < pos[0] < winx+width and \
        winy < pos[1] < winy+height:
        return True
    else:
        return False


class Graph:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPosition(self):
        rx = random()
        ry = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        return posx, posy


def RRT(startpos, endpos, obstacles, n_iter, radius, stepSize):
    ''' RRT algorithm '''
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        print(_)
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            print('success')
            break
    return G


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    ''' RRT star algorithm '''
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        print(_)
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles, radius):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
            except:
                G.distances[endidx] = G.distances[newidx]+dist

            G.success = True
            print('success')
            break
    return G



def dijkstra(G):
    '''
    Dijkstra algorithm for finding shortest path from start position to end.
    '''
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)



def plot(G, obstacles, radius, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''
    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='red')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    # plt.savefig('rrt_viz.png')
    plt.show()


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        # plot(G, obstacles, radius, path)
        return path


class ViconDecode:
    def __init__(self):
        # declare initial location
        self.start_point = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
        self.start_angle = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
        self.flag1 = True
        self.flag2 = True
        self.flag3 = True
        self.flag4 = True
        self.flag5 = True
        # current position
        self.current_position = []
        # get tag position
        self.vicon_pose1 = TransformStamped()
        self.vicon_pose2 = TransformStamped()
        self.vicon_pose3 = TransformStamped()
        self.vicon_pose4 = TransformStamped()
        self.vicon_pose5 = TransformStamped()
        # sub tag pose
        self.tagpose_sub1 = rospy.Subscriber("vicon/bot1/bot1",TransformStamped,self.viconCB1)
        self.tagpose_sub2 = rospy.Subscriber("vicon/bot2/bot2",TransformStamped,self.viconCB2)
        self.tagpose_sub3 = rospy.Subscriber("vicon/bot3/bot3",TransformStamped,self.viconCB3)
        self.tagpose_sub4 = rospy.Subscriber("vicon/obs1/obs1",TransformStamped,self.viconCB4)
        self.tagpose_sub5 = rospy.Subscriber("vicon/obs2/obs2",TransformStamped,self.viconCB5)
        # every robot's velocity
        self.vel = []
        # loop frequency
        rate = rospy.Rate(20)
        self.start_time = time.time()
        # loop
        #self.goal = [0.75,0.54]
        while not rospy.is_shutdown():
            for i in range(0,5):
                if i==0:
                    x = self.vicon_pose1.transform.translation.x
                    y = self.vicon_pose1.transform.translation.y
                    z = self.vicon_pose1.transform.translation.z
                    #print(self.vicon_pose1.transform.translation)
                    quat_x = self.vicon_pose1.transform.rotation.x
                    quat_y = self.vicon_pose1.transform.rotation.y
                    quat_z = self.vicon_pose1.transform.rotation.z
                    quat_w = self.vicon_pose1.transform.rotation.w
                    angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
                    current_0 = [x,y,z]
                    current_angle_0 = angle
                if i==1:
                    x = self.vicon_pose2.transform.translation.x
                    y = self.vicon_pose2.transform.translation.y
                    z = self.vicon_pose2.transform.translation.z
                    quat_x = self.vicon_pose2.transform.rotation.x
                    quat_y = self.vicon_pose2.transform.rotation.y
                    quat_z = self.vicon_pose2.transform.rotation.z
                    quat_w = self.vicon_pose2.transform.rotation.w
                    angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
                    current_1 = [x,y,z]
                    current_angle_1 = angle
                if i==2:
                    x = self.vicon_pose3.transform.translation.x
                    y = self.vicon_pose3.transform.translation.y
                    z = self.vicon_pose3.transform.translation.z
                    quat_x = self.vicon_pose3.transform.rotation.x
                    quat_y = self.vicon_pose3.transform.rotation.y
                    quat_z = self.vicon_pose3.transform.rotation.z
                    quat_w = self.vicon_pose3.transform.rotation.w
                    angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
                    current_2 = [x,y,z]
                    current_angle_2 = angle
                if i==3:
                    x = self.vicon_pose4.transform.translation.x
                    y = self.vicon_pose4.transform.translation.y
                    z = self.vicon_pose4.transform.translation.z
                    quat_x = self.vicon_pose4.transform.rotation.x
                    quat_y = self.vicon_pose4.transform.rotation.y
                    quat_z = self.vicon_pose4.transform.rotation.z
                    quat_w = self.vicon_pose4.transform.rotation.w
                    angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
                    current_3 = [x,y,z]
                    current_angle_3 = angle
                if i==4:
                    x = self.vicon_pose5.transform.translation.x
                    y = self.vicon_pose5.transform.translation.y
                    z = self.vicon_pose5.transform.translation.z
                    quat_x = self.vicon_pose5.transform.rotation.x
                    quat_y = self.vicon_pose5.transform.rotation.y
                    quat_z = self.vicon_pose5.transform.rotation.z
                    quat_w = self.vicon_pose5.transform.rotation.w
                    angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
                    current_4 = [x,y,z]
                    current_angle_4 = angle

            self.current_position = [current_0, current_1, current_2, current_3,
                                        current_4]
            self.current_angle = [current_angle_0, current_angle_1, current_angle_2,
                                        current_angle_3, current_angle_4]
            # print("Pos:")
            # print("bot1:", self.current_position[0])
            # print("bot2:", self.current_position[1])
            # print("bot3:", self.current_position[2])
            # print("Ang:")
            # print("bot1:", self.current_angle[0])
            # print("bot2:", self.current_angle[1])
            # print("bot3:", self.current_angle[2])
            # print("\n")
            self.goal = self.current_position[1]
            th = math.atan2((self.goal[1]-self.current_position[0][1]),(self.goal[0]-self.current_position[0][0]))
            #print(self.goal[1]-self.current_position[0][1])
            #print(self.goal[0]-self.current_position[0][0])
            th = (th/3.14159)*180
            # print("th",th)
            rate.sleep()
            self.vel = []

    def viconCB1(self, msg):
        # get tags position
        self.vicon_pose1 = msg
        if self.flag1:
            x = self.vicon_pose1.transform.translation.x
            y = self.vicon_pose1.transform.translation.y
            z = self.vicon_pose1.transform.translation.z
            quat_x = self.vicon_pose1.transform.rotation.x
            quat_y = self.vicon_pose1.transform.rotation.y
            quat_z = self.vicon_pose1.transform.rotation.z
            quat_w = self.vicon_pose1.transform.rotation.w
            angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
            start_0 = [x, y, z]
            angle_0 = angle
            self.start_point[0] = start_0
            self.start_angle[0] = angle_0
        self.flag1 = False

    def viconCB2(self, msg):
        # get tags position
        self.vicon_pose2 = msg
        if self.flag2:
            x = self.vicon_pose2.transform.translation.x
            y = self.vicon_pose2.transform.translation.y
            z = self.vicon_pose2.transform.translation.z
            quat_x = self.vicon_pose2.transform.rotation.x
            quat_y = self.vicon_pose2.transform.rotation.y
            quat_z = self.vicon_pose2.transform.rotation.z
            quat_w = self.vicon_pose2.transform.rotation.w
            angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
            start_1 = [x, y, z]
            angle_1 = angle
            self.start_point[1] = start_1
            self.start_angle[1] = angle_1
        self.flag2 = False

    def viconCB3(self,msg):
        # get tags position
        self.vicon_pose3 = msg
        if self.flag3:
            x = self.vicon_pose3.transform.translation.x
            y = self.vicon_pose3.transform.translation.y
            z = self.vicon_pose3.transform.translation.z
            quat_x = self.vicon_pose3.transform.rotation.x
            quat_y = self.vicon_pose3.transform.rotation.y
            quat_z = self.vicon_pose3.transform.rotation.z
            quat_w = self.vicon_pose3.transform.rotation.w
            angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
            start_2 = [x, y, z]
            angle_2 = angle
            self.start_point[2] = start_2
            self.start_angle[2] = angle_2
        self.flag3 = False

    def viconCB4(self, msg):
        # get tags position
        self.vicon_pose4 = msg
        if self.flag4:
            x = self.vicon_pose4.transform.translation.x
            y = self.vicon_pose4.transform.translation.y
            z = self.vicon_pose4.transform.translation.z
            quat_x = self.vicon_pose4.transform.rotation.x
            quat_y = self.vicon_pose4.transform.rotation.y
            quat_z = self.vicon_pose4.transform.rotation.z
            quat_w = self.vicon_pose4.transform.rotation.w
            angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
            start_3 = [x, y, z]
            angle_3 = angle
            self.start_point[3] = start_3
            self.start_angle[3] = angle_3
        self.flag4 = False

    def viconCB5(self,msg):
        # get tags position
        self.vicon_pose5 = msg
        if self.flag5:
            x = self.vicon_pose5.transform.translation.x
            y = self.vicon_pose5.transform.translation.y
            z = self.vicon_pose5.transform.translation.z
            quat_x = self.vicon_pose5.transform.rotation.x
            quat_y = self.vicon_pose5.transform.rotation.y
            quat_z = self.vicon_pose5.transform.rotation.z
            quat_w = self.vicon_pose5.transform.rotation.w
            angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
            start_4 = [x, y, z]
            angle_4 = angle
            self.start_point[4] = start_4
            self.start_angle[4] = angle_4
            print("start----", self.start_point, self.start_angle)
            self.d01 = self.start_point[1][0] - self.start_point[0][0]
            self.d02 = self.start_point[0][0] - self.start_point[2][0]
            print(self.d01, self.d02)
        self.flag4 = False

    def quat_to_euler_angle(self, x, y, z, w):
        t0 = -2.0 * (x * y - w * z)
        t1 = w * w + x * x - y * y - z * z
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (x * z + w * y)
        pitch_y = math.asin(t2)

        t3 = -2.0 * (y * z - w * x)
        t4 = w * w - x * x - y * y + z * z
        yaw_z = math.atan2(t3, t4)

        x = (roll_x/3.14159)*180 - 90
        if x <= -180:
            x = x + 360
        y = (pitch_y/3.14159)*180
        z = (yaw_z/3.14159)*180

        #if x < 0:
        #    x = x + 360

        theta = [x,y,z]

        return theta # in degree


if __name__ == '__main__':
    rospy.init_node("vicon_decode")
    env = ViconDecode()

    if time.time() - env.start_time <= 3:
        rospy.spin()
        # print(time.time() - env.start_time)

    else:
        startpos = (env.current_position[0][0], env.current_position[0][1]) # (0., 0.)
        endpos = (env.current_position[1][0], env.current_position[1][1])
        obstacles = [(env.current_position[2][0], env.current_position[2][1]),
                     (env.current_position[3][0], env.current_position[3][1]),
                     (env.current_position[4][0], env.current_position[4][1])]
        n_iter = 10000
        radius = 0.2
        stepSize = 0.02

        # G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
        G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)

        if G.success:
            path = dijkstra(G)
            print(path)
            plot(G, obstacles, radius, path)

        else:
            plot(G, obstacles, radius)
