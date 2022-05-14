# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import math
from numpy import random
from scipy import spatial


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        dist = math.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
        dist = abs(dist)
        return dist

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        i = 0.05
        while i < 1:
            row = node1.row * i + (node2.row * (1 - i))
            col = node1.col * i + (node2.col * (1 - i))
            i = i + 0.05
            if self.map_array[int(row)][int(col)] == 1:
                continue
            else:
                return False
        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        row = random.randint(0, self.size_row - 1)
        col = random.randint(0, self.size_col - 1)
        i = random.choice([True, False], p=[goal_bias, 1-goal_bias])
        if i:
            point = (self.goal.row, self.goal.col)
        else:
            point = (row, col)
        return point

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        nearest_node = None
        min_dist = math.sqrt(self.size_row**2 + self.size_col**2)
        for i in self.vertices:
            dist = self.dis(Node(point[0], point[1]), i)
            if dist < min_dist:
                nearest_node = i
                min_dist = dist
            else:
                continue

        return nearest_node


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbours = []

        for i in self.vertices:
            dist = self.dis(new_node, i)
            if i == new_node:
                continue
            elif dist <= neighbor_size:
                neighbours.append(i)
            else:
                continue
        return neighbours


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        for i in neighbors:
            cost = i.cost + self.dis(new_node, i)
            if cost < new_node.cost and self.check_collision(new_node, i):
                new_node.parent = i
                new_node.cost = cost

        for i in neighbors:
            cost = new_node.cost + self.dis(new_node, i)
            if cost < i.cost and self.check_collision(new_node, i):
                i.parent = new_node
                i.cost = cost
                # print("rewired")

    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                # print(cur.row, cur.col)
                # print(cur.parent.row, cur.parent.col)
                # print("------------------")
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')


        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()
        ### YOUR CODE HERE ###
        i = 0
        while i <= n_pts:
            i = i + 1
            # if i == n_pts:
            #     point = (self.goal.row, self.goal.col)
            #     # print("here")
            # else:
            point = self.get_new_point(0.1)
            nearest_node = self.get_nearest_node(point)
            r = 10
            s = math.sqrt((point[0]-nearest_node.row)**2 + (point[1]-nearest_node.col)**2)
            if s == 0:
                row = nearest_node.row
            else:
                temp = (point[0]-nearest_node.row)/s
                row1 = nearest_node.row + r*temp
                row2 = nearest_node.row - r*temp

                if (nearest_node.row - 1 < row1 < point[0] + 1) or (nearest_node.row + 1 > row1 > point[0] - 1):
                    row = row1
                elif (nearest_node.row - 1 < row2 < point[0] + 1) or (nearest_node.row + 1 > row2 > point[0] - 1):
                    row = row2
                else:
                    row = point[0]

            if s == 0:
                col = nearest_node.col
            else:
                temp = (point[1] - nearest_node.col) / s
                col1 = nearest_node.col + r * temp
                col2 = nearest_node.col - r * temp

                if (nearest_node.col - 1 < col1 < point[1] + 1) or (nearest_node.col + 1 > col1 > point[1] - 1):
                    col = col1
                elif (nearest_node.col - 1 < col2 < point[1] + 1) or (nearest_node.col + 1 > col2 > point[1] - 1):
                    col = col2
                else:
                    col = point[1]

                # col = (((point[1]-nearest_node.col)/(point[0]-nearest_node.row))*(row - nearest_node.row)) + nearest_node.col

            new_node = Node(int(row), int(col))
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.dis(new_node, nearest_node)
            # print(self.dis(new_node, nearest_node))

            if self.check_collision(new_node, nearest_node):
                if (new_node.row, new_node.col) == (self.goal.row, self.goal.col):
                    self.goal.parent = new_node.parent
                    self.goal.cost = new_node.cost
                    self.vertices.append(self.goal)
                    self.found = True

                    break
                else:
                    self.vertices.append(new_node)
                # print(new_node.parent.col)
            else:
                continue



        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        i = 0
        while i <= n_pts:
            i = i + 1
            goal_bias = 0.1
            # if self.found:
            #     goal_bias = 0
            point = self.get_new_point(goal_bias)
            nearest_node = self.get_nearest_node(point)
            r = 10
            s = math.sqrt((point[0] - nearest_node.row) ** 2 + (point[1] - nearest_node.col) ** 2)
            if s == 0:
                row = nearest_node.row
            else:
                temp = (point[0] - nearest_node.row) / s
                row1 = nearest_node.row + r * temp
                row2 = nearest_node.row - r * temp

                if (nearest_node.row - 1 < row1 < point[0] + 1) or (nearest_node.row + 1 > row1 > point[0] - 1):
                    row = row1
                elif (nearest_node.row - 1 < row2 < point[0] + 1) or (nearest_node.row + 1 > row2 > point[0] - 1):
                    row = row2
                else:
                    row = point[0]

            if s == 0:
                col = nearest_node.col
            else:
                temp = (point[1] - nearest_node.col) / s
                col1 = nearest_node.col + r * temp
                col2 = nearest_node.col - r * temp

                if (nearest_node.col - 1 < col1 < point[1] + 1) or (nearest_node.col + 1 > col1 > point[1] - 1):
                    col = col1
                elif (nearest_node.col - 1 < col2 < point[1] + 1) or (nearest_node.col + 1 > col2 > point[1] - 1):
                    col = col2
                else:
                    col = point[1]

            new_node = Node(int(row), int(col))
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.dis(new_node, nearest_node)
            # print(self.dis(new_node, nearest_node))

            if self.check_collision(new_node, nearest_node):
                neighbors = self.get_neighbors(new_node, 20)
                self.rewire(new_node, neighbors)
                if (new_node.row, new_node.col) == (self.goal.row, self.goal.col) and self.found == False:
                    self.goal.parent = new_node.parent
                    self.goal.cost = new_node.cost
                    self.vertices.append(self.goal)
                    self.found = True
                    # print(self.goal.parent.row, self.goal.parent.col)
                else:
                    self.vertices.append(new_node)


            else:
                continue

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
