import matplotlib.pyplot as plt
import numpy
import random
import math
'''
________________________________________________________________________________________
Classes
'''
class Obstacle():
    def __init__(self, x:float, y:float, radius:float=.75) -> None:
        self.x = x
        self.y = y
        self.radius = radius
        
class Node():
    def __init__(self,x:float,y:float,parent_index:float,cost:float, Acost:float):
        self.x = x
        self.y = y
        self.parent_index = parent_index
        self.cost = cost
        self.Acost = Acost
#   For Djikstras I have only defined to different classes of objects: Obstacles and Nodes.
#For problem 3, we will use all of the values in the Node class and only the x,y
#coordinates of the Obstacles

'''
________________________________________________________________________________________
Functions
'''

def inside(self,obstacle,node) -> bool:
        dist_from = numpy.sqrt((node.x - obstacle.x)**2 + (node.y - obstacle.y)**2)
        robot_radius = 0.5
        if dist_from > obstacle.radius + robot_radius:
            return False
        return True

#   The Cost function will take the current node (Node_1) and the Node that is being
#checked (Node_2_x, Node_2_y) x and y values and compute the euclidian distance
#between the two. Then this euclidian distance is added to the exisitng cost of 
# the node that is being checked to get the cost to move to this node from the 
#current node.
def Cost(self,Node_1,Node_2_x, Node_2_y, Node_cost) -> float:
 euclidian = numpy.sqrt(((Node_2_x - Node_1.x)**2+(Node_2_y - Node_1.y)**2)) + Node_cost
 return euclidian

def ACost(self,Node_1,Node_2_x, Node_2_y, Node_cost, goal_x,goal_y) -> float:
 euclidian =numpy.sqrt(((Node_2_x - goal_x)**2+(Node_2_y - goal_y)**2)) +\
     numpy.sqrt(((Node_2_x - Node_1.x)**2+(Node_2_y - Node_1.y)**2)) + Node_cost
 return euclidian
#   The compute_index function will take in the current nodes x and y values, the boundary
#values for the x and y axis, and the grid spacing for the nodes to find the index
# value of the current node.
def compute_index(self,min_x:int, max_x:int, min_y:int, max_y:int, 
                  gs:int, x_curr:int, y_curr:int) ->int:
    index = ((x_curr - min_x)/gs) + ((y_curr - min_y)/gs*(max_x+gs-min_x)/gs)

    return index

#   Node_Search will take the current node we have visited, search all adjacent positions
#for potential nodes to visit, and if the adjacent positions are valid locations
#for a node, create the node and place it in our unvisited nodes dictionary. Also
# will update an already existing unvisited node if Node_Search finds a cheaper way to 
#arrive at that unvisited node.
def Node_Search(self,current_node,goal_x,goal_y,visited_nodes,unvisited_nodes,obstacle_list):
# The two for loops below define the search area for potential nodes.
    for j in self.domain_gs:
        for i in self.domain_gs:
           checking = Node(current_node.x + i, current_node.y + j,
                           current_node.parent_index, 
                           Cost(self,current_node, current_node.x + i, current_node.y + j, current_node.cost),\
                               ACost(self,current_node, current_node.x + i, current_node.y + j, current_node.cost,goal_x,goal_y)) 
           checking.Acost = checking.Acost
           checking_index = compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, checking.x, checking.y)
#   All the if statements below check if the current location being checked is a valid 
#location and if so, stores the lowest cost found for the current location as the node.
           if checking_index == checking.parent_index or checking.x < self.min_x or checking.x > self.max_x or checking.y < self.min_y  or checking.y > self.max_y or checking_index == 0:
             continue
           if checking_index in visited_nodes:
             continue
           if checking_index in obstacle_list:
                    continue
           if checking_index in unvisited_nodes:
               if (unvisited_nodes[checking_index].cost) > (checking.cost):
                unvisited_nodes[checking_index] = checking
                unvisited_nodes[checking_index].parent_index = compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, current_node.x, current_node.y)
               else:
                continue
           k = 0
           for obs in obstacle_list:
             
               if inside(self,obstacle_list[obs],checking):
                  k = 1
           if k == 1:
                   continue
           
           else:
               unvisited_nodes[checking_index] = checking
               unvisited_nodes[checking_index].parent_index = compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, current_node.x, current_node.y)

class AStar():
   
    def __init__(self,min_x,min_y,max_x,max_y,gs,domain_x,domain_y,domain_gs):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.gs = gs
        self.domain_x = domain_x
        self.domain_y = domain_y 
        self.domain_gs = domain_gs
    
    def main(self,goal_x,goal_y,start_x,start_y,Obstacle_x,Obstacle_y):
        obstacle_list = dict()
        my_obstacles = list()
        unvisited_nodes = dict()
        visited_nodes = dict()
        path_nodes = dict()
        #   Given obstacle list is described.

        c = 0
        while c < len(Obstacle_x):
            my_obstacles.append((Obstacle_x[c],Obstacle_y[c])) 
            c = c + 1
        for i in my_obstacles:
        #   Storing the given obstacles into the Obstacle class with the given diameter.
            obstacle = Obstacle(i[0],i[1])
            obstacle_list[compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, obstacle.x, obstacle.y)] = obstacle
        #   Creates the node at the origin point and places the node in the 
        #visited nodes dictionary.  
        visited_nodes[compute_index(self,0, 15, 0, 15, self.gs, start_x, start_y)] = Node(1, 1, -1, 0,0)
        current_node = visited_nodes[compute_index(self,0, 15, 0, 15, self.gs, start_x, start_y)]

        #   The While loop below will run the Node Search function, find a new node to
        #move to by searching for the node with the lowest cost inside the unvisited 
        #nodes dictionary, and moving that node from unvisited to the visited dictionary.
        while current_node.x != goal_x or current_node.y != goal_y:
            Node_Search(self,current_node,goal_x,goal_y,visited_nodes,unvisited_nodes,obstacle_list)
            current_node = unvisited_nodes[min(unvisited_nodes, key=lambda x:unvisited_nodes[x].Acost)]
            visited_nodes[compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y,self.gs, current_node.x, current_node.y)] = current_node
            unvisited_nodes.pop(compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, current_node.x, current_node.y))

        #   The optimal path is initialized below by finding the node for the goal and
        #documenting it under path nodes.
        path_nodes[compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, current_node.x, current_node.y)] = current_node
        trail_node = current_node
        visited_nodes.pop(compute_index(self,self.min_x, self.max_x,self.min_y, self.max_y, self.gs, trail_node.x, trail_node.y)) 

        #   The While loop below will travel to the parent node of each node from the
        #goal until it reaches the start point to find our desired path.
        while trail_node.parent_index != -1:
            trail_node = visited_nodes[trail_node.parent_index]
            path_nodes[compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, trail_node.x, trail_node.y)] = trail_node
            visited_nodes.pop(compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, trail_node.x, trail_node.y))    

        #   Code to plot our map and our optimal path. The figure will open on a new tab.
        # Pink = Unvisited Nodes
        # Orange = Obstacles
        # Blue = Visited Nodes
        # Red = Optimal Path
        # for j in domain_y:
        #     for i in domain_x:
        #         index = compute_index(min_x, max_x, min_y, max_y, gs, i, j)
        #         if index in unvisited_nodes:
        #             plt.text(i, j, str(index), color='magenta', fontsize=10)
        #         if index in obstacle_list:
        #             plt.text(i, j, str(index), color = 'orange', fontsize=10)
        #         if index in visited_nodes:
        #             plt.text(i, j, str(index), color='blue', fontsize=10)
        #         if index in path_nodes:
        #             plt.text(i, j, str(index), color='red', fontsize=10)

        # plt.axis([min_x, max_x + gs, min_y, max_y + gs])
        return path_nodes

def RRT_Brancher(self,step, visited_nodes, min_x, max_x, min_y, max_y, gs, i, obstacle_list):
    global check
    check = True
    x = random.randint(self.min_x, self.max_x)
    y = random.randint(self.min_y, self.max_y)
    for n in visited_nodes:
        visited_nodes[n].Acost = Cost(self,visited_nodes[n], x, y, 0)
    current_node = visited_nodes[min(
        visited_nodes, key=lambda x:visited_nodes[x].Acost)]
    xt = x - current_node.x
    yt = y - current_node.y
    theta = math.atan2(yt, xt)
    xs = step*math.cos(theta)
    ys = step*math.sin(theta)
    cost = current_node.cost + 0.5
    parent = (compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y,
              self.gs, current_node.x, current_node.y))
    new_node = Node(xs + current_node.x, ys + current_node.y, list(visited_nodes.keys())
                    [list(visited_nodes.values()).index(current_node)], cost, 0)
    for l in obstacle_list:
        distance = numpy.sqrt(
            ((new_node.x - obstacle_list[l].x)**2+(new_node.y - obstacle_list[l].y)**2))
        if (obstacle_list[l].radius) > distance:

            check = False
    if not check:
        return current_node
    else:
        visited_nodes[i] = new_node
        #plt.plot(new_node.x, new_node.y, 'bo')
        #plt.pause(0.05)
        #plt.show()

        return new_node
class RRT():
    def __init__(self,min_x,min_y,max_x,max_y,gs,domain_x,domain_y,domain_gs):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.gs = gs
        self.domain_x = domain_x
        self.domain_y = domain_y 
        self.domain_gs = domain_gs    
    def main(self,goal_x,goal_y,start_x,start_y,Obstacle_x,Obstacle_y):
        obstacle_list = dict()
        my_obstacles = list()
        unvisited_nodes = dict()
        visited_nodes = dict()
        path_nodes = dict()
        x_path = []
        y_path = []  
        step = 0.5
        visited_nodes[0] = Node(1,1,-1,0,0)
        goal = Node(7,13,0,0,0)
        #   Given obstacle list is described.

        c = 0
        while c < len(Obstacle_x):
            my_obstacles.append((Obstacle_x[c],Obstacle_y[c])) 
            c = c + 1
        for i in my_obstacles:
        #   Storing the given obstacles into the Obstacle class with the given diameter.
            obstacle = Obstacle(i[0],i[1])
            obstacle_list[compute_index(self,self.min_x, self.max_x, self.min_y, self.max_y, self.gs, obstacle.x, obstacle.y)] = obstacle
        new_node = visited_nodes[0]
        i = 1
        while Cost(self,new_node, goal_x, goal_y, 0) > 1:
            new_node = RRT_Brancher(self, step, visited_nodes, self.min_x, self.max_x, self.min_y, self.max_y, self.gs,i,obstacle_list)
            i = i + 1
            
        goal.cost = Cost(self,new_node,goal.x,goal.y,new_node.cost)
        goal.parent_index = list(visited_nodes.keys())[list(visited_nodes.values()).index(new_node)]
        path_nodes[0] = goal
        trail_node = goal
        x_path.append(goal.x)
        y_path.append(goal.y)
        k = 1
        while trail_node.parent_index != -1:
            trail_node = visited_nodes[trail_node.parent_index]
            path_nodes[k] = trail_node
            x_path.append(trail_node.x)
            y_path.append(trail_node.y)
            k = k + 1
        x = 1
        plt.axis([self.min_x, self.max_x + self.gs, self.min_y, self.max_y + self.gs])

        for o in obstacle_list:
            plt.plot(obstacle_list[o].x, obstacle_list[o].y,'ks', markersize = 15)
            plt.show
        for i in path_nodes:
            if x != len(path_nodes):
                plt.plot(x_path, y_path, 'ro-')
            x = x + 1
        #plt.show()
        return path_nodes