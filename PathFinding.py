import math
import numpy as np
from Utils import *
import matplotlib.pyplot as plt

class Node:
    def __init__(self, coords, map2d, parent=None):
        self.coords = coords
        self.parent = parent
        self.map2D = map2d
        if parent:
            self.h = Utils.euclidean_distance(coords, self.map2D.end_quantified)
            self.g = Utils.euclidean_distance(coords, parent.coords) + parent.g
            self.f = self.h + self.g  
        else:
            self.h = Utils.euclidean_distance(coords, self.map2D.end_quantified)
            self.g = 0
            self.f = self.h
            
    def __hash__(self):
        return hash(tuple(self.coords))
    
    def __str__(self):
        return "Node " + str(tuple(self.coords)) + ", parent: " + str(self.parent.coords) + ", f: " + str(self.f)
    
    #def __repr__(self):
    #    return self.__str__()
    
class PathFinding:
    def __init__(self, map2d):
        self.map2D = map2d
        self.open_hashmap = dict()
        start_node = Node(self.map2D.start_quantified, self.map2D)
        self.open_hashmap[tuple(start_node.coords)] = start_node
        
        self.closed_set = set()
        self.current_node = None
        self.drone_radius = 2
    
    def is_colliding(self, coords):
        #check if drone is in bounds
        if coords[0] - self.drone_radius >= 0 and coords[0] + self.drone_radius < self.map2D.grid.shape[0]:
            if coords[1] - self.drone_radius >= 0 and coords[1] + self.drone_radius < self.map2D.grid.shape[1]:
                # check if drone is colliding an obstacle
                collision_map = self.map2D.grid[coords[0] - self.drone_radius:coords[0] + self.drone_radius+ 1,
                    coords[1] - self.drone_radius:coords[1] + self.drone_radius+ 1]
                if not np.any(collision_map):
                    return False

                return True
    
    def is_shorter_path_to_node(self, node):
        if tuple(node.coords) in self.open_hashmap:
            if self.open_hashmap[tuple(node.coords)].g > node.g:
                #print("Replace ", self.open_hashmap[tuple(node.coords)], " with ", node)
                self.open_hashmap[tuple(node.coords)] = node
                return True
            
        return False

    def check_neighbors(self, node):
        # compute all f-cost from all 8-neighbors


        neighbors_offset = np.array([[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        for neigh_offset in neighbors_offset:
            neigh_coords = node.coords + neigh_offset

            if(not self.is_colliding(neigh_coords)):
                neigh_node = Node(neigh_coords, self.map2D, self.current_node)
                
                
                if neigh_node not in self.closed_set:
                    if tuple(neigh_node.coords) not in self.open_hashmap or self.is_shorter_path_to_node(neigh_node):
                        
                        if neigh_node not in self.open_hashmap:
                            self.open_hashmap[tuple(neigh_coords)] = neigh_node
                            #print("Add: ", neigh_node)
            
            

    def astar(self):

        print("Computing path...")
        while self.open_hashmap:
            self.current_node = min(self.open_hashmap.values(),key=lambda k: k.f)
            
            del self.open_hashmap[tuple(self.current_node.coords)]
            self.closed_set.add(self.current_node)

            if np.array_equal(self.current_node.coords, self.map2D.end_quantified):
                print("Found path.")
                break
            
            self.check_neighbors(self.current_node)


        backtrack = []
        node = self.current_node
        
        while node:
            backtrack.append(node.coords)
            node = node.parent
 
        path = np.stack(backtrack)
        return path

    def visualize_path(self):
        np.add.at(self.map2D.grid, tuple(zip(*self.astar())), 1)

        plt.figure(figsize = (20,15))
        plt.imshow(self.map2D.grid)