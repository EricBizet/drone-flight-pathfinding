import math
import numpy as np
from Utils import *
import matplotlib.pyplot as plt
import csv

class Node:
    """Represents node object in A* search algorithm"""

    def __init__(self, coords, map2d, parent=None):
        """Compute f,g and h cost of a node"""
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
        """Two points with same coords but different metadata (e.g. f-cost) will yield a same hash return
        
        Other data is not needed when storing already explored nodes
        """
        return hash(tuple(self.coords))
    
    def __str__(self):
        return "Node " + str(tuple(self.coords)) + ", parent: " + str(self.parent.coords) + ", f: " + str(self.f)
    
class PathFinding:
    def __init__(self, map2d):
        self.map2D = map2d

        self.open_hashmap = dict()
        #Initialize open hashmap
        start_node = Node(self.map2D.start_quantified, self.map2D)
        self.open_hashmap[tuple(start_node.coords)] = start_node
        
        self.closed_set = set()
        self.current_node = None

        # Drone radius for collisions
        self.drone_radius = 2 # 5x5 subdivision drone

        self.new_path = None
    
    def is_colliding(self, coords):
        """Check if drone is in bounds and if not colliding a wall
        
        Uses drone radius to compute collision, can be used as a safety margin
        """
        if tuple(coords) in self.closed_set:
            return True
        else:
            #Check if drone is in bounds
            if coords[0] - self.drone_radius >= 0 and coords[0] + self.drone_radius < self.map2D.grid.shape[0]:
                if coords[1] - self.drone_radius >= 0 and coords[1] + self.drone_radius < self.map2D.grid.shape[1]:
                    # Check if drone is colliding an obstacle. In this case: read a 5*5 view of the obstacle grid and check for obstacles
                    collision_map = self.map2D.grid[coords[0] - self.drone_radius:coords[0] + self.drone_radius+ 1,
                        coords[1] - self.drone_radius:coords[1] + self.drone_radius+ 1]
                    if not np.any(collision_map):
                        return False
        
        # If node is not traversable, add to closed set
        self.closed_set.add(tuple(coords))

        return True
    
    def is_shorter_path_to_node(self, node):
        """If a shorter path to a node exist, update his parent with new f and g costs"""
        if self.open_hashmap[tuple(node.coords)].g > node.g:
            #print("Replace ", self.open_hashmap[tuple(node.coords)], " with ", node)
            self.open_hashmap[tuple(node.coords)] = node
            return True
            
        return False

    def check_neighbors(self, node):
        """Explore all 8-neighbors from current node"""


        neighbors_offset = np.array([[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        for neigh_offset in neighbors_offset:
            neigh_coords = node.coords + neigh_offset

            neigh_node = Node(neigh_coords, self.map2D, self.current_node)

            if(not self.is_colliding(neigh_coords)):
                                    
                if tuple(neigh_node.coords) not in self.open_hashmap or self.is_shorter_path_to_node(neigh_node):
                    
                    self.open_hashmap[tuple(neigh_coords)] = neigh_node
                        #print("Add: ", neigh_node)
            
            

    def astar(self):
        """Main body of A* algorithm"""

        print("Computing path...")
        while self.open_hashmap:
            # Get node from the open hashmap with lowest f-cost
            self.current_node = min(self.open_hashmap.values(),key=lambda k: k.f)
            
            del self.open_hashmap[tuple(self.current_node.coords)]
            self.closed_set.add(tuple(self.current_node.coords))

            if np.array_equal(self.current_node.coords, self.map2D.end_quantified):
                print("Found path.")
                break
            
            self.check_neighbors(self.current_node)

        #Once path is found, backtrack from last node to first node using parents relationship
        backtrack = []
        node = self.current_node
        
        while node:
            backtrack.append(node.coords)
            node = node.parent
 
        path = np.stack(backtrack)
        return path

    def generate_path(self):
        self.new_path = self.astar()

    def visualize_path(self, title):
        """Use obstacle grid to visualize new path within it. Used for debugging"""
        grid_copy = self.map2D.grid.copy()

        np.add.at(grid_copy, tuple(zip(*self.new_path)), 1)

        
        fig = plt.figure(figsize = (20,15))
        fig.canvas.set_window_title(title)
        plt.imshow(grid_copy)

    def write_path_csv(self, path):
        """Write new generated path to csv file"""
        csv_path = self.new_path.copy().astype("float32")
        csv_path = np.flip(csv_path, 0)
        csv_path[:, 0] = csv_path[:, 0] * (self.map2D.X_lidar_size / self.map2D.X_res) + self.map2D.X_lidar_offset
        csv_path[:, 1] = csv_path[:, 1] * (self.map2D.Y_lidar_size / self.map2D.Y_res) + self.map2D.Y_lidar_offset

        with open(path, 'w') as f:

            writer = csv.writer(f)
    
            for i, point in enumerate(csv_path):
                writer.writerow((i, 1))
                writer.writerow(point)
   