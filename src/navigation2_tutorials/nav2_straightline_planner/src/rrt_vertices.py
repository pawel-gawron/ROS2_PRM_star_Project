import rclpy
import time
from mapr_6_student.grid_map import GridMap
import numpy as np
from random import randrange
import math as mt
import heapq
import cv2
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point

np.random.seed(444)


# class RRT(GridMap):
#     def __init__(self):
#         super(RRT, self).__init__()
#         self.step = 0.05

#     def check_if_valid(self, a, b):
#         """
#         Checks if the segment connecting a and b lies in the free space.
#         :param a: point in 2D
#         :param b: point in 2D
#         :return: boolean
#         """
#         # D = mt.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
#         # X = a[0] - b[0]
#         # for i in np.linspace(0.01, D, 100):
#         #     S = i
#         #     XS = (S*X)/D
#         #     Y = mt.sqrt(S**2 - XS**2)
#         #     if b[1] > a[1]:
#         #         Y = -Y
#         #     self.get_logger().info(str((b[0] + XS, b[1] + Y)))
            
#         #     if self.map[(int)(b[1] + Y), (int)(b[0] + XS)] >= 80:
#         #         return False

#         # in_free_space = True
#         # return in_free_space

#         x_div = np.linspace(a[0], b[0], 100)
#         y_div = np.linspace(a[1], b[1], 100)
#         in_free_space = True
#         for x,y in zip(x_div, y_div):
#             x = int(x/self.resolution)
#             y = int(y/self.resolution)
#             if self.map[y,x] >= 80:
#                 return False
#         return in_free_space
    

#     def random_point(self):
#         """
#         Draws random point in 2D
#         :return: point in 2D
#         """
#         x = np.random.uniform(0, self.width)
#         y = np.random.uniform(0, self.height)
#         return np.array([x, y])

#     def find_closest(self, pos):
#         """
#         Finds the closest vertex in the graph to the pos argument

#         :param pos: point id 2D
#         :return: vertex from graph in 2D closest to the pos
#         """
#         # tree = np.asarray(self.parent)
#         # idx = np.abs(mt.sqrt(tree[:][0]**2 + tree[:][1]**2) - mt.sqrt(pos[0]**2 + pos[1]**2)).argmin()
#         # closest = tree[idx]
#         distance_dict = {}
#         # self.get_logger().info("###############################")
#         for key in self.parent.keys():
#             distance = mt.sqrt((key[0] - pos[0])**2 +
#                             (key[1] - pos[1])**2)
#             distance_dict[key] = distance
#         closest = min(distance_dict, key=distance_dict.get)
#         return closest

#     def new_pt(self, pt, closest):
#         """
#         Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

#         :param pt: point in 2D
#         :param closest: vertex in the tree (point in 2D)
#         :return: point in 2D
#         """
#         # odleglosc a^2 + b^2
#         # self.get_logger().info(str(closest))
#         D = (mt.sqrt((pt[0] - closest[0])**2 + (pt[1] - closest[1])**2))
#         S = self.step
#         X = pt[0] - closest[0]
#         XS = (S*X)/D
#         Y = mt.sqrt(S**2 - XS**2)

#         if closest[1] > pt[1]:
#             Y = -Y

#         return [closest[0] + XS, closest[1] + Y]

#     def search(self):
#         """
#         RRT search algorithm for start point self.start and desired state self.end.
#         Saves the search tree in the self.parent dictionary, with key value pairs representing segments
#         (key is the child vertex, and value is its parent vertex).
#         Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
#         """
#         self.parent[self.start] = None
#         Finished = False

#         while not Finished:
#             point =self.random_point()

#             closest = self.find_closest(point)

#             new_point = self.new_pt(point, closest)
#             valid = self.check_if_valid(new_point, closest)
#             self.get_logger().info(str(new_point))

#             if valid:
#                 self.parent[(new_point[0], new_point[1])] = closest

#                 if(self.check_if_valid(new_point, self.end)):
#                     self.parent[self.end] = new_point
#                     Finished = True

#             self.publish_search()
#         path = []
#         path.append(self.end)
#         current = self.end

#         while current != self.start:
#             path.append(self.parent[current])
#             current = tuple(self.parent[current])
        
#         path.append(self.start)
#         self.publish_path(path)

#         # while not self.rp.is_shutdown():
#         #     rp.sleep(0.01)

class PRMStar(GridMap):
    def __init__(self):
        super(PRMStar, self).__init__()
        self.num_samples = 500
        self.connection_radius = 0.4
        self.num_neighbors = 5
        self.graph = {}
        self.path = []
        self.points = []
        self.parent = {}

    def heuristic_cost(self, point):
        return mt.sqrt((self.end[0] - point[0]) ** 2 + (self.end[1] - point[1]) ** 2)

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        x_div = np.linspace(a[0], b[0], 100)
        y_div = np.linspace(a[1], b[1], 100)
        in_free_space = True
        for x,y in zip(x_div, y_div):
            x = int(x/self.resolution)
            y = int(y/self.resolution)
            if self.map[y,x] >= 50:
                return False
        return in_free_space

    def random_point(self):
        """
        Draws random point in 2D
        :return: point in 2D
        """
        points = []
        points.append(self.start)
        points.append(self.end)
        for _ in range(self.num_samples):
            x = np.random.uniform(0, self.width)
            y = np.random.uniform(0, self.height)

            x_check = int(x/self.resolution)
            y_check = int(y/self.resolution)
            if self.map[y_check,x_check] <= 80:
                points.append((x, y))

        return points
    
    def find_neighbors(self, point):
        distances = []
        for neighbor in self.graph.keys():
            if neighbor != point:
                if neighbor in self.graph:
                    if self.check_if_valid(point, neighbor):
                        dist = mt.sqrt((point[0] - neighbor[0]) ** 2 + (point[1] - neighbor[1]) ** 2)
                        if dist <= self.connection_radius:
                            distances.append((neighbor, dist))
        distances.sort(key=lambda x: x[1])
        return distances[:self.num_neighbors]
    
    def graph_create(self):
        self.points = self.random_point()
        # print(points)

        for point in self.points:
            key = tuple(point)
            self.graph[key] = []
            self.parent[key] = []

        for point in self.points:
            key = tuple(point) 
            self.graph[key] = self.find_neighbors(point)
            # print(point[0])
            # print(point[1])

        # self.graph[tuple(self.end)] = [] 
        # self.path = [self.start] 

    def search(self):
        self.graph_create()
        # print(self.graph)

        start_dist = {self.start: 0}
        queue = [(self.heuristic_cost(self.start), self.start)]
        visited = set()

        print("self.start: ", self.start)
        print("self.end: ", self.end)
        print("self.width: ", self.width)
        print("self.height: ", self.height)
        print("len(): ", len(self.graph))

        # print("self.graph: ", self.graph)
        # print("self.graph[self.end]: ", self.graph[self.end])
        # print("self.graph[self.end]: ", self.graph[(2.562340030717397, 1.3001026391748907)])

        while queue:
            _, current = heapq.heappop(queue)
            # print("current: ", current)

            if current == self.end:
                print("Finish")
                self.construct_path()
                break

            if current in visited:
                continue

            visited.add(current)
            # print("visited: ", visited)

            # print("self.graph[current]: ", self.graph[current])

            neighbors = self.graph[current]
            prev_neighbor = None
            prev_distance = float('inf')
            # print("neighbors: ", neighbors)
            for neighbor, distance in neighbors:
                new_distance = start_dist[current] + distance
                if neighbor not in start_dist or (prev_neighbor is not None and new_distance + self.heuristic_cost(neighbor) < prev_distance + self.heuristic_cost(prev_neighbor)):
                    self.parent[neighbor] = tuple(current)
                    start_dist[neighbor] = new_distance
                    prev_neighbor = neighbor
                    prev_distance = new_distance

                heapq.heappush(queue, (new_distance + self.heuristic_cost(neighbor), neighbor))
                # if neighbor not in self.parent.keys():
                #     self.parent[neighbor] = current
                #     temp_distance = new_distance
                    # print("test1: ", self.parent[neighbor])
                # elif new_distance + self.heuristic_cost(neighbor) < distance + self.heuristic_cost(self.parent[neighbor]):
                #     self.parent[neighbor] = current
                #     print("test2:", self.parent[neighbor])

                # print(self.parent.items())

                if current in self.graph:
                    del self.graph[current]
                    for node_neighbors in self.graph.values():
                        node_neighbors[:] = [(n, d) for n, d in node_neighbors if n != current]
                
            self.publish_search()
                # print(self.parent)

    def construct_path(self):
        path = []
        path.append(self.end)
        current = self.end

        while current != self.start:
            path.append(self.parent[current])
            current = tuple(self.parent[current])

        path.append(self.start)
        self.publish_path(path)

        # path = []
        # path.append(self.start)
        # current = self.end

        # while current != self.end:
        #     current = self.parent[current]
        #     path.append(current)

        # self.path = path[::-1]  # Odwróć kolejność, aby uzyskać ścieżkę od self.start do self.end
        # print("Path:", self.path)
        # self.publish_path(self.path)

        # current = self.end
        # self.path.append(current)
        # # neighbors = self.graph[current]
        # # del self.graph[current]
        # print("len(): ", len(self.graph))

        # while current != self.start:
        #     neighbors = self.graph[current]
        #     # print("neighbors: ", neighbors)
        #     if not neighbors:
        #         print("No valid neighbors for current node:", current)
        #         break
        #     next_node, _ = min(neighbors, key=lambda x: self.heuristic_cost(x[0]) + x[1])
        #     # print(f"Current: {current}, Next Node: {next_node}")
        #     self.path.append(next_node)

        #     if next_node == self.end:
        #         break
            
        #     # print("self.graph[self.end]: ", self.graph[(2.562340030717397, 1.3001026391748907)])
        #     del self.graph[current]
        #     for node_neighbors in self.graph.values():
        #         node_neighbors[:] = [neighbor for neighbor in node_neighbors if neighbor[0] != current]
        #     # print("self.graph[self.end]: ", self.graph[(2.562340030717397, 1.3001026391748907)])
        #     current = next_node
        #     # break

        # # self.path.reverse()
        # print("Path:", self.path)
        # self.publish_path(self.path)



def main(args=None):
    rclpy.init(args=args)
    rrt = PRMStar()
    while not rrt.data_received():
        rrt.get_logger().info("Waiting for data...")
        rclpy.spin_once(rrt)
        time.sleep(0.5)

    rrt.get_logger().info("Start graph searching!")
    time.sleep(1)
    rrt.search()


if __name__ == '__main__':
    main()