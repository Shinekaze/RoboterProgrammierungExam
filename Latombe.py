import numpy as np
import random
import networkx as nx
from IPEnvironment import CollisionChecker


class Smoothing:

# ======================================================================================================================

    def __init__(self, path_arr, obstacles, k_value=3):
        self.path_arr = path_arr
        self.k_value = k_value
        self.k_previous = 0
        self.k_next = 0
        self.G_graph = nx.Graph()
        self.world_collider = CollisionChecker(obstacles, limits=[[-6.0, 6.0], [-6.0, 6.0]])
        self.start_point = 0
        self.iteration = 0
        self.variance_list = []

# ======================================================================================================================

    def graph_builder(self):
        # Rebuild path array into a graph network

        for i in range(len(self.path_arr)):
            self.G_graph.add_node(i, pos=self.path_arr[i])

        for i in range(len(self.path_arr)-1):
            self.G_graph.add_edge(i, i+1)

# ======================================================================================================================

    def start_picker(self):
        # Choose a random node, but not one that's at the very ends of the path, ie not start or finish!
        self.start_point = random.randint(1, len(self.path_arr)-1)

# ======================================================================================================================

    def k_checker(self):
        if self.k_value >= 1:

            # Check the Lower K stays in range!
            if (self.start_point - self.k_value) < 0:
                self.k_previous = 0
            elif (self.start_point - self.k_value) >= 0:
                self.k_previous = (self.start_point - self.k_value)

            # Check the Upper K stays in range!
            if (self.start_point + self.k_value) < len(self.path_arr):
                self.k_next = (self.start_point + self.k_value)
            elif (self.start_point + self.k_value) >= len(self.path_arr):
                self.k_next = len(self.path_arr)-1

            return True

# ======================================================================================================================

    def DelTree(self):
        print("K is 1, attempting DelTree")
        DT_Flag = True
        t = 1
        eps = 0.5

        # Gather the points
        pA = np.array(self.path_arr[self.k_previous])
        pB = np.array(self.path_arr[self.start_point])
        pC = np.array(self.path_arr[self.k_next])

        # Calculate distance
        dAB = pA - pB
        dCB = pC - pB

        while DT_Flag:
            pD = pB + dAB/pow(2, t)  # Pz1 from slides
            pE = pB + dCB/pow(2, t)  # Pz2 from slides
            # Check for line collision
            line_check = self.world_collider.lineInCollision(pD, pE)

            if not line_check:
                print("DelTree successful")
                DT_Flag = False  # Breaks while loop
                pD = pD.tolist()
                pE = pE.tolist()
                self.path_arr.insert(self.k_next, pE)  # Inserts Pz2
                del self.path_arr[self.start_point]  # Deletes corner point
                self.path_arr.insert(self.start_point, pD)  #Inserts Pz1

            if np.linalg.norm(dAB)/pow(2, t) < eps or np.linalg.norm(dCB)/pow(2, t) < eps:
                print("Line value smaller than epsilon")
                # if magnitude/2^t is smaller than eps, break loop
                DT_Flag = False

            t += 1

        return True

# ======================================================================================================================

    def line_connector(self):
        point_a = self.path_arr[self.k_previous]
        print(self.start_point)
        print(len(self.path_arr))
        print(self.k_next)
        point_b = self.path_arr[self.k_next]
        line_check = self.world_collider.lineInCollision(point_a, point_b)

        if line_check:
            self.k_value -= 1
            print("Line cannot be joined: De-incrementing K")

            if self.k_value == 1:
                return self.DelTree()  # Returns True after successful run of DelTree

            return False

        else:

            self.G_graph.add_edge[self.k_previous, self.k_next]

            del self.path_arr[self.k_previous + 1:self.k_next - 1]  # deletes the points between

            print("Line successfully joined, extra points deleted. New path is: ")
            print(self.path_arr)
            return True

# ======================================================================================================================
    # Todo: implement Variance
    def variance_tracker(self):  # alternately --> iterator
        variance = 1
        self.variance_list.append(variance)
        if len(self.variance_list) > 5:  # set a fixed length of variance steps to track, remove oldest
            del self.variance_list[0]

# ======================================================================================================================

    def plotter(self):
        # Todo: implement plot output
        placeholder = 1

# ======================================================================================================================

    def smoother(self):
        path_flag = False
        point_flag = False
        while not path_flag:
            self.graph_builder()
            self.start_picker()
            while not point_flag:
                self.k_checker()
                # if self.k_value == 1:
                #     path_flag = self.DelTree()
                point_flag = self.line_connector()
            print(self.path_arr)
            x = self.variance_tracker()  # Stand-in pseudo-code currently

            if x < 1:
                print("Variance is too small, ending loop")
                # Todo: x and 1 are placeholders for the variance functionality
                path_flag = True

            self.iteration += 1

            if self.iteration >= 10:  #back-up loop-break functionality 50-100 iterations
                # Todo: Change iterations to 50
                print("Maximum iterations reached")
                path_flag = True


# ======================================================================================================================
'''End of File'''
# ======================================================================================================================
