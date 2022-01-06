import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import math
import time

class IPSmoothing:
    """
    Smoothing algorithm and helper functions
    """
    statistics = []

    def __init__(self, result, temporary_planner):
        self.graph = result.planner.graph
        self.solution = result.solution
        self.collision_checker = result.planner._collisionChecker
        self.benchmark = result.benchmark
        self.planner = result.planner
        self.plannerFactoryName = result.plannerFactoryName
        self.temp_planner = temporary_planner
        self.size_history = []
        self.length_history = []

    def smooth_solution(self, k_max, eps, variance_steps, debug = False):
        """
        Returns smoothed graph containing only the solution node and edges
        """
        start_time = time.time()

        if self.solution == []:
            return None

        pos = nx.get_node_attributes(self.graph,'pos')
        smooth_graph = nx.Graph(nx.subgraph(self.graph, self.solution))
        path = nx.shortest_path(smooth_graph,"start","goal")

        self.length_history.append(self.get_path_length(smooth_graph, path))
        self.size_history.append(len(path))
        tx = 0

        # TODO: Abbruchkriterium Varianz der letzten x versuche
        for n in range(20): # Todo: numpy variance -> length_history[-variance_steps]
            xx = 0
            i = random.randint(1, len(path)-2)
            break_loop = False

            for k in range(k_max, 0, -1):

                if break_loop:
                    # print("breaking!")
                    break
                iscolliding = False

                tx += 1
                xx += 1
                # print(f"total steps: {tx}")
                # print(f"n step: {xx}")
                # print(f"n: {n}")
                # print(f"k: {k}")
                # print(self.plannerFactoryName)

                if i-k <= 0:
                    k_prev_node = "start"
                else:
                    k_prev_node = path[i-k]

                if i+k >= len(path)-1:
                    k_next_node = "goal"
                else:
                    k_next_node = path[i+k]

                # print(f"Initial Path: {path}")
                # print(f"i: {i}")
                # print(f"length of path: {len(path)}")
                # print(f"k_prev: {k_prev_node}")
                # print(f"Centered: {path[i]}")
                # print(f"k_next: {k_next_node}")

                if self.collision_checker.lineInCollision(pos[k_prev_node],pos[k_next_node]):
                    # print("Line collides, No change")
                    iscolliding = True

                else:
                    smooth_graph.add_edge(k_prev_node,k_next_node)

                    between_nodes = path[path.index(k_prev_node)+1:path.index(k_next_node)]
                    # print(f"Deleted Nodes: {between_nodes}")

                    for node in between_nodes:
                        smooth_graph.remove_node(node)

                    path = nx.shortest_path(smooth_graph,"start","goal")
                    # print(f'new path creation, new path: {path}')

                    # self.length_history.append(self.get_path_length(smooth_graph, path))
                    # self.size_history.append(len(path))
                    
                    if debug:
                        print(path)
                        print("new edge (", k_prev_node,"-", k_next_node, ") k =",k, " remove:", between_nodes)

                    #  Allows for iterative visualization
                    # self.visualize_path(self.temp_planner, smooth_graph, tx) #=========================Remove

                    # break
                    break_loop = True

                self.length_history.append(self.get_path_length(smooth_graph, path))
                self.size_history.append(len(path))

                if k == 1 and iscolliding and not break_loop:
                    smooth_graph = self.del_tree(smooth_graph, path, eps, i)

                #  Allows for iterative visualization
                # self.visualize_path(self.temp_planner, smooth_graph, tx) #=========================Remove

                pos = nx.get_node_attributes(smooth_graph,'pos')
                path = nx.shortest_path(smooth_graph,"start","goal")

        end_time = time.time()
        IPSmoothing.statistics.append({"benchmark_name": self.benchmark.name,
                                        "planner_name": self.plannerFactoryName,
                                        "original_length" : self.get_path_length(self.graph, self.solution),
                                        "original_size" : len(self.solution),
                                        "smoothed_length" : self.get_path_length(smooth_graph, path),
                                        "smoothed_size" : len(path),
                                        "min_length": self.get_path_length(smooth_graph, ["start", "goal"]),
                                        "length_history": self.length_history,
                                        "size_history": self.size_history,
                                        "time": end_time-start_time})

        return smooth_graph

    def del_tree(self, graph, path, eps, center_index):
        # print("del_tree")
        DT_Flag = True
        t = 1
        # eps = 0.5
        # pos = nx.get_node_attributes(graph, 'pos')
        # print(pos)
        # print(path)

        # Gather the points
        # print(f"DelTree centered on list item {center_index}, node: {path[center_index]}")
        # print(f"Number of nodes in graph: {graph.number_of_nodes()}")
        # print(f"Number of edges in graph: {graph.number_of_edges()}")

        k_prev_node = path[center_index-1]
        center_node = path[center_index]
        k_next_node = path[center_index+1]

        pA = np.array(graph.nodes[k_prev_node]['pos'])
        pB = np.array(graph.nodes[center_node]['pos'])
        pC = np.array(graph.nodes[k_next_node]['pos'])

        # Calculate distance
        dAB = pA - pB
        dCB = pC - pB

        while DT_Flag:
            pD = pB + dAB/pow(2, t)  # Pz1 from slides
            pE = pB + dCB/pow(2, t)  # Pz2 from slides

            # Check for line collision
            if np.linalg.norm(dAB)/pow(2, t) < eps or np.linalg.norm(dCB)/pow(2, t) < eps:
                # print("DelTree failed, line value smaller than epsilon")
                # if magnitude/2^t is smaller than eps, break loop
                DT_Flag = False

            elif not self.collision_checker.lineInCollision(pD, pE):

                DT_Flag = False  # Breaks while loop

                # print(f"graph nodes: {graph.nodes}")
                test = max([i for i in graph.nodes if isinstance(i, int)])

                # print(test)

                new_id1 = test+1
                new_id2 = test+2

                graph.add_node(new_id1, pos=pD.tolist())
                graph.add_node(new_id2, pos=pE.tolist())

                graph.add_edge(k_prev_node, new_id1)
                graph.add_edge(new_id1, new_id2)
                graph.add_edge(new_id2, k_next_node)

                graph.remove_node(center_node)
                # print(f"Adding nodes: {new_id1} and {new_id2}")
                # print(f"deleting center node: {center_node}")

                # pos = nx.get_node_attributes(graph, 'pos')
                # print(pos)

                # self.path_arr.insert(self.k_next, pE)  # Inserts Pz2
                # del self.path_arr[self.start_point]  # Deletes corner point
                # self.path_arr.insert(self.start_point, pD)  #Inserts Pz1

                # print("DelTree successful")

                path = nx.shortest_path(graph,"start","goal") #====================Needed?
                # print(f'del_tree path creation, new path: {path}')

            else:
                # print("DelTree line collides")
                pass

            self.length_history.append(self.get_path_length(graph, path))
            self.size_history.append(len(path))

            t += 1

        return graph
        
    def visualize_path(self, plannerFactory, smoothed_graph, X):
        """
        Draws smoothed path over original solution and scene
        """

        fig_local = plt.figure(figsize=(10, 10))
        ax = fig_local.add_subplot(1, 1, 1)
        title = self.plannerFactoryName + " - " + self.benchmark.name
        ax.set_title(title, color='w')

        plt.title(X)

        # Draw scene with original solution
        plannerFactory[self.plannerFactoryName][2](self.planner, self.solution, ax=ax, nodeSize=100)

        pos = nx.get_node_attributes(smoothed_graph, 'pos')


        # draw nodes based on solution path
        nx.draw_networkx_nodes(smoothed_graph,
                               pos,
                               node_size=300,
                               node_color='orange',
                               ax=ax
                               )

        nx.draw_networkx_labels(smoothed_graph, pos, font_size=10, font_color='black') #===================================Remove
            
        # draw edges based on solution path
        nx.draw_networkx_edges(smoothed_graph,
                               pos,
                               alpha=0.8,
                               edge_color='purple',
                               width=10,
                               ax=ax
                               )
        plt.show()

    def draw_history(self):
        """
        Draw history graph for the last performed smoothing
        """

        #print(IPSmoothing.statistics[-1]["length_history"], IPSmoothing.statistics[-1]["planner_name"])
        x = range(len(IPSmoothing.statistics[-1]["length_history"]))
        y = [ length / IPSmoothing.statistics[-1]["min_length"] for length in IPSmoothing.statistics[-1]["length_history"]]

        fig = plt.figure()
        ax = fig.add_subplot()
        
        ax.plot(x, y, color='g') #, marker='o', linestyle='dashed')
        ax.set_ylabel(IPSmoothing.statistics[-1]["benchmark_name"] + " Relative path length", color="g")
        ax.set_title("History of smoothing algorithm", color='w')
        ax.set_xlabel("Number of collision checks")
        ax.set_xticks(np.arange(0, len(x), 10))

        ax2 = ax.twinx()
        ax2.plot(x, IPSmoothing.statistics[-1]["size_history"], color='purple') #, marker='o', linestyle='dashed')
        ax2.set_ylabel(IPSmoothing.statistics[-1]["benchmark_name"] + " Number of nodes", color="purple")

    def get_path_length(self, graph, solution):
        pos = nx.get_node_attributes(graph,'pos')

        prev_node = None
        length = 0

        for node in solution:
            if prev_node is not None:
                length += self.distance(pos[node], pos[prev_node])

            prev_node = node

        return length

    def distance(self, point_1, point_2):
        distance = math.sqrt(math.pow(point_1[0]-point_2[0], 2) + math.pow(point_1[1]-point_2[1], 2))
        # print(point_1, point_2, distance)
        return distance
        
    def draw_statistics(benchList):
        """
        Bar plot for every benchmark with (smoothed) solution path per planner algorithm
        """

        for bench in benchList:
            data_list = [entry for entry in IPSmoothing.statistics if entry["benchmark_name"] == bench.name]
            #print(data_list)
            #print([data["original_size"] for data in data_list])

            fig, ax = plt.subplots()
        
            width = 0.2
            
            ax.set_title("Solution path before and after smoothing", color='w')
            ax.bar(np.arange(len(data_list)), [data["smoothed_length"] / data["min_length"] for data in data_list],width, color="g")
            ax.bar(np.arange(len(data_list)), [data["original_length"] / data["min_length"] for data in data_list],width, color="None", edgecolor='darkgreen', hatch='//')
            ax.set_ylabel(bench.name + " Relative path length", color="g")
            ax.set_xticks(np.arange(len(data_list)) + width/2)
            ax.set_xticklabels([data["planner_name"] for data in data_list])

            ax2 = ax.twinx()
            ax2.bar(np.arange(len(data_list))+width, [data["smoothed_size"] for data in data_list],-width, color="purple")
            ax2.bar(np.arange(len(data_list))+width, [data["original_size"] for data in data_list],width, color="None", edgecolor='indigo', hatch='//')
            ax2.set_ylabel(bench.name + " Number of nodes", color="purple")

            ax3 = ax.twinx()
            ax3.bar(np.arange(len(data_list))+2*width, [data["time"] for data in data_list],width, color="y")
            ax3.set_ylabel(bench.name + " Smoothing time",  color="y")
            ax3.spines['right'].set_position(('axes', 1.15))
            ax3.spines['right'].set_color("y")

        IPSmoothing.statistics = []


    def draw_history_per_benchmark(benchList):
        pass

    def draw_history_all_combined(benchList):
        pass

    def draw_statistics_all_combined(benchList):
        pass
