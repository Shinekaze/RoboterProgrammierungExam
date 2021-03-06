import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import time

class IPSmoothing:
    """
    Smoothing algorithm and helper functions

    :param graph: Stores the generated graph from result
    :param solution: Stores the original solution from result, which is generated by the planner
    :param collision_checker: Stores information about obstables allowing for collision checks
    :param benchmark: Stores the environment with obstacles
    :param planner: Stores information about the original planning technique
    :param plannerFactoryName: The name of the technique used to plan the original graph and path
    :param temp_planner: Stores a copy of the original graph and path, unedited, for use in iterative graphs during debug
    :param size_history: A list storing the number of nodes in the path for each iteration
    :param length_history: A list storing the overall length of the path for each iteration
    :param debug: Boolean, enables or disables the printed output of debug information and iterative plots of each
    smoothing step. (default False)
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
        self.debug = False

    def smooth_solution(self, k_max, eps, variance_steps, min_variance, debug=False):
        """
        Smooths the path solution over a range of iterations, breaking if the range count is exceeded, or if the path
        variance is smaller than a minimum value.

        Selects a random point (P) on the path, then attempts to smooth the path by connecting the points (P-k_max) and
        (P+k_max) directly, if successful, all points between are deleted. By failure, k is de-incremented and the new
        points checked for connection. After repeated failure, and k=1, then delTree is called. Returns the smoothed
        path as an NetworkX Graph object.

        :param k_max: k value for selecting the predecessor and follower points to be joined
        :param eps: epsilon value for input into delTree
        :param variance_steps: window width for the rolling variance, how many previous iterations should be considered
        :param min_variance: minimum value for the variance, used to end the process early when path change is minimal
        :param debug: Boolean check to enable/disable printed information. (default False)
        :return: smooth_graph -- NetworkX Graph object containing the improved path
        """

        start_time = time.time()

        self.debug = debug

        if self.solution == []:
            return None

        pos = nx.get_node_attributes(self.graph,'pos')
        smooth_graph = nx.Graph(nx.subgraph(self.graph, self.solution))
        path = nx.shortest_path(smooth_graph,"start","goal")

        self.length_history.append(self.get_path_length(smooth_graph, path))
        self.size_history.append(len(path))
        if debug:
            tx = 0

        for n in range(50):
            rolling_var = pd.DataFrame(self.length_history).rolling(window=10).var()
            if debug:
                print(f"Length of History: {len(self.length_history)}")
                print(rolling_var)

            if len(self.length_history) > variance_steps and rolling_var.iloc[-1:].to_numpy() < min_variance:
                print(f"Variance: {rolling_var.iloc[-1:].to_numpy()}")
                print("Breaking due to small variance")
                break

            if debug:
                xx = 0

            i = random.randint(1, len(path)-2)
            break_loop = False

            for k in range(k_max, 0, -1):
                if break_loop:
                    if debug:
                        print("Line Connected, Break_Loop triggered")
                    break

                iscolliding = False

                if debug:
                    tx += 1
                    xx += 1
                    print(f"total steps: {tx}")
                    print(f"n step: {xx}")
                    print(f"n: {n}")
                    print(f"k: {k}")
                    print(self.plannerFactoryName)

                if i-k <= 0:
                    k_prev_node = "start"
                else:
                    k_prev_node = path[i-k]

                if i+k >= len(path)-1:
                    k_next_node = "goal"
                else:
                    k_next_node = path[i+k]

                if debug:
                    print(f"Initial Path: {path}")
                    print(f"i: {i}")
                    print(f"length of path: {len(path)}")
                    print(f"k_prev: {k_prev_node}")
                    print(f"Centered: {path[i]}")
                    print(f"k_next: {k_next_node}")

                if self.collision_checker.lineInCollision(pos[k_prev_node], pos[k_next_node]):
                    if debug:
                        print("Line collides, No change")

                    iscolliding = True

                else:
                    smooth_graph.add_edge(k_prev_node,k_next_node)

                    between_nodes = path[path.index(k_prev_node)+1:path.index(k_next_node)]

                    for node in between_nodes:
                        smooth_graph.remove_node(node)

                    path = nx.shortest_path(smooth_graph,"start","goal")

                    if debug:
                        print("new edge (", k_prev_node,"-", k_next_node, ") k =",k, " remove:", between_nodes)
                        print(f'New path: {path}')
                        self.visualize_path(self.temp_planner, smooth_graph)

                    break_loop = True

                self.length_history.append(self.get_path_length(smooth_graph, path))
                self.size_history.append(len(path))

                if k == 1 and iscolliding and not break_loop:
                    smooth_graph = self.del_tree(smooth_graph, path, eps, i)

                if debug:
                    #  Allows for iterative visualization
                    self.visualize_path(self.temp_planner, smooth_graph)

                pos = nx.get_node_attributes(smooth_graph,'pos')
                path = nx.shortest_path(smooth_graph,"start","goal")

        end_time = time.time()
        IPSmoothing.statistics.append({"benchmark_name": self.benchmark.name,
                                        "planner_name": self.plannerFactoryName,
                                        "original_length" : self.get_path_length(self.graph, self.solution),
                                        "original_size" : len(self.solution),
                                        "smoothed_length" : self.get_path_length(smooth_graph, path),
                                        "smoothed_size" : len(path),
                                        "min_length": self.get_path_length(smooth_graph, path), #self.get_path_length(smooth_graph, ["start", "goal"]),
                                        "length_history": self.length_history,
                                        "size_history": self.size_history,
                                        "time": end_time-start_time})

        return smooth_graph

    def del_tree(self, graph, path, eps, center_index):
        """
        Smooths the path when pre-existing points cannot be directly joined. Generates new midpoints along the paths
        between points (P) and (P-1) or (P+1). Attempts to join the midpoints directly and delete (P). If this fails,
        then a new attempt is made with a new set of midpoints, closer to (P). Repeats until successful, or until the
        distance between (P) and the new midpoints is smaller than a user-defined value, eps. Returns a NetworkX Graph
        object, graph.

        :param graph: A NetworkX Graph object to be smoothed
        :param path: NetworkX Union object containing the shortest path between Start and Goal
        :param eps: Shortest allowable distance between centerpoint (P) and the generated midpoint.
        :param center_index: Index of centerpoint (P)
        :return: graph -- NetworkX Graph object containing the improved path
        """
        DT_Flag = True
        t = 1

        # Gather the points
        k_prev_node = path[center_index-1]
        center_node = path[center_index]
        k_next_node = path[center_index+1]

        if self.debug:
            print(f"DelTree centered on list item {center_index}, node: {path[center_index]}")

        pA = np.array(graph.nodes[k_prev_node]['pos'])
        pB = np.array(graph.nodes[center_node]['pos'])
        pC = np.array(graph.nodes[k_next_node]['pos'])

        # Calculate distance
        dAB = pA - pB
        dCB = pC - pB

        while DT_Flag:
            pD = pB + dAB/pow(2, t)  # Pz1 from slides
            pE = pB + dCB/pow(2, t)  # Pz2 from slides

            # if magnitude/2^t is smaller than eps, break loop
            if np.linalg.norm(dAB)/pow(2, t) < eps or np.linalg.norm(dCB)/pow(2, t) < eps:
                if self.debug:
                    print("DelTree failed, line value smaller than epsilon")

                DT_Flag = False

            # Test line connection between new points for collisions
            elif not self.collision_checker.lineInCollision(pD, pE):

                DT_Flag = False  # Breaks while loop

                highest_node_name = max([i for i in graph.nodes if isinstance(i, int)])
                if self.debug:
                    print(f"graph nodes: {graph.nodes}")
                    print(highest_node_name)

                new_id1 = highest_node_name+1
                new_id2 = highest_node_name+2

                graph.add_node(new_id1, pos=pD.tolist()) #  Add new Nodes
                graph.add_node(new_id2, pos=pE.tolist())

                graph.add_edge(k_prev_node, new_id1) #  Add new Edges
                graph.add_edge(new_id1, new_id2)
                graph.add_edge(new_id2, k_next_node)

                graph.remove_node(center_node) #  Delete corner node

                if self.debug:
                    print(f"Adding nodes: {new_id1} and {new_id2}")
                    print(f"deleting center node: {center_node}")
                    print("DelTree successful")
                    path = nx.shortest_path(graph,"start","goal")
                    print(f'del_tree path creation, new path: {path}')

            else:
                if self.debug:
                    print("DelTree line collides")

            path = nx.shortest_path(graph,"start","goal")
            self.length_history.append(self.get_path_length(graph, path))
            self.size_history.append(len(path))

            t += 1

        return graph
        
    def visualize_path(self, plannerFactory, smoothed_graph):
        """
        Draws smoothed path over original solution and scene. No return.

        :param plannerFactory: NetworkX object containing the scene to be drawn
        :param smoothed_graph: NetworkX Graph object containing the improved path
        :return: None
        """

        fig_local = plt.figure(figsize=(10, 10))
        ax = fig_local.add_subplot(1, 1, 1)
        title = self.plannerFactoryName + " - " + self.benchmark.name
        ax.set_title(title, color='w')

        # Draw scene with original solution
        plannerFactory[self.plannerFactoryName][2](self.planner, self.solution, ax=ax, nodeSize=100)

        pos = nx.get_node_attributes(smoothed_graph, 'pos')


        # draw nodes based on solution path
        nx.draw_networkx_nodes(smoothed_graph,
                               pos,
                               node_size=300,
                               node_color='purple',
                               ax=ax
                               )
            
        # draw edges based on solution path
        nx.draw_networkx_edges(smoothed_graph,
                               pos,
                               alpha=0.8,
                               edge_color='purple',
                               width=10,
                               ax=ax
                               )
        # plt.show()

    def draw_history(self):
        """
        Draws history graph for the last performed smoothing. No return.

        :return: None
        """

        #print(IPSmoothing.statistics[-1]["length_history"], IPSmoothing.statistics[-1]["planner_name"])
        x = range(len(IPSmoothing.statistics[-1]["length_history"]))
        y = [ length / IPSmoothing.statistics[-1]["min_length"] for length in IPSmoothing.statistics[-1]["length_history"]]

        fig = plt.figure()
        ax = fig.add_subplot()
        
        ax.plot(x, y, color='g') #, marker='o', linestyle='dashed')
        ax.set_ylabel(IPSmoothing.statistics[-1]["benchmark_name"] + " Relative path length", color="g")
        ax.set_title("History of smoothing algorithm", color='k')
        ax.set_xlabel("Number of collision checks")
        ax.set_xticks(np.arange(0, len(x), 20))

        ax2 = ax.twinx()
        ax2.plot(x, IPSmoothing.statistics[-1]["size_history"], color='purple') #, marker='o', linestyle='dashed')
        ax2.set_ylabel(IPSmoothing.statistics[-1]["benchmark_name"] + " Number of nodes", color="purple")

    def get_path_length(self, graph, solution):
        """
        Calculates the length of the path based on each node's position. Returns length.

        :param graph: NetworkX Graph object containing each node's position
        :param solution: the solution path to be checked
        :return: length -- the length of the solution path
        """

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
        Plots a bar graph for every benchmark with (smoothed) solution path per planner algorithm. No return.

        :return: None
        """

        for bench in benchList:
            data_list = [entry for entry in IPSmoothing.statistics if entry["benchmark_name"] == bench.name]

            #print(data_list)
            if data_list == []:
                continue
            
            #print([data["original_size"] for data in data_list])

            fig, ax = plt.subplots()
        
            width = 0.2
            
            ax.set_title("Solution path before and after smoothing", color='k')
            ax.bar(np.arange(len(data_list)), [data["smoothed_length"] / data["min_length"] for data in data_list],width, color="g", label="after")
            ax.bar(np.arange(len(data_list)), [data["original_length"] / data["min_length"] for data in data_list],width, color="None", edgecolor='darkgreen', hatch='//', label="before")
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

            ax.legend(loc='upper right')


    def draw_history_per_benchmark(benchList, num_coll_checks, combine_all):
        """
        Draws a history graph for all algorithms per benchmark. No return.

        :param num_coll_checks: number of collision checks per drawing
        :param combine_all: Boolean, True draws a single graph with all benchmarks
        :return: None
        """

        for bench in benchList:
            data_list = [entry for entry in IPSmoothing.statistics if (entry["benchmark_name"] == bench.name) or combine_all]
            
            # print(data_list)
            if data_list == []:
                continue

            length_means = []
            length_asymmetric_errors = [[], []]
            size_means = []
            size_asymmetric_errors = [[], []]

            for coll_check in range(num_coll_checks):
                length_per_algorithm = []
                size_per_algorithm = []

                for data in data_list:
                    if coll_check < len(data["length_history"]):
                        length_per_algorithm.append(data["length_history"][coll_check] / data["min_length"])
                        size_per_algorithm.append(data["size_history"][coll_check])
                    else:
                        length_per_algorithm.append(np.nan)
                        size_per_algorithm.append(np.nan)

                lenght_mean = np.mean(length_per_algorithm)
                length_means.append(lenght_mean)
                length_asymmetric_errors[0].append(lenght_mean - min(length_per_algorithm))
                length_asymmetric_errors[1].append(max(length_per_algorithm) - lenght_mean)
                
                size_mean = np.mean(size_per_algorithm)
                size_means.append(size_mean)
                size_asymmetric_errors[0].append(size_mean - min(size_per_algorithm))
                size_asymmetric_errors[1].append(max(size_per_algorithm) - size_mean)


            fig = plt.figure()
            ax = fig.add_subplot()
            
            x = np.arange(num_coll_checks)
            
            ax.set_ylabel("Relative path length", color="g")
            ax.set_xlabel("Number of collision checks")
            ax.set_xticks(np.arange(0, len(x), 20))

            ax2 = ax.twinx()
            ax2.set_ylabel("Number of nodes", color="purple")

            if combine_all:
                ax.set_title("History of smoothing for all planners and benchmarks", color='k')
                #ax.set_ylim([1,4])
                #ax2.set_ylim([5,15])
                ax2.plot(x+0.5, size_means, color='purple')
                ax.plot(x, length_means, color='g')

                break
            else:
                ax.set_title("'" + bench.name + "' History of smoothing for all planners", color='k')
                ax.errorbar(x, length_means, yerr=length_asymmetric_errors, color='g', elinewidth = 0.5)
                ax2.errorbar(x+0.5, size_means, yerr=size_asymmetric_errors, color='purple', elinewidth = 0.5)


    def draw_statistics_all_combined(self):
        """
        Draws a bar graph for every benchmark with (smoothed) solution path per planner algorithm. Returns none.
        :return: None
        """

        planner_name = IPSmoothing.statistics[0]["planner_name"]
        plot_data= {"mean_smoothed_length": [],
                    "error_smoothed_length": [[], []],
                    "mean_original_length": [],
                    "error_original_length": [[], []],
                    "mean_original_size": [],
                    "error_original_size": [[], []],
                    "mean_smoothed_size": [],
                    "error_smoothed_size": [[], []],
                    "mean_time": [],
                    "error_time": [[], []],
                    "planner_name": []}

        smoothed_length_per_algorithm = []
        original_length_per_algorithm = []
        smoothed_size_per_algorithm = []
        original_size_per_algorithm = []
        time_per_algorithm = []

        for entry in IPSmoothing.statistics:
            if entry["planner_name"] == planner_name:
                smoothed_length_per_algorithm.append(entry["smoothed_length"] / entry["min_length"])
                original_length_per_algorithm.append(entry["original_length"] / entry["min_length"])
                smoothed_size_per_algorithm.append(entry["smoothed_size"])
                original_size_per_algorithm.append(entry["original_size"])
                time_per_algorithm.append(entry["time"])
            else:
                plot_data["mean_smoothed_length"].append(np.mean(smoothed_length_per_algorithm))
                plot_data["error_smoothed_length"][0].append(np.mean(smoothed_length_per_algorithm) - min(smoothed_length_per_algorithm))
                plot_data["error_smoothed_length"][1].append(max(smoothed_length_per_algorithm) - np.mean(smoothed_length_per_algorithm))
                plot_data["mean_original_length"].append(np.mean(original_length_per_algorithm))
                plot_data["error_original_length"][0].append(np.mean(original_length_per_algorithm) - min(original_length_per_algorithm))
                plot_data["error_original_length"][1].append(max(original_length_per_algorithm) - np.mean(original_length_per_algorithm))
                plot_data["mean_original_size"].append(np.mean(original_size_per_algorithm))
                plot_data["error_original_size"][0].append(np.mean(original_size_per_algorithm) - min(original_size_per_algorithm))
                plot_data["error_original_size"][1].append(max(original_size_per_algorithm) - np.mean(original_size_per_algorithm))
                plot_data["mean_smoothed_size"].append(np.mean(smoothed_size_per_algorithm))
                plot_data["error_smoothed_size"][0].append(np.mean(smoothed_size_per_algorithm) - min(smoothed_size_per_algorithm))
                plot_data["error_smoothed_size"][1].append(max(smoothed_size_per_algorithm) - np.mean(smoothed_size_per_algorithm))
                plot_data["mean_time"].append(np.mean(time_per_algorithm))
                plot_data["error_time"][0].append(np.mean(time_per_algorithm) - min(time_per_algorithm))
                plot_data["error_time"][1].append(max(time_per_algorithm) - np.mean(time_per_algorithm))
                plot_data["planner_name"].append(entry["planner_name"])

                smoothed_length_per_algorithm = [entry["smoothed_length"] / entry["min_length"]]
                original_length_per_algorithm = [entry["original_length"] / entry["min_length"]]
                smoothed_size_per_algorithm = [entry["smoothed_size"]]
                original_size_per_algorithm = [entry["original_size"]]
                time_per_algorithm = [entry["time"]]

                planner_name = entry["planner_name"]

        fig, ax = plt.subplots()
    
        width = 0.2
        x = np.arange(len(plot_data["mean_smoothed_length"]))
        
        ax.set_title("Smoothing result for all benchmarks", color='k')
        ax.bar(x, plot_data["mean_smoothed_length"], yerr=plot_data["error_smoothed_length"], width=width, color="g", label="after")
        ax.bar(x, plot_data["mean_original_length"], width=width, color="None", edgecolor='darkgreen', hatch='//', label="before") #, yerr=plot_data["error_original_length"])
        ax.set_ylabel("Relative path length", color="g")
        ax.set_xticks(x + width/2)
        ax.set_xticklabels(plot_data["planner_name"])

        ax2 = ax.twinx()
        ax2.bar(x + width, plot_data["mean_smoothed_size"], yerr=plot_data["error_smoothed_size"], width=-width, color="purple")
        ax2.bar(x + width, plot_data["mean_original_size"], width=width, color="None", edgecolor='indigo', hatch='//') #, yerr=plot_data["error_original_size"])
        ax2.set_ylabel("Number of nodes", color="purple")

        ax3 = ax.twinx()
        ax3.bar(x + 2*width, plot_data["mean_time"], yerr=plot_data["error_time"], width=width, color="y")
        ax3.set_ylabel("Smoothing time",  color="y")
        ax3.spines['right'].set_position(('axes', 1.15))
        ax3.spines['right'].set_color("y")

        ax.legend(loc='upper right')