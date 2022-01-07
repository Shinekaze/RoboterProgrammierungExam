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
        for n in range(50): # Todo: numpy variance -> length_history[-variance_steps]
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
                               node_color='purple',
                               ax=ax
                               )

        # nx.draw_networkx_labels(smoothed_graph, pos, font_size=10, font_color='black') #===================================Remove
            
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
        ax.set_title("History of smoothing algorithm", color='k')
        ax.set_xlabel("Number of collision checks")
        ax.set_xticks(np.arange(0, len(x), 20))

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
        Draw history graph for all algorithms per benchmark
        Number of collision checks to draw
        if combine_all = true one single graph with all benchmarks is drawn
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


    def draw_statistics_all_combined():
        """
        Bar plot for every benchmark with (smoothed) solution path per planner algorithm
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

        
        #print(data_list)

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