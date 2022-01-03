import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import math

class IPSmoothing:
    """
    Smoothing algorithm and helper functions
    """
    statistics = []

    def __init__(self, result):
        self.graph = result.planner.graph
        self.solution = result.solution
        self.collision_checker = result.planner._collisionChecker
        self.benchmark = result.benchmark
        self.planner = result.planner
        self.plannerFactoryName = result.plannerFactoryName

    def smooth_solution(self, k_max, eps, debug = False):
        """
        Returns smoothed graph containing only the solution node and edges
        """

        if self.solution == []:
            return None

        pos = nx.get_node_attributes(self.graph,'pos')
        smooth_graph = nx.Graph(nx.subgraph(self.graph, self.solution))
        path = nx.shortest_path(smooth_graph,"start","goal")

        length_history = [self.get_path_length(smooth_graph, path)]
        size_history = [len(path)]

        # TODO: Abbruchkriterium Varianz der letzten x versuche
        for n in range(20):

            for k in range(k_max, 0, -1):

                i = random.randint(1, len(path)-2)

                if i-k <= 0:
                    k_prev_node = "start"
                else:
                    k_prev_node = path[i-k]

                if i+k >= len(path)-2:
                    k_next_node = "goal"
                else:
                    k_next_node = path[i+k]

                if not self.collision_checker.lineInCollision(pos[k_prev_node],pos[k_next_node]):
                    smooth_graph.add_edge(k_prev_node,k_next_node)

                    between_nodes = path[path.index(k_prev_node)+1:path.index(k_next_node)]

                    for node in between_nodes:
                        smooth_graph.remove_node(node)

                    path = nx.shortest_path(smooth_graph,"start","goal")

                    length_history.append(self.get_path_length(smooth_graph, path))
                    size_history.append(len(path))
                    
                    if debug:
                        print(path)
                        print("new edge (", k_prev_node,"-", k_next_node, ") k =",k, " remove:", between_nodes)
                    
                    break
                elif k == 1:
                    smooth_graph = self.del_tree(smooth_graph, eps)

        IPSmoothing.statistics.append({"benchmark_name": self.benchmark.name,
                                        "planner_name": self.plannerFactoryName,
                                        "original_length" : self.get_path_length(self.graph, self.solution),
                                        "original_size" : len(self.solution),
                                        "smoothed_length" : self.get_path_length(smooth_graph, path),
                                        "smoothed_size" : len(path),
                                        "length_history": length_history,
                                        "size_history": size_history})

        return smooth_graph

    def del_tree(self, graph, eps):
        # print("del_tree")
        return graph
        
    def visualize_path(self, plannerFactory, smoothed_graph):
        """
        Draws smoothed path over original solution and scene
        """

        fig_local = plt.figure(figsize=(10,10))
        ax = fig_local.add_subplot(1,1,1)
        title = self.plannerFactoryName + " - " + self.benchmark.name
        ax.set_title(title, color='w')
        
        # Draw scene with original solution
        plannerFactory[self.plannerFactoryName][2](self.planner, self.solution, ax=ax, nodeSize=100)

        pos = nx.get_node_attributes(smoothed_graph,'pos')

        # draw nodes based on solution path
        nx.draw_networkx_nodes(smoothed_graph,pos,
                                node_size=300,
                                node_color='g',  ax = ax)
            
        # draw edges based on solution path
        nx.draw_networkx_edges(smoothed_graph,pos,alpha=0.8,edge_color='purple',width=10,  ax = ax)

    def draw_statistics(self):
        """
        Draw history graph for the last performed smoothing
        """

        #print(IPSmoothing.statistics[-1]["length_history"], IPSmoothing.statistics[-1]["planner_name"])
        x = range(len(IPSmoothing.statistics[-1]["length_history"]))

        fig = plt.figure()
        ax = fig.add_subplot()
        
        ax.plot(x, IPSmoothing.statistics[-1]["length_history"], color='g', marker='o', linestyle='dashed')
        ax.set_ylabel(IPSmoothing.statistics[-1]["benchmark_name"] + " Path length", color="g")
        ax.set_title("History of smoothing algorithm", color='w')
        ax.set_xlabel("Number of collision checks")
        ax.set_xticks(x)

        ax2 = ax.twinx()
        ax2.plot(x, IPSmoothing.statistics[-1]["size_history"], color='purple', marker='o', linestyle='dashed')
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
        
    def draw_comparison(benchList):
        """
        Bar plot for every benchmark with (smoothed) solution path per planner algorithm
        """

        for bench in benchList:
            data_list = [entry for entry in IPSmoothing.statistics if entry["benchmark_name"] == bench.name]
            #print(data_list)
            #print([data["original_size"] for data in data_list])

            fig, ax = plt.subplots()
        
            width = 0.15
            
            ax.set_title("Solution path before and after smoothing", color='w')
            ax.bar(np.arange(len(data_list))-width, [data["original_length"] for data in data_list],width, color="g")
            ax.bar(np.arange(len(data_list)), [data["smoothed_length"] for data in data_list],width, color="lightgreen",)
            ax.set_ylabel(bench.name + " Path length", color="g")
            ax.set_xticks(np.arange(len(data_list)) + width/2)
            ax.set_xticklabels([data["planner_name"] for data in data_list])

            ax2 = ax.twinx()
            ax2.bar(np.arange(len(data_list))+width, [data["original_size"] for data in data_list],width, color="purple")
            ax2.bar(np.arange(len(data_list))+2*width, [data["smoothed_size"] for data in data_list],-width, color="violet")
            ax2.set_ylabel(bench.name + " Number of nodes", color="purple")