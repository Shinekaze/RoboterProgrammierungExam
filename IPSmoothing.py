import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd

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
        self.debug = False

    def smooth_solution(self, k_max, eps, variance_steps, min_variance, debug=False):
        """
        Returns smoothed graph containing only the solution node and edges
        """

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

        for n in range(20):
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

        IPSmoothing.statistics.append({"benchmark_name": self.benchmark.name,
                                        "planner_name": self.plannerFactoryName,
                                        "original_length" : self.get_path_length(self.graph, self.solution),
                                        "original_size" : len(self.solution),
                                        "smoothed_length" : self.get_path_length(smooth_graph, path),
                                        "smoothed_size" : len(path),
                                        "length_history": self.length_history,
                                        "size_history": self.size_history})

        return smooth_graph

    def del_tree(self, graph, path, eps, center_index):
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
        Draws smoothed path over original solution and scene
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
        # plt.show()

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
#%%
