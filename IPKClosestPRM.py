# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import IPPRMBase
from IPPerfMonitor import IPPerfMonitor
import networkx as nx

# reduce coding effort by using function provided by scipy
from scipy.spatial.distance import euclidean


class KClosestPRM(IPPRMBase.PRMBase):

    def __init__(self, _collChecker):
        super(KClosestPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()

    
    @IPPerfMonitor
    def _inSameConnectedComponent(self, node1, node2):
        """ Check whether to nodes are part of the same connected component using
            functionality from NetworkX
        """
        for connectedComponent in nx.connected_components(self.graph):
            if (node1 in connectedComponent) & (node2 in connectedComponent):
                return True

        return False

    
    @IPPerfMonitor
    def _nearestNeighbours(self, pos, k):
        """ Brute Force method to find the **k** closest nodes of a 
        graph to the given position **pos** """

        result = list()
        distances = list()
        for node in self.graph.nodes(data=True):
            distances.append((node, euclidean(node[1]['pos'],pos)))

        distances.sort(key=lambda x:x[1])

        if distances[0][1] == 0:
            del(distances[0])

        for i, node in enumerate(distances):
            result.append(node[0])
            if k == i+1:
                break

        return result
    
    @IPPerfMonitor
    def _learnRoadmapNearestNeighbour(self, k, numNodes):
        """ Generate a roadmap by given number of nodes and amount k, that should be tested for connection."""
        # nodeID is used for uniquely enumerating all nodes and as their name
        nodeID = 1
        while nodeID <= numNodes:
        
            # Generate a 'randomly chosen, free configuration'
            newNodePos = self._getRandomFreePosition()
            self.graph.add_node(nodeID, pos=newNodePos)
            
            # Find set of candidates to connect to sorted by distance
            result = self._nearestNeighbours(newNodePos, k)

            # for all nearest neighbours check whether a connection is possible
            for data in result:
                if self._inSameConnectedComponent(nodeID,data[0]):
                    break

                
                if not self._collisionChecker.lineInCollision(newNodePos,data[1]['pos']):
                    self.graph.add_edge(nodeID,data[0])
            
            nodeID += 1
            
    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        """
        
        Args:
            start (array): start position in planning space
            goal (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["k"]   = 5
            config["numNodes"] = 300
            config["useKDTree"] = True
            
            startList = [[1,1]]
            goalList  = [[10,1]]
            
            instance.planPath(startList,goalList,config)
        
        """
        # 0. reset
        self.graph.clear()
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)
        
        # 2. learn Roadmap
        self._learnRoadmapNearestNeighbour(config["k"],config["numNodes"])

        # 3. find connection of start and goal to roadmap
        # find nearest, collision-free connection between node on graph and start
        result = self._nearestNeighbours(checkedStartList[0], config["k"])
        for node in result:
            if not self._collisionChecker.lineInCollision(checkedStartList[0],node[1]['pos']):
                 self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                 self.graph.add_edge("start", node[0])
                 break

        result = self._nearestNeighbours(checkedGoalList[0], config["k"])
        for node in result:
            if not self._collisionChecker.lineInCollision(checkedGoalList[0],node[1]['pos']):
                 self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
                 self.graph.add_edge("goal", node[0])
                 break

        try:
            path = nx.shortest_path(self.graph,"start","goal")
        except:
            return []
        return path