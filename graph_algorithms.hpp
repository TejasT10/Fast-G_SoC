// graph_algorithms.hpp

#ifndef GRAPH_ALGORITHMS_HPP
#define GRAPH_ALGORITHMS_HPP

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

#define MAX 100000

class Graph {
public:
    Graph(int numNodes);
    void addEdge(int from, int to, int weight);
    std::vector<int> getNeighbors(int node) const;
    int getWeight(int from, int to) const;
    int getNumNodes() const;

private:
    int numNodes;
    std::vector<std::vector<int>> adjacencyMatrix;
};

std::vector<int> BFS(const Graph& graph, int startNode);
std::vector<int> DFS(const Graph& graph, int startNode);
std::vector<int> dijkstra(const Graph& graph, int startNode);
int fordFulkerson(Graph& graph, int source, int sink);
std::vector<int> hungarianAlgorithm(const Graph& graph);

#endif // GRAPH_ALGORITHMS_HPP
