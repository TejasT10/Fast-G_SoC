// graph_algorithms.cpp

#include "graph_algorithms.hpp"
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

const int INF = std::numeric_limits<int>::max();

Graph::Graph(int numNodes) : numNodes(numNodes), adjacencyMatrix(numNodes, std::vector<int>(numNodes, 0)) {}

void Graph::addEdge(int from, int to, int weight) {
    adjacencyMatrix[from][to] = weight;
}

std::vector<int> Graph::getNeighbors(int node) const {
    std::vector<int> neighbors;
    for (int i = 0; i < numNodes; ++i) {
        if (adjacencyMatrix[node][i] != 0) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

int Graph::getWeight(int from, int to) const {
    return adjacencyMatrix[from][to];
}

int Graph::getNumNodes() const {
    return numNodes;
}

std::vector<int> BFS(const Graph& graph, int startNode) {
    std::vector<int> result;
    std::vector<bool> visited(graph.getNumNodes(), false);
    std::queue<int> bfsQueue;
    bfsQueue.push(startNode);
    visited[startNode] = true;

    while (!bfsQueue.empty()) {
        int currentNode = bfsQueue.front();
        bfsQueue.pop();
        result.push_back(currentNode);

        for (int neighbor : graph.getNeighbors(currentNode)) {
            if (!visited[neighbor]) {
                bfsQueue.push(neighbor);
                visited[neighbor] = true;
            }
        }
    }

    return result;
}

void DFSUtil(const Graph& graph, int currentNode, std::vector<bool>& visited, std::vector<int>& result) {
    visited[currentNode] = true;
    result.push_back(currentNode);

    for (int neighbor : graph.getNeighbors(currentNode)) {
        if (!visited[neighbor]) {
            DFSUtil(graph, neighbor, visited, result);
        }
    }
}

std::vector<int> DFS(const Graph& graph, int startNode) {
    std::vector<int> result;
    std::vector<bool> visited(graph.getNumNodes(), false);

    DFSUtil(graph, startNode, visited, result);

    return result;
}

std::vector<int> dijkstra(const Graph& graph, int startNode) {
    int numNodes = graph.getNumNodes();
    std::vector<int> distances(numNodes, INF);
    std::vector<bool> visited(numNodes, false);

    distances[startNode] = 0;

    for (int i = 0; i < numNodes - 1; ++i) {
        int minDistance = INF;
        int minNode = -1;

        // Find the node with the minimum distance among the unvisited nodes
        for (int node = 0; node < numNodes; ++node) {
            if (!visited[node] && distances[node] < minDistance) {
                minDistance = distances[node];
                minNode = node;
            }
        }

        if (minNode == -1) {
            break; // All remaining nodes are unreachable
        }

        visited[minNode] = true;

        // Relaxation step - update distances through the current node
        for (int neighbor : graph.getNeighbors(minNode)) {
            int newDistance = distances[minNode] + graph.getWeight(minNode, neighbor);
            if (newDistance < distances[neighbor]) {
                distances[neighbor] = newDistance;
            }
        }
    }

    return distances;
}

bool findAugmentingPath(int u, const Graph& graph, std::vector<int>& match, std::vector<bool>& visited) {
    for (int v = 0; v < graph.getNumNodes(); ++v) {
        if (!visited[v] && graph.getWeight(u, v) > 0) {
            visited[v] = true;

            if (match[v] == -1 || findAugmentingPath(match[v], graph, match, visited)) {
                match[v] = u;
                return true;
            }
        }
    }

    return false;
}

int fordFulkerson(Graph& graph, int source, int sink) {
    int numNodes = graph.getNumNodes();
    std::vector<int> match(numNodes, -1);

    while (findAugmentingPath(source, graph, match, std::vector<bool>(numNodes, false))) {
        // Find the bottleneck capacity along the augmenting path
        int bottleneck = INF;
        for (int v = sink; v != source; v = match[v]) {
            int u = match[v];
            bottleneck = std::min(bottleneck, graph.getWeight(u, v));
        }

        // Update the flow along the augmenting path
        for (int v = sink; v != source; v = match[v]) {
            int u = match[v];
            graph.addEdge(u, v, graph.getWeight(u, v) - bottleneck);
            graph.addEdge(v, u, graph.getWeight(v, u) + bottleneck);
        }
    }

    int maxFlow = 0;
    for (int v = 0; v < numNodes; ++v) {
        maxFlow += graph.getWeight(source, v);
    }

    return maxFlow;
}

/* std::vector<int> hungarianAlgorithm(const Graph& graph) {
    I shall later append the Hungarian Algorithm for solving matching in bipartite graphs here later, if need be.
    At the moment the code is complicated enough to run all other algorithms and hence, for now at least
    the Hungarian Algorithm shall remain in a separate, independent file.
} */

int main() {
    int numNodes, numEdges;
    std::cout << "Enter the number of nodes and edges: ";
    std::cin >> numNodes >> numEdges;

    Graph graph(numNodes);

    std::cout << "Enter the edges and their weights (from to weight):" << std::endl;
    for (int i = 0; i < numEdges; ++i) {
        int from, to, weight;
        std::cin >> from >> to >> weight;
        graph.addEdge(from, to, weight);
    }

    int startNode;
    std::cout << "Enter the starting node for BFS: ";
    std::cin >> startNode;
    std::vector<int> bfsResult = BFS(graph, startNode);
    std::cout << "BFS traversal starting from node " << startNode << ": ";
    for (int node : bfsResult) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    std::cout << "Enter the starting node for DFS: ";
    std::cin >> startNode;
    std::vector<int> dfsResult = DFS(graph, startNode);
    std::cout << "DFS traversal starting from node " << startNode << ": ";
    for (int node : dfsResult) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    std::cout << "Enter the starting node for Dijkstra's algorithm: ";
    std::cin >> startNode;
    std::vector<int> shortestDistances = dijkstra(graph, startNode);
    std::cout << "Shortest distances from node " << startNode << " to other nodes:" << std::endl;
    for (int i = 0; i < numNodes; ++i) {
        std::cout << "Node " << i << ": " << shortestDistances[i] << std::endl;
    }

    int source, sink;
    std::cout << "Enter the source and sink nodes for Ford-Fulkerson: ";
    std::cin >> source >> sink;

    int maxFlow = fordFulkerson(graph, source, sink);
    std::cout << "Maximum flow from node " << source << " to node " << sink << ": " << maxFlow << std::endl;
}
