#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <queue>
#include <stack>
#include <climits>

using namespace std;

#define MAX 100000

// Structure to represent a node and its distance from the source
struct NodeDistance {
    int node;
    int distance;

    NodeDistance(int n, int d) : node(n), distance(d) {}

    // Overload the comparison operator for the priority queue
    bool operator>(const NodeDistance& other) const {
        return distance > other.distance;
    }
};

// Function to perform Dijkstra's algorithm
vector<int> dijkstra(vector<vector<pair<int, int>>>& graph, int startNode) {
    int numNodes = graph.size();
    vector<int> distances(numNodes, INT_MAX); // Initialize distances to infinity
    priority_queue<NodeDistance, vector<NodeDistance>, greater<NodeDistance>> pq;

    distances[startNode] = 0;
    pq.push(NodeDistance(startNode, 0));

    while (!pq.empty()) {
        int currentNode = pq.top().node;
        int currentDistance = pq.top().distance;
        pq.pop();

        if (currentDistance > distances[currentNode]) {
            continue; // Skip this node if a shorter distance is already found
        }

        // Explore neighbors and update distances
        for (auto neighbor : graph[currentNode]) {
            int neighborNode = neighbor.first;
            int neighborDistance = neighbor.second;

            if (currentDistance + neighborDistance < distances[neighborNode]) {
                distances[neighborNode] = currentDistance + neighborDistance;
                pq.push(NodeDistance(neighborNode, distances[neighborNode]));
            }
        }
    }

    return distances;
}

int main() {
    int numNodes, numEdges;
    cout << "Enter the number of nodes and edges: ";
    cin >> numNodes >> numEdges;

    vector<vector<pair<int, int>>> graph(numNodes);

    cout << "Enter the edges and their weights (node1 node2 weight):" << endl;
    for (int i = 0; i < numEdges; ++i) {
        int node1, node2, weight;
        cin >> node1 >> node2 >> weight;
        graph[node1].emplace_back(node2, weight);
        graph[node2].emplace_back(node1, weight); // For undirected graphs
    }
    //note the use of member function emplace_back of std::vector over push_back due to greater efficiency

    int startNode;
    cout << "Enter the starting node for Dijkstra's algorithm: ";
    cin >> startNode;

    vector<int> shortestDistances = dijkstra(graph, startNode);

    cout << "Shortest distances from node " << startNode << " to other nodes:" << endl;
    for (int i = 0; i < numNodes; ++i) {
        cout << "Node " << i << ": " << shortestDistances[i] << endl;
    }

    return 0;
}
