#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <queue>
#include <stack>
#include <climits>
#include <algorithm>

using namespace std;

#define MAX 1000000

// Class representing a directed graph
class Graph {
private:
    int numNodes;
    vector<vector<int>> capacity;

public:
    Graph(int n) : numNodes(n), capacity(n, vector<int>(n, 0)) {}

    void addEdge(int from, int to, int capacity) {
        this->capacity[from][to] = capacity;
    }

    // Ford-Fulkerson algorithm to find the maximum flow
    int fordFulkerson(int source, int sink) {
        vector<vector<int>> residualCapacity = capacity;
        vector<int> parent(numNodes, -1);
        int maxFlow = 0;

        while (bfs(source, sink, residualCapacity, parent)) {
            int pathFlow = INT_MAX;
            for (int node = sink; node != source; node = parent[node]) {
                int prevNode = parent[node];
                pathFlow = min(pathFlow, residualCapacity[prevNode][node]);
            }

            for (int node = sink; node != source; node = parent[node]) {
                int prevNode = parent[node];
                residualCapacity[prevNode][node] -= pathFlow;
                residualCapacity[node][prevNode] += pathFlow;
            }

            maxFlow += pathFlow;
        }

        return maxFlow;
    }

private:
    // BFS to find augmenting paths
    bool bfs(int source, int sink, vector<vector<int>>& residualCapacity, vector<int>& parent) {
        vector<bool> visited(numNodes, false);
        queue<int> q;
        q.push(source);
        visited[source] = true;
        parent[source] = -1;

        while (!q.empty()) {
            int currentNode = q.front();
            q.pop();

            for (int nextNode = 0; nextNode < numNodes; ++nextNode) {
                if (!visited[nextNode] && residualCapacity[currentNode][nextNode] > 0) {
                    q.push(nextNode);
                    parent[nextNode] = currentNode;
                    visited[nextNode] = true;
                }
            }
        }

        return visited[sink];
    }
};

int main() {
    int numNodes, numEdges;
    cout << "Enter the number of nodes and edges: ";
    cin >> numNodes >> numEdges;

    Graph graph(numNodes);

    cout << "Enter the edges and their capacities (from to capacity):" << endl;
    for (int i = 0; i < numEdges; ++i) {
        int from, to, capacity;
        cin >> from >> to >> capacity;
        graph.addEdge(from, to, capacity);
    }

    int source, sink;
    cout << "Enter the source and sink nodes: ";
    cin >> source >> sink;

    int maxFlow = graph.fordFulkerson(source, sink);

    cout << "Maximum flow from node " << source << " to node " << sink << ": " << maxFlow << endl;
}