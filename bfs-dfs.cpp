#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <stack>
#include <queue>

using namespace std;

#define MAX 100000

/* BFS Algorithm:
    1. Enqueue the starting node (or nodes) into a queue.
    2. While the queue is not empty:
        Dequeue a node from the queue.
        Process the node (e.g., visit it or perform some operations).
        Enqueue all its unvisited neighbors into the queue.
    */

// Function to perform BFS traversal on a graph
void BFS(vector<vector<int>>& graph, int startNode) {
    int numNodes = graph.size();
    vector<bool> visited(numNodes, false);

    queue<int> bfsQueue;
    bfsQueue.push(startNode);
    visited[startNode] = true;

    cout << "BFS traversal starting from node " << startNode << ": ";
    while (!bfsQueue.empty()) {
        int currentNode = bfsQueue.front();
        bfsQueue.pop();
        cout << currentNode << " "; // Print the node

        for (int neighbor : graph[currentNode]) {
            if (!visited[neighbor]) {
                bfsQueue.push(neighbor);
                visited[neighbor] = true;
            }
        }
    }

    cout << endl;
}

/*  DFS Algorithm:
    1. Push the starting node (or nodes) onto a stack.
    2. While the stack is not empty:
        Pop a node from the stack.
        Process the node (e.g., visit it or perform some operations).
        Push all its unvisited neighbors onto the stack.
    */

// Function to perform DFS traversal on a graph
void DFSUtil(vector<vector<int>>& graph, int currentNode, vector<bool>& visited) {
    visited[currentNode] = true;
    cout << currentNode << " "; // Print the node

    for (int neighbor : graph[currentNode]) {
        if (!visited[neighbor]) {
            DFSUtil(graph, neighbor, visited);
        }
    }
}

void DFS(vector<vector<int>>& graph, int startNode) {
    int numNodes = graph.size();
    vector<bool> visited(numNodes, false);

    cout << "DFS traversal starting from node " << startNode << ": ";
    DFSUtil(graph, startNode, visited);

    cout << endl;
}

int main() {
    int numNodes, numEdges;
    cout << "Enter the number of nodes and edges: ";
    cin >> numNodes >> numEdges;

    vector<vector<int>> graph(numNodes);

    cout << "Enter the edges (node1 node2):" << endl;
    for (int i = 0; i < numEdges; ++i) {
        int node1, node2;
        cin >> node1 >> node2;
        graph[node1].push_back(node2);
        graph[node2].push_back(node1);
        //graphs are undirected
    }

    int startNode;
    cout << "Enter the starting node for BFS: ";
    cin >> startNode;

    BFS(graph, startNode);

    cout << "Enter the starting node for DFS: ";
    cin >> startNode;

    DFS(graph, startNode);
}