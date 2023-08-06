#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>

using namespace std;

const int INF = numeric_limits<int>::max();

// Function to find an augmenting path using DFS
bool findAugmentingPath(int u, const vector<vector<int>>& graph, vector<int>& match, vector<bool>& visited) {
    for (int v = 0; v < graph[u].size(); ++v) {
        if (graph[u][v] && !visited[v]) {
            visited[v] = true;

            if (match[v] == -1 || findAugmentingPath(match[v], graph, match, visited)) {
                match[v] = u;
                return true;
            }
        }
    }

    return false;
}

// Function to find the minimum-weight perfect matching using the Hungarian algorithm
vector<int> hungarianAlgorithm(const vector<vector<int>>& graph, int n) {
    vector<int> match(n, -1);

    for (int u = 0; u < n; ++u) {
        vector<bool> visited(n, false);
        findAugmentingPath(u, graph, match, visited);
    }

    return match;
}

int main() {
    int n;
    cout << "Enter the number of vertices on each side of the bipartite graph: ";
    cin >> n;

    // Assuming a square bipartite graph, where the weight of the edge between u and v is given as graph[u][v]
    vector<vector<int>> graph(n, vector<int>(n, 0));
    cout << "Enter the weights of the edges (u v weight):" << endl;

    int u, v, weight;
    while (cin >> u >> v >> weight) {
        graph[u][v] = weight;
    }

    vector<int> match = hungarianAlgorithm(graph, n);

    // Printing the minimum-weight perfect matching
    cout << "Minimum-weight perfect matching:" << endl;
    for (int v = 0; v < n; ++v) {
        if (match[v] != -1) {
            cout << "Edge (" << match[v] << ", " << v << ") with weight " << graph[match[v]][v] << endl;
        }
    }
}
