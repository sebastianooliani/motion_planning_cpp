#include <iostream>
#include <vector>
#include <algorithm>

class DepthFirstSearch {
    public:
        DepthFirstSearch() {
            std::cout << "DepthFirstSearch constructor called." << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph) {
            std::cout << "Performing depth-first search..." << std::endl;
            // Implementation of depth-first search algorithm goes here.
            std::cout << "Searching from node " << startNode
                    << " to node " << goalNode << std::endl;

            std::vector<int> queue;
            queue.push_back(startNode);
            std::vector<int> visited;
            visited.push_back(startNode);

            int iteration = 0;
            while(queue.size() > 0) {
                iteration++;
                int current_node = queue.back();

                std::cout << "Current node: " << current_node << std::endl;

                queue.erase(queue.end()-1); // or queue.pop_back()
                if(current_node == goalNode){
                    std::cout << "Goal node found: " << goalNode << std::endl;
                    std::cout << "Total iterations: " << iteration << std::endl;
                    return;
                }
                
                std::vector<int> next_nodes = graph[current_node];
                int len = next_nodes.size();

                for(int i = 0; i < len; i++) {
                    int next_node = next_nodes[i];

                    bool found = (std::find(visited.begin(), visited.end(), next_node) != visited.end());

                    if(!found) {
                        queue.push_back(next_node);
                        visited.push_back(next_node);
                    }
                }
            }
        }
};

class Node {
    public:
        int id;
        Node(int id) : id(id) {}
};

class BreadthFirstSearch {
    public:
        BreadthFirstSearch() {
            std::cout << "BreadthFirstSearch constructor called." << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph) {
            std::cout << "Performing breadth-first search..." << std::endl;
            // Implementation of breadth-first search algorithm goes here.
            std::cout << "Searching from node " << startNode
                    << " to node " << goalNode << std::endl;

            // Similar implementation as DepthFirstSearch but using a queue for BFS
            std::vector<int> queue;
            queue.push_back(startNode);
            std::vector<int> visited;
            visited.push_back(startNode);

            int iteration = 0;
            while(queue.size() > 0) {
                iteration++;
                int current_node = queue[0];

                std::cout << "Current node: " << current_node << std::endl;

                queue.erase(queue.begin()); // or queue.pop_back()
                if(current_node == goalNode){
                    std::cout << "Goal node found: " << goalNode << std::endl;
                    std::cout << "Total iterations: " << iteration << std::endl;
                    return;
                }
                
                std::vector<int> next_nodes = graph[current_node];
                int len = next_nodes.size();

                for(int i = 0; i < len; i++) {
                    int next_node = next_nodes[i];

                    bool found = (std::find(visited.begin(), visited.end(), next_node) != visited.end());

                    if(!found) {
                        queue.push_back(next_node);
                        visited.push_back(next_node);
                    }
                }
            }
        }
};

int main() {
    // Create a simple graph as an adjacency list
    // Example: 0 -> {1, 2}, 1 -> {2}, 2 -> {0, 3}, 3 -> {}
    std::vector<std::vector<int>> graph = {
        {1, 2},    // 0 → 1, 2
        {3, 4},    // 1 → 3, 4
        {5},       // 2 → 5
        {},        // 3 → (none)
        {},        // 4 → (none)
        {}         // 5 → (none)
    };

    DepthFirstSearch dfs;
    int startNode = 0;
    int goalNode = 3;
    dfs.search(startNode, goalNode, graph);
    return 0;
}