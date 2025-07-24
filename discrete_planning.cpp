#include <iostream>
#include <vector>
#include <algorithm>
#include <deque> 
#include "utils.h"
#include "discrete_planning.h"

void DepthFirstSearch::search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph) {
    std::cout << "Performing depth-first search..." << std::endl;
    // Implementation of depth-first search algorithm goes here.
    std::cout << "Searching from node " << startNode
            << " to node " << goalNode << std::endl;

    if(checkNode(startNode, goalNode, graph.size()) == 0) {
        return;
    }

    std::deque<int> queue;
    queue.push_back(startNode);
    std::vector<int> visited;
    visited.push_back(startNode);
    // parent vector to store the parent of each node
    std::vector<int> parent(graph.size(), -1);

    int iteration = 0;
    while(queue.size() > 0) {
        iteration++;
        int current_node = queue[0];

        queue.erase(queue.begin()); // or queue.pop_back()
        if(current_node == goalNode){
            std::cout << "Goal node found: " << goalNode << std::endl;
            std::cout << "Total iterations: " << iteration << std::endl;

            std::vector<int> path = returnpath(startNode, goalNode, parent);
            printPath(path);
            return;
        }
        
        std::vector<int> next_nodes = graph[current_node];
        int len = next_nodes.size();
        std::vector<int> add_to_queue;

        for(int i = 0; i < len; i++) {
            int next_node = next_nodes[i];

            bool found = (std::find(visited.begin(), visited.end(), next_node) != visited.end());

            if(!found) {
                add_to_queue.push_back(next_node);
                visited.push_back(next_node);
                parent[next_node] = current_node; // set parent of next node
            }
        }
        // Add all next nodes to the queue
        for(int i = 0; i < add_to_queue.size(); i++) {
            queue.push_front(add_to_queue[add_to_queue.size() - 1 - i]); // reverse order to maintain DFS-like behavior
        }
    }
    std::cout << "Goal node not found within max depth." << std::endl;
    std::cout << "Total iterations: " << iteration << std::endl;
};

void BreadthFirstSearch::search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph) {
    std::cout << "Performing breadth-first search..." << std::endl;
    // Implementation of breadth-first search algorithm goes here.
    std::cout << "Searching from node " << startNode
            << " to node " << goalNode << std::endl;

    if(checkNode(startNode, goalNode, graph.size()) == 0) {
        return;
    }

    // Similar implementation as DepthFirstSearch but using a queue for BFS
    std::vector<int> queue;
    queue.push_back(startNode);
    std::vector<int> visited;
    visited.push_back(startNode);
    // parent vector to store the parent of each node
    std::vector<int> parent(graph.size(), -1);

    int iteration = 0;
    while(queue.size() > 0) {
        iteration++;
        int current_node = queue[0];

        queue.erase(queue.begin());
        if(current_node == goalNode){
            std::cout << "Goal node found: " << goalNode << std::endl;
            std::cout << "Total iterations: " << iteration << std::endl;

            std::vector<int> path = returnpath(startNode, goalNode, parent);
            printPath(path);
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
                parent[next_node] = current_node; // set parent of next node
            }
        }
    }
    std::cout << "Goal node not found within max depth." << std::endl;
    std::cout << "Total iterations: " << iteration << std::endl;
};

void IterativeDeepening::search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph) {
    std::cout << "Performing iterative deepening search..." << std::endl;
    // Implementation of iterative deepening search algorithm goes here.
    // This is a placeholder for the actual implementation.
    std::cout << "Searching from node " << startNode
            << " to node " << goalNode << std::endl;

    if(checkNode(startNode, goalNode, graph.size()) == 0) {
        return;
    }
    
    int max_depth = graph.size(); // assuming max depth is the size of the graph         
    int current_depth = 1;
    int iteration = 0;
    // Start with a depth of 1 and increase until the goal is found or max depth is reached
    std::cout << "Max depth: " << max_depth << std::endl;
    std::cout << "Starting depth: " << current_depth << std::endl;
    

    int current_node = startNode;
    while(current_depth < max_depth) {

        std::deque<int> queue;
        queue.push_back(startNode);
        std::vector<int> visited;
        visited.push_back(startNode);
        std::vector<int> depth(graph.size(), -1);
        depth[startNode] = 1; // set depth of start node to 1    
        // parent vector to store the parent of each node
        std::vector<int> parent(graph.size(), -1);

        while(queue.size() > 0) {
            iteration++;
            current_node = queue[0];

            queue.erase(queue.begin()); // or queue.pop_back()
            if(current_node == goalNode){
                std::cout << "Goal node found: " << goalNode << ", at depth: " << current_depth << std::endl;
                std::cout << "Total iterations: " << iteration << std::endl;

                std::vector<int> path = returnpath(startNode, goalNode, parent);
                printPath(path);
                return;
            }
            
            std::vector<int> next_nodes = graph[current_node];
            int len = next_nodes.size();
            std::vector<int> add_to_queue;

            for(int i = 0; i < len; i++) {
                int next_node = next_nodes[i];

                bool found = (std::find(visited.begin(), visited.end(), next_node) != visited.end());

                if(!found) {
                    int next_depth = depth[current_node] + 1; // increment depth of next node
                    if(next_depth > current_depth) {
                        std::cout << "Reached max depth for this iteration: " << current_depth << std::endl;
                        continue; // skip adding nodes beyond the current depth
                    }
                    depth[next_node] = next_depth; // set depth of next node
                    add_to_queue.push_back(next_node);
                    visited.push_back(next_node);
                    parent[next_node] = current_node; // set parent of next node
                }
            }
            // Add all next nodes to the queue
            for(int i = 0; i < add_to_queue.size(); i++) {
                queue.push_front(add_to_queue[add_to_queue.size() - 1 - i]); // reverse order to maintain DFS-like behavior
            }
        }
        current_depth++;
        std::cout << "\nIncreasing depth to: " << current_depth << std::endl;
    }
    std::cout << "Goal node not found within max depth." << std::endl;
    std::cout << "Total iterations: " << iteration << std::endl;
};

/*int main() {
    // Create a simple graph as an adjacency list
    // Example: 0 -> {1, 2}, 1 -> {2}, 2 -> {0, 3}, 3 -> {}
    std::vector<std::vector<int>> graph = {
        {1, 2},        // 0 → 1, 2
        {3, 4},        // 1 → 3, 4
        {5, 6},        // 2 → 5, 6
        {7},           // 3 → 7
        {8},           // 4 → 8
        {9, 10},       // 5 → 9, 10
        {11},          // 6 → 11
        {12},          // 7 → 12
        {13},          // 8 → 13
        {},            // 9 → (none)
        {4},           // 10 → 4 (cycle back)
        {14},          // 11 → 14
        {},            // 12 → (none)
        {},            // 13 → (none)
        {0, 6}         // 14 → 0, 6 (back edges creating cycles)
    };

    int startNode = 0; // Starting node
    int goalNode = 4;  // Goal node
    DepthFirstSearch dfs;
    dfs.search(startNode, goalNode, graph);

    BreadthFirstSearch bfs;
    bfs.search(startNode, goalNode, graph);

    IterativeDeepening id;
    id.search(startNode, goalNode, graph);
    return 0;
}*/