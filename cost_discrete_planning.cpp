#include <iostream>
#include <vector>
#include <algorithm>
#include <deque>
#include "utils.h"

class UniformCostSearch: public Search {
    public:
        UniformCostSearch() {
            std::cout << "\nUniformCostSearch constructor called.\n" << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<std::vector<int>>>& graph_and_costs) {
            std::cout << "Performing uniform cost search..." << std::endl;
            std::cout << "Searching from node " << startNode
                      << " to node " << goalNode << std::endl;

            if(checkNode(startNode, goalNode, graph_and_costs.size()) == 0) {
                return;
            }

            std::vector<int> queue;
            queue.push_back(startNode);
            std::vector<int> parent(graph_and_costs.size(), -1);
            std::vector<int> cost(graph_and_costs.size(), 10000); // cost to reach each node
            cost[startNode] = 0; // cost of start node is 0

            int iteration = 0;
            while(queue.size() > 0) {
                iteration++;
                int idx = find_index_minimum_cost(queue, cost);
                int current_node = queue[idx];

                // Remove the current node from the queue
                queue.erase(queue.begin() + idx);

                if(current_node == goalNode) {
                    std::cout << "Goal node found: " << goalNode << std::endl;
                    std::cout << "Total iterations: " << iteration << std::endl;
                    std::cout << "Cost to reach goal node: " << cost[goalNode] << std::endl;

                    std::vector<int> path = returnpath(startNode, goalNode, parent);
                    int path_cost = cost[goalNode];
                    printPath(path);
                    std::cout << "Path cost: " << path_cost << std::endl;
                    return;
                }

                std::vector<std::vector<int>> next_nodes = graph_and_costs[current_node];
                int len = next_nodes.size();
                for(int i = 0; i < len; i++) {
                    int next_node = next_nodes[i][0];
                    int edge_cost = next_nodes[i][1];
                    int new_cost = cost[current_node] + edge_cost;

                    if(new_cost < cost[next_node]){
                        cost[next_node] = new_cost; 
                        parent[next_node] = current_node; // set parent of next node
                        queue.push_back(next_node); // add next node to queue
                    }
                }
            }
            std::cout << "Goal node not found." << std::endl;
            std::cout << "Total iterations: " << iteration << std::endl;
        }
    };

class AStarSearch: public Search {
    public:
        AStarSearch() {
            std::cout << "\nA* search constructor called.\n" << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<std::vector<int>>>& graph_and_costs) {
            std::cout << "Performing A* search..." << std::endl;
            std::cout << "Searching from node " << startNode
                      << " to node " << goalNode << std::endl;

            if(checkNode(startNode, goalNode, graph_and_costs.size()) == 0) {
                return;
        }

        std::vector<int> queue;
        queue.push_back(startNode);
        std::vector<int> parent(graph_and_costs.size(), -1);
        std::vector<int> cost(graph_and_costs.size(), 10000); // cost to reach each node
        cost[startNode] = 0; // cost of start node is 0
        std::vector<int> heuristic(graph_and_costs.size(), 1); // heuristic values for each node

        int iteration = 0;
        while(queue.size() > 0) {
            iteration++;
            int idx = find_index_minimum_cost_heuristic(queue, cost, heuristic);
            int current_node = queue[idx];
            queue.erase(queue.begin() + idx);

            if (current_node == goalNode) {
                std::cout << "Goal node found!" << std::endl;
                std::cout << "Total iterations: " << iteration << std::endl;

                std::vector<int> path = returnpath(startNode, goalNode, parent);
                int path_cost = cost[goalNode];
                printPath(path);
                std::cout << "Path cost: " << path_cost << std::endl;
                return;
            }

            std::vector<std::vector<int>> next_nodes = graph_and_costs[current_node];
            int len = next_nodes.size();
            for(int i = 0; i < len; i++) {
                int next_node = next_nodes[i][0];
                int edge_cost = next_nodes[i][1];
                int new_cost = cost[current_node] + edge_cost;

                if (new_cost < cost[next_node]) {
                    cost[next_node] = new_cost;
                    parent[next_node] = current_node;
                    queue.push_back(next_node);
                }
            }
        }
    }
};

// Example usage
int main() {
    UniformCostSearch ucs;
    // Each pair is (neighbor, cost)
    std::vector<std::vector<std::vector<int>>> graph = {
        {{1, 2}, {2, 4}},        // Node 0: to 1 (cost 2), to 2 (cost 4)
        {{0, 2}, {3, 7}, {4, 3}},// Node 1: to 0 (2), to 3 (7), to 4 (3)
        {{0, 4}, {4, 1}},        // Node 2: to 0 (4), to 4 (1)
        {{1, 7}, {5, 1}},        // Node 3: to 1 (7), to 5 (1)
        {{1, 3}, {2, 1}, {5, 5}},// Node 4: to 1 (3), to 2 (1), to 5 (5)
        {{3, 1}, {4, 5}}         // Node 5: to 3 (1), to 4 (5)
    };
    ucs.search(0, 5, graph);

    AStarSearch a_search;
    a_search.search(0, 5, graph);
    return 0;
}