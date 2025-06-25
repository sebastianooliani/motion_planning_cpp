#include <iostream>
#include <vector>
#include <algorithm>
#include <deque>
#include "utils.h"

class UniformCostSearch {
    public:
        UniformCostSearch() {
            std::cout << "\nUniformCostSearch constructor called.\n" << std::endl;
        }

        int checkNode(int startNode, int goalNode,
                      int graph_length) {
            // This function checks if the start node is the same as the goal node.
            // If they are the same, it returns 1, otherwise it returns 0.
            if(graph_length == 0) {
                std::cout << "Graph is empty." << std::endl;
                return 0;
            }
            if(startNode < 0 || startNode >= graph_length || 
               goalNode < 0 || goalNode >= graph_length) {
                std::cout << "Invalid start or goal node." << std::endl;
                return 0;
            }

            return 1;
        }

        int find_index_minimum_cost(const std::vector<int>& queue,
                                         const std::vector<int>& cost) {
            int min_index = -1;
            int min_cost = 100000;

            for (int i = 0; i < queue.size(); ++i) {
                if (cost[queue[i]] < min_cost) {
                    min_cost = cost[queue[i]];
                    min_index = i;
                }
            }
            return min_index;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph) {
            std::cout << "Performing uniform cost search..." << std::endl;
            std::cout << "Searching from node " << startNode
                      << " to node " << goalNode << std::endl;

            if(checkNode(startNode, goalNode, graph.size()) == 0) {
                return;
            }

            std::vector<int> queue;
            queue.push_back(startNode);
            std::vector<int> parent(graph.size(), -1);
            std::vector<int> cost(graph.size(), 0); // cost to reach each node
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

                    std::vector<int> path = returnpath(startNode, goalNode, parent);
                    printPath(path);
                    return;
                }

                std::vector<int> next_nodes = graph[current_node];
                int len = next_nodes.size();
                for(int i = 0; i < len; i++) {
                    int next_node = next_nodes[i];
                    //TODO: see how to read cost from graph
                    int new_cost = cost[current_node] + 1; // assuming cost is 1 for each edge

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