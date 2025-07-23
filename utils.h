#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <algorithm>

std::vector<int> returnpath(int startNode, int goalNode,
                     const std::vector<int>& parent);

void printPath(const std::vector<int>& path);

class Search{
    public:
        int checkNode(int startNode, int goalNode, int graph_length) {
            if (graph_length == 0) {
                std::cout << "Graph is empty." << std::endl;
                return 0;
            }
            if (startNode < 0 || startNode >= graph_length || 
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

        int find_index_minimum_cost_heuristic(const std::vector<int>& queue,
                                         const std::vector<int>& cost,
                                         const std::vector<int>& heuristic) {
            int min_index = -1;
            int min_cost = 100000;

            for (int i = 0; i < queue.size(); ++i) {
                int total_cost = cost[queue[i]] + heuristic[queue[i]];
                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    min_index = i;
                }
            }
            return min_index;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph) {
            std::cout << "Implement your search algorithm here!" << std::endl;
        }
};

#endif