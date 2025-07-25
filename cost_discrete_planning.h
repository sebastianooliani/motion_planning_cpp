#ifndef COST_DISCRETE_PLANNING_H
#define COST_DISCRETE_PLANNING_H

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
                    const std::vector<std::vector<std::vector<int>>>& graph_and_costs);
};

class AStarSearch: public Search {
    public:
        AStarSearch() {
            std::cout << "\nAStarSearch constructor called.\n" << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<std::vector<int>>>& graph_and_costs);
};

#endif