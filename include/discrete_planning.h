#ifndef DISCRETE_PLANNING_H
#define DISCRETE_PLANNING_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <deque> 
#include "utils.h"

class DepthFirstSearch: public Search {
    public:
        DepthFirstSearch() {
            std::cout << "\nDepthFirstSearch constructor called.\n" << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph);
};

class BreadthFirstSearch: public Search {
    public:
        BreadthFirstSearch() {
            std::cout << "\nBreadthFirstSearch constructor called.\n" << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph);
};

class IterativeDeepening: public Search {
    public:
        IterativeDeepening() {
            std::cout << "\nIterativeDeepening constructor called.\n" << std::endl;
        }

        void search(int startNode, int goalNode,
                    const std::vector<std::vector<int>>& graph);
};

#endif