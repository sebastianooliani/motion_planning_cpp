#include <iostream>
#include <vector>
#include <algorithm>
#include "utils.h"

std::vector<int> returnpath(int startNode, int goalNode,
                    const std::vector<int>& parent) {
    std::cout << "Returning path from node " << startNode
              << " to node " << goalNode << std::endl;
    // This function would typically return the path found by the search algorithm.
    // For now, we will just return an empty vector.
    // start from goal node and go to start node
    std::vector<int> path;
    if (startNode == goalNode) {
        path.push_back(startNode);
        return path;
    }
    int prev_node = parent[goalNode]; // set parent of goal node to start node
    path.push_back(goalNode);
    while (prev_node != startNode) {
        path.push_back(prev_node);
        prev_node = parent[prev_node]; // go to parent of previous node
    }
    // finally, revert the path to start from start node
    path.push_back(startNode);
    std::reverse(path.begin(), path.end());
    return path;
}

void printPath(const std::vector<int>& path) {
    std::cout << "Path: ";
    for (int i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
}
