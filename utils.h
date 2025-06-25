#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <algorithm>

std::vector<int> returnpath(int startNode, int goalNode,
                     const std::vector<int>& parent);

void printPath(const std::vector<int>& path);

#endif