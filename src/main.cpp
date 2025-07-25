/**
 * @file main.cpp
 * @brief Main file for the motion planning project
 *
 * This file contains the entry point for the motion planning application.
 * When compliling, include the necessary headers and link against the required libraries.
 * The main function initializes the map and can be extended to include various planning algorithms.
 */

#include "utils.h"
#include "discrete_planning.h"
#include "cost_discrete_planning.h"
#include "sampling_based_planning.h"

int main() {
    Map map;

    // Sample a number of points in free space (you can increase this)
    std::vector<Point> sampled_points;
    for (int i = 0; i < 20; ++i) {
        sampled_points.push_back(sampleFreePoint(map));
    }

    std::cout << "Sampled Points in Free Space:\n";
    map.display(sampled_points);  // Visualize the map and sampled nodes

    // write dowmn the sampled points
    std::cout << "Sampled Points:\n";
    for (const auto& p : sampled_points) {
        std::cout << "(" << p.x << ", " << p.y << ")\n";
    }

    // Your PRM logic (graph construction, neighbor connection, etc.) goes here
    PRMPlanner prm(map);
    std::vector<std::vector<int>> graph = prm.build_connected_graph(sampled_points);

    // print the graph structure
    /*for (size_t i = 0; i < graph.size(); ++i) {
        std::cout << "Node " << i << " connects to: ";
        for (int neighbor : graph[i]) {
            std::cout << neighbor << " ";
        }
        std::cout << std::endl;
    }*/
    // Example usage of a search algorithm
    int startNode = 0;  // Starting from the first sampled point
    int goalNode = 5;   // Example goal node

    // print the actual coordinates of the start and goal nodes
    std::cout << "Start Node: (" << sampled_points[startNode].x << ", " << sampled_points[startNode].y << ")\n";
    std::cout << "Goal Node: (" << sampled_points[goalNode].x << ", " << sampled_points[goalNode].y << ")\n";

    BreadthFirstSearch bfs;
    prm.findPath(startNode, goalNode, bfs, graph);
}
