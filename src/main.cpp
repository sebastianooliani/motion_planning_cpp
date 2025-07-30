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
#include <fstream>

#define SAMPLING_POINTS 1000

int main() {
    Map map;

    // Sample a number of points in free space (you can increase this)
    std::vector<Point> sampled_points;
    for (int i = 0; i < SAMPLING_POINTS; ++i) {
        sampled_points.push_back(sampleFreePoint(map));
    }

    // Display the map with sampled points
    std::cout << "Sampled Points in Free Space:\n";
    map.display(sampled_points);  // Visualize the map and sampled nodes

    // write down the sampled points
    std::cout << "Sampled Points:\n";
    for (const auto& p : sampled_points) {
        std::cout << "(" << p.x << ", " << p.y << ")\n";
    }

    std::ofstream file("graph.txt");

    for (const auto& p : sampled_points) {
        file << "N " << p.x << " " << p.y << "\n";
    }
    // to get the edges, access the map and look for '#' characters
    for(int y = 0; y < MAP_HEIGHT; ++y) {
        for (int x = 0; x < MAP_WIDTH; ++x) {
            if (map.isFree(x, y)) {
                // Check neighbors to find edges
                if (map.isFree(x + 1, y)) { // Right neighbor
                    file << "E " << x << " " << y << " " << x + 1 << " " << y << "\n";
                }
                if (map.isFree(x, y + 1)) { // Down neighbor
                    file << "E " << x << " " << y << " " << x << " " << y + 1 << "\n";
                }
            }
        }
    }
    // for (const auto& edge : edges) {
    //     const Point& a = nodes[edge.first];
    //     const Point& b = nodes[edge.second];
    //     file << "E " << a.x << " " << a.y << " " << b.x << " " << b.y << "\n";
    // }
    file.close();

    // Your PRM logic (graph construction, neighbor connection, etc.) goes here
    PRMPlanner prm(map);
    // std::vector<std::vector<int>> graph = prm.build_connected_graph(sampled_points, map);
    std::vector<std::vector<std::pair<int, double>>> graph_and_costs = prm.build_connected_graph_and_costs(sampled_points, map);

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
    AStarSearch aStar;
    // prm.findPath(startNode, goalNode, bfs, graph);
    prm.findPath_and_costs(startNode, goalNode, aStar, graph_and_costs);
}
