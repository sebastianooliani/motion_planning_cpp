#include <iostream>
#include <vector>
#include <random>
#include <cmath>

#include "utils.h"
#include "discrete_planning.h"
#include "sampling_based_planning.h"

// Randomly sample a valid point in free space on the map
Point sampleFreePoint(const Map& map) {
    static std::default_random_engine gen;
    static std::uniform_real_distribution<double> x_dist(0, MAP_WIDTH - 1);
    static std::uniform_real_distribution<double> y_dist(0, MAP_HEIGHT - 1);

    while (true) {
        int x = static_cast<int>(std::round(x_dist(gen)));
        int y = static_cast<int>(std::round(y_dist(gen)));
        if (map.isFree(x, y)) {
            return Point{static_cast<double>(x), static_cast<double>(y)};
        }
    }
}

bool isCollisionFree(const Map& map, int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (!map.isFree(x0, y0)) return false;  // Obstacle hit
        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return true;  // No obstacle along the line
}

std::vector<std::vector<int>> PRMPlanner::find_neighbours(const std::vector<Point>& sampled_points, double distance_threshold) {
    std::vector<std::vector<int>> graph(sampled_points.size());
    for (size_t i = 0; i < sampled_points.size(); ++i) {
        const auto& point = sampled_points[i];
        for (size_t j = 0; j < sampled_points.size(); ++j) {
            if (i == j) continue;
            const auto& other = sampled_points[j];
            double distance = std::sqrt(std::pow(point.x - other.x, 2) +
                                        std::pow(point.y - other.y, 2));
            if (distance <= distance_threshold) {
                if(isCollisionFree(map, static_cast<int>(point.x), static_cast<int>(point.y),
                                static_cast<int>(other.x), static_cast<int>(other.y))) {
                    graph[i].push_back(static_cast<int>(j));  // Store the neighbor index
                }
            }
        }
    }
    return graph;
}

bool PRMPlanner::isConnected(const std::vector<std::vector<int>>& graph) {
    for (size_t i = 0; i < graph.size(); ++i) {
        if (graph[i].empty()) {
            /*std::cout << "Node " << i << " has no connections." << std::endl;*/
            return false;  // If any node has no connections, the graph is not connected
        }
    }
    return true;  // If all nodes have connections, the graph is connected
}

std::vector<std::vector<int>> PRMPlanner::build_connected_graph(std::vector<Point>& sampled_points) {
    // find the nearest neighbors for each sampled point
    // consider a distance threshold for connection
    double connection_distance = 5.0;  // Example threshold
    std::vector<std::vector<int>> graph(sampled_points.size());

    graph = find_neighbours(sampled_points, connection_distance);

    // check if the graph is connected
    bool is_not_connected = true;
    while (is_not_connected) {
        bool result = isConnected(graph);
        if (result) {
            is_not_connected = false;
        } else {
            // increase the distance threshold
            connection_distance += 1.0;
            // Rebuild the graph with the new distance threshold
            graph.clear();
            graph = find_neighbours(sampled_points, connection_distance);
        }
    }
    std::cout << '\n';
    std::cout << "Graph is connected with distance threshold: " << connection_distance << std::endl;
    std::cout << "Total nodes in graph: " << graph.size() << std::endl;
    std::cout << '\n';
    // Return the connected graph
    return graph;
}

// int main() {
//     Map map;

//     // Sample a number of points in free space (you can increase this)
//     std::vector<Point> sampled_points;
//     for (int i = 0; i < 20; ++i) {
//         sampled_points.push_back(sampleFreePoint(map));
//     }

//     std::cout << "Sampled Points in Free Space:\n";
//     map.display(sampled_points);  // Visualize the map and sampled nodes

//     // write dowmn the sampled points
//     std::cout << "Sampled Points:\n";
//     for (const auto& p : sampled_points) {
//         std::cout << "(" << p.x << ", " << p.y << ")\n";
//     }

//     // Your PRM logic (graph construction, neighbor connection, etc.) goes here
//     PRMPlanner prm(map);
//     std::vector<std::vector<int>> graph = prm.build_connected_graph(sampled_points);

//     // print the graph structure
//     /*for (size_t i = 0; i < graph.size(); ++i) {
//         std::cout << "Node " << i << " connects to: ";
//         for (int neighbor : graph[i]) {
//             std::cout << neighbor << " ";
//         }
//         std::cout << std::endl;
//     }*/
//     // Example usage of a search algorithm
//     int startNode = 0;  // Starting from the first sampled point
//     int goalNode = 5;   // Example goal node

//     // print the actual coordinates of the start and goal nodes
//     std::cout << "Start Node: (" << sampled_points[startNode].x << ", " << sampled_points[startNode].y << ")\n";
//     std::cout << "Goal Node: (" << sampled_points[goalNode].x << ", " << sampled_points[goalNode].y << ")\n";

//     BreadthFirstSearch bfs;
//     prm.findPath(startNode, goalNode, bfs, graph);
// }
