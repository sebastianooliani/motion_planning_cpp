#ifndef SAMPLING_BASED_PLANNING_H
#define SAMPLING_BASED_PLANNING_H

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

// Map dimensions
const int MAP_WIDTH = 40;
const int MAP_HEIGHT = 20;

// Simple 2D point structure
struct Point {
    double x, y;
};

// Class representing a 2D grid map with obstacles
class Map {
    public:
        Map() {
            // Initialize grid with all cells free (false means free)
            grid.resize(MAP_HEIGHT, std::vector<bool>(MAP_WIDTH, false));
            // Add static obstacles to the map
            // keep memory of where the obstacles are
            obstacles = addObstacles();
        }

        // Check if a cell is free (not an obstacle)
        bool isFree(int x, int y) const {
            if (x < 0 || y < 0 || x >= MAP_WIDTH || y >= MAP_HEIGHT) return false;
            return !grid[y][x];
        }

        // Display the grid map
        // '.' = free space, '#' = obstacle, 'o' = sampled point
        void display(const std::vector<Point>& nodes = {}) const {
            for (int y = 0; y < MAP_HEIGHT; ++y) {
                for (int x = 0; x < MAP_WIDTH; ++x) {
                    bool printed = false;
                    // Check if this cell matches any sampled node
                    for (const auto& p : nodes) {
                        if (std::round(p.x) == x && std::round(p.y) == y) {
                            std::cout << 'o';  // Node marker
                            printed = true;
                            break;
                        }
                    }
                    if (!printed) {
                        std::cout << (grid[y][x] ? '#' : '.');
                    }
                }
                std::cout << '\n';
            }
        }

        const std::vector<std::vector<int>>& getObstacles() const {
            return obstacles;
        }

    private:
        std::vector<std::vector<bool>> grid;  // true = obstacle, false = free
        std::vector<std::vector<int>> obstacles;  // keep track of obstacle positions

        // Add static obstacles to the map
        const std::vector<std::vector<int>> addObstacles() {
            std::vector<std::vector<int>> obstacles;
            // Horizontal wall
            for (int x = 10; x < 30; ++x) {
                grid[10][x] = true;
                obstacles.push_back({x, 10});
            }
            // Vertical barrier
            for (int y = 0; y < 10; ++y) {
                grid[y][20] = true;
                obstacles.push_back({20, y});
            }
            return obstacles;
        }
};

// Randomly sample a valid point in free space on the map
Point sampleFreePoint(const Map& map);

bool isCollisionFree(const Map& map, int x0, int y0, int x1, int y1);

class PRMPlanner {
    public:
        PRMPlanner(const Map& map) : map(map) {}

        std::vector<std::vector<int>> find_neighbours(const std::vector<Point>& sampled_points, double distance_threshold);

        bool isConnected(const std::vector<std::vector<int>>& graph);

        std::vector<std::vector<int>> build_connected_graph(std::vector<Point>& sampled_points);

        template <typename searchAlgorithm>
        // given a search algorithm, a connected graph, start node and goal node, return the path
        void findPath(int startNode, int goalNode, searchAlgorithm algo, const std::vector<std::vector<int>>& graph) {
            algo.search(startNode, goalNode, graph);
        }
        
    private:
        const Map& map;
        std::vector<Point> sampled_points;
};

#endif