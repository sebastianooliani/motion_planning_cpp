# Motion Planning C++

A C++ library for motion planning algorithms, designed for robotics and autonomous systems. This project provides implementations of popular motion planning techniques and utilities for pathfinding and trajectory generation.

<!--
    This section outlines the key features of the motion_planning_cpp project.
    - Highlights the modular and extensible design, enabling easy integration of new algorithms and components.
    - Lists supported motion planning algorithms, including RRT (Rapidly-exploring Random Tree), PRM (Probabilistic Roadmap), and A*.
    - Specifies compatibility with both 2D and 3D workspaces for versatile application.
    - Mentions built-in utilities for collision detection to ensure safe path planning.
    - Notes the inclusion of example applications and demonstration scripts to help users get started quickly.

## Features

- Modular and extensible architecture
- Implementations of algorithms such as RRT, PRM, and A*
- 2D/3D workspace support
- Collision checking utilities
- Example applications and demos
-->
## Getting Started

### Prerequisites

- C++14 compatible compiler
- CMake 3.10 or higher

### Build Instructions

For Ubuntu users:
```bash
git clone https://github.com/sebastianooliani/motion_planning_cpp.git
cd motion_planning_cpp
mkdir build && cd build
cmake ..
make
```

For Windows users:
```bash
g++ -std=c++14 discrete_planning.cpp -o main
./main.exe
```
<!--
### Usage

Include the library in your project or run the provided examples:

```bash
./examples/rrt_example
```

## Documentation

See the [docs](docs/) folder for detailed API documentation and usage guides.
-->
## Contributing

Contributions are welcome! Please open issues or submit pull requests.

## License

This project is licensed under the MIT License.