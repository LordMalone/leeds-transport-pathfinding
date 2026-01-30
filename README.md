# Leeds Transport Pathfinding

A Python implementation of Dijkstra's and A* algorithms for the Leeds transport network (Bus, Train, and Proposed Metro).

## Features
- **Dijkstra's Algorithm**: Finds the absolute shortest path based on edge weights (time or distance).
- **A* Algorithm**: Optimized pathfinding using the Haversine formula as a heuristic.
- **Leeds Dataset**: Includes major rail stations, bus hubs, and proposed Mass Transit (Metro) stops.
- **Multimodal Support**: Different speeds and wait times for different transport modes.

## Installation
No external dependencies required (standard library only).

```bash
python leeds_pathfinding.py
```

## Dataset Overview
The graph covers:
- Leeds City Station (Rail)
- Leeds City Bus Station
- Proposed Metro stops (St James' Hospital, Victoria Bridge, etc.)
- Regional rail links (Bradford, Morley, Cross Gates)
