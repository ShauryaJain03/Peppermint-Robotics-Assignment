# A* Search For Finding Path in 2D Grid

## Overview
This repository demonstrates the implementation of A* algorithm to compute an optimal path between a start cell and a goal cell on a 2D grid, as part of the Internship Hiring Challenge at Peppermint Robotics.

---

## Table of Contents

- [Introduction](#introduction)
- [Assumptions](#assumptions)
- [Features](#features)
- [Setup Instructions](#setup-instructions)
- [Implementation Details with Pseudocode](#implementation-details-with-pseudocode)
- [Results](#results)

## Introduction

This implementation solves the classic grid based path planning problem by computing a shortest path for an agent moving in 4 directions (up, down, left, right) on a 2D map containing obstacles.
The output is a sequence of grid coordinates that represent the computed path from the start to the goal.
If no path exists, the algorithm returns None.

## Assumptions 

- Start and goal are within bounds
- 4 direction movement only (up, down, left, right), no diagonal movement allowed
- Unit cost = 1 for each move
- Manhattan heuristic chosen for calculating 'h' as movement is assumed to be in 4 directions only


## Features 

- A* search algorithm with Manhattan heuristic
- 4 direction grid movement
- Reconstructs and returns the full path
- Visual output showing the found path marked using '-'

## Setup Instructions

### 0. Prerequisites
Python must be installed on your system

### 1. Clone the repository

```bash
git clone https://github.com/ShauryaJain03/Peppermint-Robotics-Assignment.git
```

### 2. Run the script
```bash
cd Peppermint-Robotics-Assignment/
python3 astar.py
```
## Implementation Details with Pseudocode

The A* algorithm computes the shortest path by evaluating cells based on the function: f(n)=g(n)+h(n)

Where

g(n) → cost from start to node

h(n) → heuristic estimate from node to goal (Manhattan distance)

f(n) → priority score used to choose the next node to explore

The algorithm uses a priority queue (open_list) to always expand the most promising node first.
Movement is allowed in 4 directions: up, down, left, right.


This is the pseudocode for the algorithm
```
function A*(start, goal, grid):
    open_list = priority queue ordered by f-score
    push start into open_list with f = 0

    g_score[start] = 0
    parent = empty map

    while open_list not empty:
        current = node with smallest f from open_list

        if current == goal:
            return reconstruct_path(parent, current)

        for each neighbor of current in (up, down, left, right):
            if neighbor is within grid bounds AND grid[neighbor] == 0:
                tentative_g = g_score[current] + 1

                if neighbor not visited OR tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    push neighbor to open_list
                    parent[neighbor] = current

    return None  


```
## Results

[![The implementation tested on a 10x10 grid](https://github.com/user-attachments/assets/b953e608-a0c5-42ea-b345-35f592bd3939)](https://www.youtube.com/watch?v=QnNHx6G1rHI)
