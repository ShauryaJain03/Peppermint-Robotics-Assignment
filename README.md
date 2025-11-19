# A* Search For Finding Path in 2D Grid

## Overview
This repository demonstrates the implementation of A* algorithm to compute an optimal path between a start cell and a goal cell on a 2D grid, as part of the Internship Hiring Challenge at Peppermint Robotics.

---

## Table of Contents

- [Introduction](#introduction)
- [Assumptions](#assumptions)
- [Features](#features)
- [Setup Instructions](#setup-instructions)
- [Implementation Details](#implementation-details)
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


