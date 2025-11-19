"""
Very simple A* implementation on a 2D grid.

Assumptions:
- 0 = free cell
- 1 = obstacle
- Movement allowed: up, down, left, right (no diagonals)
- Cost of each move = 1
- Heuristic: Manhattan distance
"""

def manhattan(a, b):
    """Heuristic: Manhattan distance between two (row, col) points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(cell, grid):
    """Return valid 4-connected neighbors (up, down, left, right)."""
    (r, c) = cell
    neighbors = []
    rows = len(grid)
    cols = len(grid[0])

    # Up
    if r - 1 >= 0 and grid[r - 1][c] == 0:
        neighbors.append((r - 1, c))
    # Down
    if r + 1 < rows and grid[r + 1][c] == 0:
        neighbors.append((r + 1, c))
    # Left
    if c - 1 >= 0 and grid[r][c - 1] == 0:
        neighbors.append((r, c - 1))
    # Right
    if c + 1 < cols and grid[r][c + 1] == 0:
        neighbors.append((r, c + 1))

    return neighbors


def reconstruct_path(came_from, current):
    """Rebuild the path from start to goal using the came_from map."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(start_point, goal_point, grid):
    """
    A* search on a grid.

    Inputs:
        start_point: (row, col)
        goal_point: (row, col)
        grid: 2D list with 0 = free, 1 = obstacle

    Output:
        path: list of (row, col) from start to goal, or None if no path.
    """

    open_set = [start_point]       # cells to be evaluated
    came_from = {}                 # child -> parent
    g_score = {start_point: 0}     # cost from start to this cell

    while open_set:
        # Pick the cell in open_set with the lowest f = g + h
        current = None
        current_f = None
        for cell in open_set:
            g = g_score.get(cell, float("inf"))
            h = manhattan(cell, goal_point)
            f = g + h
            if current is None or f < current_f:
                current = cell
                current_f = f

        # If we reached the goal, reconstruct the path
        if current == goal_point:
            return reconstruct_path(came_from, current)

        # Move current from open_set to closed_set (implicitly, by removal)
        open_set.remove(current)

        # Check all neighbors
        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + 1  # cost between neighbors is 1

            # If this is a better path to neighbor, record it
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                if neighbor not in open_set:
                    open_set.append(neighbor)

    # No path found
    return None


if __name__ == "__main__":
    # Example from the problem statement
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    start_point = (0, 0)
    goal_point = (2, 2)

    path = astar(start_point, goal_point, grid)

    print("Grid:")
    for row in grid:
        print(row)
    print("\nStart:", start_point)
    print("Goal: ", goal_point)

    print("\nPath:")
    print(path)
