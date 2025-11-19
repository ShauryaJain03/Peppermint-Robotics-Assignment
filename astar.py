import heapq

#Heuristic function to calculate h: Manhattan distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

#Get 4 connected neighbors, assuming movement is allowed only in 4 directions (up,down,left,right)
def get_neighbors(cell, grid_map):
    (r, c) = cell
    neighbors = []
    directions = [(0,1), (0,-1), (1,0), (-1,0)]  #right,left,down,up
    for dr, dc in directions:
        nr, nc = r + dr, c + dc
        if 0 <= nr < len(grid_map) and 0 <= nc < len(grid_map[0]) and grid_map[nr][nc] == 0: #check if the neighbours are valid and not occupied
            neighbors.append((nr, nc))
    return neighbors

#store each cell's parent and trace back the path from goal to start point
def reconstruct_path(parent, end):
    path = [end]
    while end in parent:
        end = parent[end]
        path.append(end)
    return list(reversed(path))

#A* algorithm implementation
def astar(start_point, goal_point, grid_map):
    open_list = []                 #Min heap priority queue sorted by f=g+h
    heapq.heappush(open_list, (0, start_point))

    g_score = {start_point: 0}           #distance from start i.e g
    parent = {}                  

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal_point:        #if current cell is goal_point
            return reconstruct_path(parent, current)

        else:                            #otherwise check the neighbours
            for neighbor in get_neighbors(current, grid_map):    
                tentative_g = g_score[current] + 1   #assuming cost of movement between cells is 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:   #cheaper path found
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal_point)  #f = g+h
                    heapq.heappush(open_list, (f, neighbor))
                    parent[neighbor] = current

    return None  #No path found

#visualise the found path using A* search
def visualize(grid, path):
    visual = [row[:] for row in grid]

    if path is not None:
        for (r, c) in path:
            visual[r][c] = '-' 

    print("\nVisual Output")
    for row in visual:
        print(" ".join(str(x) for x in row))
    print()



if __name__ == "__main__":
    grid_map = [
        [0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,1,1,0],
        [0,1,0,0,0,0,0,0,1,0],
        [0,1,0,1,1,1,1,0,1,0],
        [0,1,0,1,0,0,1,0,1,0],
        [0,1,0,1,0,1,1,0,1,0],
        [0,1,0,1,0,0,0,0,1,0],
        [0,1,0,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,1,1,0]
    ]

    start_point = (0, 0)
    goal_point = (2, 7)

    result_path = astar(start_point, goal_point, grid_map)

    print("Map:")
    for row in grid_map:
        print(row)

    print("\nStart:", start_point)
    print("Goal: ", goal_point)

    print("Path:", result_path)
    visualize(grid_map, result_path)

