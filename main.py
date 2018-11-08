import numpy as np
import matplotlib.pyplot as plt
import heapq as heap
import GUI
import threading

def draw(input_path):
    maze = []
    grid = {}
    with open(input_path,'r') as f:
        size = list(map(int,f.readline().rstrip().split()))
        size_x, size_y = size[0],size[1]
        start = tuple(map(int,f.readline().rstrip().split()))
        goal = tuple(map(int,f.readline().rstrip().split()))
        for _ in range(size_x):
            maze.append(list(map(int,f.readline().rstrip().split())))

        for i in range(size_x):
            for j in range(size_y):
                grid[(i,j)] = {'visited':False, 'dist':np.inf, 'valid':True, 'F':np.inf}
            #Define obstacles in the grid, if 'valid' == False -> this is an obstacle
                if maze[i][j] == 1:
                    grid[(i,j)]['valid'] = False

        # print(grid)
        return grid,size,start,goal,maze

def isValid(cell, grid, size = [7,7]):
    # isValid determines if cells are valid for the given grid
    if cell[0] not in range(size[0]) or cell[1] not in range(size[1]):
        return False

    for key in grid.keys():
        if cell == key:
            if not grid[key]['valid']:
                return False
            break

    return True

def H(cell, target):
    # Function H represents the Heuristic value for A*
    # The Heuristic Value = distance from a given node (or cell) to target
    x_dist = cell[0] - target[0]
    y_dist = cell[1] - target[1]
    diag_dist = np.sqrt((x_dist) ** 2 + (y_dist) ** 2)

    return diag_dist

def aStarAlg(grid, start, end, size):
    # where grid is given in Q2, start is the start, end is at finish

    q = []

    # Direction of new path
    direct = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    # Heap with cells(x,y) and distance from the start
    heap.heappush(q, (0, start))

    grid[start]['dist'] = 0
    # Where F is the total distance (distance travelled + heuristic value)
    grid[start]['F'] = 0

    while q != []:
        v = heap.heappop(q)
        cell = v[1]

        if cell == end:
            break

        # Identify nodes visited
        plt.scatter(cell[0], cell[1], marker='*', color='.75')

        # The code above is almost identical to Q2, below I update the code for Q3
        for direction in direct:
            new_cell = (direction[0] + cell[0], direction[1] + cell[1])
            if isValid(new_cell, grid, size):
                if direction in [(1, 1), (-1, 1), (1, -1), (-1, -1)]:
                    dist = grid[cell]['dist'] + np.sqrt(2)
                    F = dist + H(new_cell, end)
                else:
                    dist = grid[cell]['dist'] + 1
                    F = dist + H(new_cell, end)
                if F < grid[new_cell]['F']:
                    grid[new_cell]['parent'] = cell
                    grid[new_cell]['dist'] = dist
                    grid[new_cell]['F'] = F
                if grid[new_cell]['visited'] != True and cell != new_cell:
                    grid[new_cell]['visited'] = True
                    heap.heappush(q, (grid[new_cell]['F'], new_cell))

    # To find shortest path - backtrack through parents of nodes
    itr = end
    p = [end]
    shortest_p = grid[end]['dist']

    while itr != start:
        itr = grid[itr]['parent']
        p.append(itr)

    print('Distance of the shortest path is %f' % (shortest_p))
    return shortest_p, p

def main(input,output):
    grid, size, start, goal, maze = draw(input)
    # Create cells
    cells = []
    for key in grid.keys():
        if grid[key]['valid'] != True:
            cells.append(key)

    x_cells = []
    for keys in cells:
        x_cells.append(keys[0])

    y_cells = []
    for keys in cells:
        y_cells.append(keys[1])

    # Plot grid
    plt.axis([0, size[0], 0, size[1]])
    plt.scatter(x_cells, y_cells, color='magenta')

    # Initalize start and end points (from diagram in 2)
    start = (start[0],start[1])
    end = (goal[0],goal[1])

    shortest_p, path = aStarAlg(grid, start, end, size)
    print(path)
    x_cells = [itr[0] for itr in path]
    y_cells = [itr[1] for itr in path]

    # Plot path
    plt.plot(x_cells, y_cells, color='cyan')

    # Plot start and end points
    plt.plot(start[0],start[1] , marker='o', color='green')
    plt.plot(goal[0],goal[1], marker='o', color='red')

    plt.title("A* Algorithm")
    plt.show()
    for i in range(size[0]):
        for j in range(size[1]):
            if [i,j] == [start[0], start[1]]:
                maze[i][j] = 'S'
            elif [i,j] == [goal[0],goal[1]]:
                maze[i][j] = 'G'
            elif (i,j) in path:
                maze[i][j] = 'x'
            elif grid[(i,j)]['valid'] == False:
                maze[i][j] = 'o'
            else:
                maze[i][j] = '-'
    print("MAZE:", maze)

    with open(output,'w') as fw:
        fw.writelines(str(len(path)) + "  #  the number of steps to reach the goal.\n")
        for point in path[::-1]:
            fw.write(str(point),)
        fw.write("\n#the position of each step in the optimal path.")
        fw.write("\n#the map with the optimal path.\n")
        for i in range(size[0]):
            for j in range(size[1]):
                fw.write(maze[i][j])
                fw.write("   ")
            fw.write('\n')

    for i in range(size[0]):
        for j in range(size[1]):
            if grid[(i,j)]['valid'] == False:
                GUI.Walls.append((i,j))


if __name__ == '__main__':
    main("input.txt","output.txt")
    # main("input1.txt","output1.txt")


