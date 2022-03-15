# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None  # cost to come (previous g + moving cost)
        self.h = h  # heuristic
        self.cost = None  # total cost (depend on the algorithm)
        self.parent = None  # previous node

def path_finder_old(child, parent, goal, start):
    a = goal
    n = 0
    path = []
    while True:
        path.append(a)
        if a == start:
            break
        for i in child:
            if i == a:
                a = parent[n]
                n = 0
                break
            n = n + 1

    path.reverse()
    return path

def path_finder(parent_grid, goal, start):
    a = goal
    path = []
    while True:
        path.append(a)
        if a == start:
            break
        a = parent_grid[a[0]][a[1]]

    path.reverse()
    return path

def min_cost(cost):

    n = 0
    for i in cost:
        if i == min(cost):
            break
        n = n + 1

    return n

def manh(current, goal):

    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    steps = 0
    q = []
    explored = []
    parent_grid = [[[] for x in range(len(grid[0]))] for y in range(len(grid))]
    found = False
    q.append(start)

    #Searching Algorithm
    while len(q) != 0:
        u = q.pop(0)
        explored.append(u)
        steps = steps + 1

        if u == goal:
            found = True
            break

        sequence = [[u[0], u[1] + 1], [u[0] + 1, u[1]], [u[0], u[1] - 1], [u[0] - 1, u[1]]]

        for s in sequence:
            if (0 <= s[0] < len(grid)) and (0 <= s[1] < len(grid[0])):
                if grid[s[0]][s[1]] == 0 and s not in explored and s not in q:
                    q.append(s)
                    parent_grid[s[0]][s[1]] = u


    path = path_finder(parent_grid, goal, start)
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")

    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    steps = 0
    stack = []
    explored = []
    parent_grid = [[[] for x in range(len(grid[0]))] for y in range(len(grid))]
    found = False
    stack.append(start)

    # Searching Algorithm
    while len(stack) != 0:
        u = stack.pop()
        explored.append(u)
        steps = steps + 1

        if u == goal:
            found = True
            break

        sequence = [[u[0] - 1, u[1]], [u[0], u[1] - 1], [u[0] + 1, u[1]], [u[0], u[1] + 1]]

        for s in sequence:
            if (0 <= s[0] < len(grid)) and (0 <= s[1] < len(grid[0])):
                if grid[s[0]][s[1]] == 0 and s not in explored:
                    if s not in stack:
                        stack.append(s)
                        parent_grid[s[0]][s[1]] = u

                    else:
                        parent_grid[s[0]][s[1]] = u


    # Finding the Path
    path = path_finder(parent_grid, goal, start)

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    steps = 0
    q = []
    cost = []
    explored = []
    parent_grid = [[[] for x in range(len(grid[0]))] for y in range(len(grid))]
    found = False
    q.append(start)
    cost.append(0)

    # Searching Algorithm
    while len(q) != 0:
        p = min_cost(cost)
        u = q.pop(p)
        c = cost.pop(p)
        explored.append(u)
        steps = steps + 1

        if u == goal:
            found = True
            break

        sequence = [[u[0], u[1] + 1], [u[0] + 1, u[1]], [u[0], u[1] - 1], [u[0] - 1, u[1]]]

        for s in sequence:
            if (0 <= s[0] < len(grid)) and (0 <= s[1] < len(grid[0])):
                if grid[s[0]][s[1]] == 0 and s not in explored:
                    if s not in q:
                        q.append(s)
                        cost.append(c+1)
                        parent_grid[s[0]][s[1]] = u

                    else:
                        n = 0
                        for i in q:
                            if i == s:
                                if cost[n] > c+1:
                                    cost[n] = c+1
                                    parent_grid[s[0]][s[1]] = u
                                    break
                            n = n + 1

    path = path_finder(parent_grid, goal, start)



    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    steps = 0
    q = []
    cost = []
    explored = []
    parent_grid = [[[] for x in range(len(grid[0]))] for y in range(len(grid))]
    found = False
    q.append(start)
    cost.append(0 + manh(start, goal))

    # Searching Algorithm
    while len(q) != 0:
        p = min_cost(cost)
        u = q.pop(p)
        c = cost.pop(p)
        explored.append(u)
        steps = steps + 1

        if u == goal:
            found = True
            break

        sequence = [[u[0], u[1] + 1], [u[0] + 1, u[1]], [u[0], u[1] - 1], [u[0] - 1, u[1]]]

        for s in sequence:
            if (0 <= s[0] < len(grid)) and (0 <= s[1] < len(grid[0])):
                if grid[s[0]][s[1]] == 0 and s not in explored:
                    if s not in q:
                        q.append(s)
                        cost.append(c + 1 - manh(u, goal) + manh(s, goal))
                        parent_grid[s[0]][s[1]] = u

                    else:
                        n = 0
                        for i in q:
                            if i == s:
                                if cost[n] > (c + 1 - manh(u, goal) + manh(s, goal)):
                                    cost[n] = (c + 1 - manh(u, goal) + manh(s, goal))
                                    parent_grid[s[0]][s[1]] = u
                                    break

                            n = n + 1

    # Finding the path
    path = path_finder(parent_grid, goal, start)

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples

    # Test all the functions
    testmod()
