import numpy as np
import heapq
import os
import time

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class Maze:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = np.zeros((width, height), dtype=bool)

    def add_wall(self, x, y):
        self.walls[x][y] = True

    def is_valid(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and not self.walls[x][y]

    def neighbors(self, x, y):
        possible_neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        valid_neighbors = [(nx, ny) for nx, ny in possible_neighbors if self.is_valid(nx, ny)]
        return valid_neighbors

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def ucs(maze, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in maze.neighbors(*current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

def a_star(maze, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in maze.neighbors(*current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

def print_maze(maze, pacman_position):
    os.system('cls' if os.name == 'nt' else 'clear')
    for y in range(maze.height):
        for x in range(maze.width):
            if (x, y) == pacman_position:
                print('P', end='')
            elif maze.walls[x][y]:
                print('#', end='')
            else:
                print('.', end='')
        print()

def main():
    maze = Maze(10, 10)
    # Add walls as needed
    maze.add_wall(1, 1)
    maze.add_wall(2, 2)
    maze.add_wall(3, 3)
    maze.add_wall(4, 4)
    maze.add_wall(5, 5)
    maze.add_wall(6, 6)
    maze.add_wall(7, 7)
    maze.add_wall(8, 8)
    
    start = (0, 0)
    goals = [(0, 9), (9, 0), (9, 9), (0, 9)]

    for goal in goals:
        came_from, _ = ucs(maze, start, goal)
        path = reconstruct_path(came_from, start, goal)
        for step in path:
            print_maze(maze, step)
            time.sleep(0.5)

    for goal in goals:
        came_from, _ = a_star(maze, start, goal)
        path = reconstruct_path(came_from, start, goal)
        for step in path:
            print_maze(maze, step)
            time.sleep(0.5)

if __name__ == "__main__":
    main()
