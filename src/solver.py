import os
import sys
import timeit
from heapq import *

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import traceback


class Solver:
    def solve(maze, agents):
        hbh(maze, agents)


def solve(problem):
    number_of_agents = len(problem.starts)
    agents = []
    waypoints = []

    for waypoint_list in problem.waypoints:
        sub_list = []
        for waypoint in waypoint_list:
            sub_list.append((waypoint[0], waypoint[1]))
        waypoints.append(sub_list)
    for i in range(number_of_agents):
        agents.append(Agent((problem.starts[i][0], problem.starts[i][1]), (problem.goals[i][0], problem.goals[i][1]),
                            waypoints[i]))

    print(problem)
    maze = Map2(problem.grid, problem.width, problem.height)

    start = timeit.default_timer()

    moves = hbh(maze, agents)

    stop = timeit.default_timer()
    print('Time: ', (stop - start) * 1000, "ms")

    paths = []
    for agent_moves in moves:
        sub_path = []
        for node in agent_moves:
            sub_path.append([node.position[0], node.position[1]])
        paths.append(sub_path)
    """
    Now paths looks like:

    paths = [path agent 1, path agent 2, ..]
    path agent 1 = [pos agent 1 at time 0, pos agent 1 at time 1, .., pos agent 1 at finishing time]
    pos = [x coordinate, y coordinate]
    """

    return paths


class Agent:
    def __init__(self, start, end, waypoints=[]):
        self.start = start
        self.end = end
        self.waypoints = waypoints

    def __str__(self):
        return str(self.start) + " " + str(self.end)

    @staticmethod
    def create_agents(file_path):
        try:
            f = open(file_path, "r")
            lines = f.read().splitlines()

            # make each agent
            agents = []
            for line in lines:
                points = line.split(",")
                if len(points) < 3:
                    points = map(lambda x: x.split(" "), points)
                    coordinates = []
                    for point in points:
                        coordinates.append((int(point[0]), int(point[1])))
                    agents.append(Agent(coordinates[0], coordinates[1]))
                else:
                    points = map(lambda x: x.split(" "), points)
                    coordinates = []
                    for point in points:
                        coordinates.append((int(point[0]), int(point[1])))
                    agents.append(Agent(coordinates[0], coordinates[-1], coordinates[1:-1]))
            print("Ready reading agent file " + file_path)
            return agents
        except FileNotFoundError:
            print("Error reading agent file " + file_path)
            traceback.print_exc()
            sys.exit()


# Class that holds all the Map data.
class Map:
    # Constructor of a Map
    # @param walls int array of tiles accessible (1) and non-accessible (0)
    # @param width width of Map (horizontal)
    # @param length length of Map (vertical)
    def __init__(self, walls, width, length):
        self.walls = walls
        self.length = length
        self.width = width
        self.start = None
        self.end = None

    # Width getter
    # @return width of the Map
    def get_width(self):
        return self.width

    # Length getter
    # @return length of the Map
    def get_length(self):
        return self.length

    # Returns a list of tuples of valid neighbouring positions. A position is valid if it is inbounds and not a wall.
    def get_neighbours(self, x, y):
        positions = []
        # Check north
        if self.valid_position(x, y - 1):
            positions.append((x, y - 1))
        # Check East
        if self.valid_position(x + 1, y):
            positions.append((x + 1, y))
        # Check South
        if self.valid_position(x, y + 1):
            positions.append((x, y + 1))
        # Check West
        if self.valid_position(x - 1, y):
            positions.append((x - 1, y))
        return positions

    def valid_position(self, x, y):
        return self.width - 1 >= x >= 0 != self.walls[x][y] and 0 <= y <= self.length - 1

    # Check whether a coordinate lies in the current Map.
    # @param position The position to be checked
    # @return Whether the position is in the current Map
    # def in_bounds(self, position):
    #     return position.x_between(0, self.width) and position.y_between(0, self.length)

    # Representation of Map as defined by the input file format.
    # @return String representation
    def __str__(self):
        string = ""
        string += str(self.width)
        string += " "
        string += str(self.length)
        string += " \n"
        for y in range(self.length):
            for x in range(self.width):
                string += str(self.walls[x][y])
                string += " "
            string += "\n"
        return string

    # Method that builds a maze from a file
    # @param filePath Path to the file
    # @return A Map object with pheromones initialized to 0's inaccessible and 1's accessible.
    @staticmethod
    def create_map(file_path):
        try:
            f = open(file_path, "r")
            lines = f.read().splitlines()
            dimensions = lines[0].split(" ")
            width = int(dimensions[0])
            length = int(dimensions[1])

            # make the map_layout
            map_layout = []
            for x in range(width):
                map_layout.append([])

            for y in range(length):
                line = lines[y + 1].split(" ")
                for x in range(width):
                    if line[x] != "":
                        state = int(line[x])
                        map_layout[x].append(state)
            print("Ready reading map file " + file_path)
            return Map(map_layout, width, length)
        except FileNotFoundError:
            print("Error reading map file " + file_path)
            traceback.print_exc()
            sys.exit()


class Map2(Map):
    def valid_position(self, x, y):
        return 0 <= x <= self.width - 1 and 0 <= y <= self.length - 1 and self.walls[y][x] != 1


class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
        self.l = 1

    def __eq__(self, other):
        return self.position == other.position

    def __str__(self):
        return "(" + str(self.position) + ", " + str(self.g) + ")"

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash(self.position)


def astar(maze, agent, reserved):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Create start and end node
    start_node = Node(None, agent.start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, agent.end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        # TODO Make priority queue
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        print("Current node: ", current_node.position, ", ", current_node.g)
        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]  # Return reversed path
        # TODO Redo collisions to use a 2d array of positions, and then a list of times for that position.
        collision = False
        for child in map(lambda x: Node(current_node, x),
                         maze.get_neighbours(current_node.position[0], current_node.position[1])):
            if child.position in reserved and current_node.g + 1 in reserved[child.position]:
                collision = True
                # Check if we can wait here.
                if current_node.position in reserved and current_node.g + 1 in reserved[child.position]:
                    closed_list.append(current_node)
                # We can wait, so add a node to wait and then move into that position.
                else:
                    wait = Node(current_node, current_node.position)
                    wait.g = current_node.g + 1
                    wait_move = Node(wait, child.position)
                    wait_move.g = current_node.g + 2
                    wait_move.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                                (child.position[1] - end_node.position[1]) ** 2)
                    wait_move.f = wait_move.g + wait_move.f
                    open_list.append(wait_move)
        if not collision:
            closed_list.append(current_node)
        # Loop through children
        for child in map(lambda x: Node(current_node, x),
                         maze.get_neighbours(current_node.position[0], current_node.position[1])):
            print(child.position)
            # TODO DEAL WITH COLLISIONS.
            if child.position in reserved and current_node.g + 1 in reserved[child.position]:
                continue
            # In case of collision
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            # TODO Compute proper f, g, and h values
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                        (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g >= open_node.g:
                    continue
            # Add the child to the open list
            open_list.append(child)
    print("NO PATH")


def MLA(maze, agent, reserved):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Create start and end node and mid node
    start_node = Node(None, agent.start)
    start_node.g = start_node.h = start_node.f = 0
    mid_node = Node(None, agent.waypoints[0])
    mid_node.g = mid_node.h = mid_node.f = 0
    end_node = Node(None, agent.end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = set()

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        print(len(open_list))
        # Get the current node
        current_node = heappop(open_list)

        closed_list.add(current_node)
        print("Current node: ", current_node.position, ", ", current_node.g)
        # Found the goal
        if current_node.l == 1 and current_node == mid_node:
            n_prime = Node(current_node.parent, current_node.position)
            n_prime.l = 2
            n_prime.g = current_node.g
            n_prime.h = abs(n_prime.position[0] - end_node.position[0]) + abs(
                n_prime.position[1] - end_node.position[1])
            n_prime.f = n_prime.g + n_prime.h
            closed_list = set()
            open_list = []
            heappush(open_list, n_prime)
            continue
        if current_node.l == 2 and current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]  # Return reversed path
        # Loop through children
        for child in map(lambda x: Node(current_node, x),
                         maze.get_neighbours(current_node.position[0], current_node.position[1])):
            if child.position in reserved and current_node.g + 1 in reserved[child.position]:
                # Check if we can wait here.
                if current_node.position in reserved and current_node.g + 1 in reserved[child.position]:
                    closed_list.add(current_node)
                # We can wait, so add a node to wait and then move into that position.
                else:
                    wait = Node(current_node, current_node.position)
                    wait.g = current_node.g + 1
                    wait_move = Node(wait, child.position)
                    wait_move.g = current_node.g + 2
                    wait_move.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                            (child.position[1] - end_node.position[1]) ** 2)
                    wait_move.f = wait_move.g + wait_move.f
                    heappush(open_list, wait_move)
        for child in map(lambda x: Node(current_node, x),
                         maze.get_neighbours(current_node.position[0], current_node.position[1])):
            print(child.position)
            if child.position in reserved and current_node.g + 1 in reserved[child.position]:
                continue
            # In case of collision
            # Child is on the closed list
            if child in closed_list:
                continue
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.l = current_node.l
            if child.l == 1:
                child.h = abs(child.position[0] - mid_node.position[0]) + abs(
                    child.position[1] - mid_node.position[1]) + abs(mid_node.position[0] - end_node.position[0]) + abs(
                    mid_node.position[1] - end_node.position[1])
            else:
                child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h
            heappush(open_list, child)
    print("NO PATH")


def construct_wait(node, start, end):
    wait_node = node
    while start <= end:
        g = wait_node.g
        wait_node = Node(wait_node, wait_node.position)
        wait_node.g = g + 1
        start += 1
    return wait_node


def hbh(maze, agents):
    paths = []
    reserved = dict()

    for start in map(lambda agent: agent.start, agents):
        reserved[start] = [0]
    for agent in agents:
        nodes = MLA(maze, agent, reserved)
        # Adding reservations twice to ensure that no agent walks into another agent, nor that two agents convergence onto the same vertex.
        for node in nodes[1:]:
            if node.position in reserved:
                reserved[node.position].append(node.g)
            else:
                reserved[node.position] = [node.g, node.g - 1]
        paths.append(nodes)
    return paths
