from solver import Node, Agent, Map, Map2, construct_wait
from multi_waypoint import MLA
from mapfw import MapfwBenchmarker
import timeit
from heapq import *
import sys
def hbh(maze, agents):
    paths = []
    reserved = dict()
    resting = []
    for start in map(lambda agent: agent.start, agents):
        reserved[start] = [0]
        resting.append(start)
    for end in map(lambda agent: agent.end, agents):
        resting.append(end)
    for agent in agents:
        resting = resting[1:]
        nodes = dynamicMLA(maze, agent, reserved, 0, resting)

        # Adding reservations twice to ensure that no agent walks into another agent, nor that two agents convergence onto the same vertex.
        for node in nodes[1:]:
            if node.position in reserved:
                reserved[node.position].append(node.g)
            else:
                reserved[node.position] = [node.g, node.g - 1]
        paths.append(nodes)
    return paths

def dynamicMLA(maze, agent, reserved, start_wait = 0, resting = []):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # copy waypoints
    waypoints = agent.waypoints.copy()
    # Initialize both open and closed list
    open_list = []
    closed_list = set()
    # Last label belongs to end node and is equal to number of waypoints + 1. Eg. 2 for an agent with 1 waypoint.
    start_node = Node(None, agent.start)
    start_node.g = start_node.h = start_node.f = 0
    if start_wait > 0:
        wait = construct_wait(start_node, 0, start_wait)
        if len(agent.waypoints) == 0:
            wait.l = 2
        open_list.append(wait)
    else:
        # Add the start node
        if len(agent.waypoints) == 0:
            start_node.l = 2
        open_list.append(start_node)
    # If we have waypoints, create waypoint node and set it to be the first waypoint.
    if len(agent.waypoints) > 0:
        waypoint = Node(None, select_waypoint(agent.start, waypoints))
        waypoint.g = waypoint.h = waypoint.f = 0
    else:
        waypoint = Node(None, (-1, -1))

    # mid_node = Node(None, agent.waypoints[0])
    # mid_node.g = mid_node.h = mid_node.f = 0
    end_node = Node(None, agent.end)
    end_node.g = end_node.h = end_node.f = 0



    # Loop until you find the end
    while len(open_list) > 0:
        # Get the current node
        current_node = heappop(open_list)
        closed_list.add(current_node)
        if current_node.position != agent.end and current_node.position in resting:
            if len(open_list) > 0:
                continue
        # TODO Do more efficiently. Checks if this space is reserved to avoid swap conflicts.
        if current_node.position in reserved and current_node.g + 1 in reserved[current_node.position]:
            continue
        # Found the waypoint
        if current_node.l == 1 and current_node == waypoint:
            # construct new search node.
            n_prime = Node(current_node.parent, current_node.position)
            n_prime.g = current_node.g
            # Waypoint is now the next waypoint.

            if len(waypoints) > 0:
                waypoint = Node(None, select_waypoint(current_node.position, waypoints))
                n_prime.h = abs(current_node.position[0] - waypoint.position[0]) + abs(
                    current_node.position[1] - waypoint.position[1])
            else:
                n_prime.l = 2
                waypoint = Node(None, (-1, -1))
                n_prime.h = abs(current_node.position[0] - end_node.position[0]) + abs(
                    current_node.position[1] - end_node.position[1])
            # N_prime is the new search node, which has the same position as the current node, but with new label and new h.



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
            return path[::-1] # Return reversed path
        # Loop through children
        # for child in map(lambda x: Node(current_node, x),
        #                  maze.get_neighbours(current_node.position[0], current_node.position[1])):
        #     if child.position in reserved and current_node.g + 1 in reserved[child.position]:
        #         collision = True
        #         # Check if we can wait here. If not we dont consider this step.
        #         if current_node.position in reserved and current_node.g + 1 in reserved[child.position]:
        #             break
        #         # We can wait, so add a node to wait and then move into that position.
        #         elif child.position in reserved and current_node.g + 2 in reserved[child.position]:
        #             break
        #         else:
        #             wait = Node(current_node, current_node.position)
        #             wait.g = current_node.g + 1
        #             wait_move = Node(wait, child.position)
        #             wait_move.g = current_node.g + 2
        #             if child.l == 1:
        #                 child.h = abs(current_node.position[0] - waypoint.position[0]) + abs(
        #                     current_node.position[1] - waypoint.position[1])
        #             else:
        #                 child.h = abs(child.position[0] - end_node.position[0]) + abs(
        #                     child.position[1] - end_node.position[1])
        #             wait_move.f = wait_move.g + wait_move.f
        #             heappush(open_list, wait_move)
        for child in map(lambda x: Node(current_node, x), maze.get_neighbours(current_node.position[0], current_node.position[1])):

            # TODO USE HASHSET
            if child.position in reserved and current_node.g + 1 in reserved[child.position]:
                # Check if we can wait here. If not we dont consider this step.
                if current_node.position in reserved and current_node.g + 1 in reserved[child.position]:
                    break
                # We can wait, so add a node to wait and then move into that position.
                elif child.position in reserved and current_node.g + 2 in reserved[child.position]:
                    break
                else:
                    wait = Node(current_node, current_node.position)
                    wait.g = current_node.g + 1
                    wait_move = Node(wait, child.position)
                    wait_move.g = current_node.g + 2
                    if child.l == 1:
                        child.h = abs(current_node.position[0] - waypoint.position[0]) + abs(
                            current_node.position[1] - waypoint.position[1])
                    else:
                        child.h = abs(child.position[0] - end_node.position[0]) + abs(
                            child.position[1] - end_node.position[1])
                    wait_move.f = wait_move.g + wait_move.f
                    heappush(open_list, wait_move)
                continue
            # In case of collision
            # Child is on the closed list
            if child in closed_list:
                continue

            if child in open_list:
                continue
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.l = current_node.l
            if child.l == 1:
                child.h = abs(current_node.position[0] - waypoint.position[0]) + abs(current_node.position[1] - waypoint.position[1])
            else:
                child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            # Add the child to the open list
            heappush(open_list, child)
    if start_wait == 0:
        return dynamicMLA(maze, agent, reserved, 1, resting)
    else:
        return dynamicMLA(maze, agent, reserved, start_wait + 1, resting)

def select_waypoint(current_position, waypoints):
    w = waypoints[0]
    if len(waypoints) == 1:
        waypoints.remove(w)
        return w
    min_h = sys.maxsize
    min_index = -1
    for i, waypoint in enumerate(waypoints):
        dist = abs(current_position[0] - waypoint[0]) + abs(current_position[1] - waypoint[1])
        h = dist
        # nearest_dist = sys.maxsize
        # for w in waypoints:
        #     if w == waypoint:
        #         continue
        #     dist_waypoints = abs(waypoint[0] - w[0]) + abs(waypoint[1] - w[1])
        #     if dist_waypoints < nearest_dist:
        #         nearest_dist = dist_waypoints
        # # found nearest node and computed distance to node.
        # h = dist + dist_waypoints
        # if h is smaller than the smallest so far, update h and remember index.
        if h < min_h:
            min_h = h
            min_index = i
    w = waypoints[min_index]
    waypoints.remove(w)
    return w

def preordering(maze, agents):
    paths = []
    reserved = dict()
    resting = []
    for start in map(lambda agent: agent.start, agents):
        reserved[start] = [0]
        resting.append(start)
    for end in map(lambda agent: agent.end, agents):
        resting.append(end)
    for agent in agents:
        resting = resting[1:]
        nodes = MLA(maze, agent, reserved, 0, resting)
        # Adding reservations twice to ensure that no agent walks into another agent, nor that two agents convergence onto the same vertex.
        for node in nodes[1:]:
            if node.position in reserved:
                reserved[node.position].append(node.g)
            else:
                reserved[node.position] = [node.g, node.g - 1]
        paths.append(nodes)
    return paths

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
        agents.append(
            Agent((problem.starts[i][0], problem.starts[i][1]), (problem.goals[i][0], problem.goals[i][1]),
                  waypoints[i]))
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

if __name__ == "__main__":
    benchmarker = MapfwBenchmarker("<INSERT API TOKEN>", <INSERT BENCHMARK HERE>, "<INSERT ALGORITHM NAME>",
                                   "<INSERT DESCRIPTION>", <TRUE OR FALSE>, solve, 1)
    benchmarker.run()
