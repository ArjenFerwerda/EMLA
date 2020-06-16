from solver import Node, Agent, Map, Map2, construct_wait
import timeit
from heapq import *


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
        nodes = MLA(maze, agent, reserved, 0, resting)
        print(agent)
        # Adding reservations twice to ensure that no agent walks into another agent, nor that two agents convergence onto the same vertex.
        for node in nodes[1:]:
            if node.position in reserved:
                reserved[node.position].append(node.g)
            else:
                reserved[node.position] = [node.g, node.g - 1]
        paths.append(nodes)
    return paths

def MLA(maze, agent, reserved, start_wait = 0, resting = []):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Initialize both open and closed list
    open_list = []
    closed_list = set()
    print(start_wait)
    # Last label belongs to end node and is equal to number of waypoints + 1. Eg. 2 for an agent with 1 waypoint.
    final_label = len(agent.waypoints)+1
    waypoint_label = 1
    start_node = Node(None, agent.start)
    start_node.g = start_node.h = start_node.f = 0
    if start_wait > 0:
        open_list.append(construct_wait(start_node, 0, start_wait-1))
    else:
        # Add the start node
        open_list.append(start_node)
    # If we have waypoints, create waypoint node and set it to be the first waypoint.
    if final_label > 1:
        waypoint = Node(None, agent.waypoints[0])
        waypoint.g = waypoint.h = waypoint.f = 0

    # Else we have no waypoints and set the label to -1.
    else:
        waypoint_label = -1

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
        if current_node.l == waypoint_label and current_node == waypoint:
            # Waypoint is now the next waypoint.
            if current_node.l < len(agent.waypoints):
                waypoint = Node(None, agent.waypoints[current_node.l])
            else:
                waypoint = Node(None, (-1, -1))
            # N_prime is the new search node, which has the same position as the current node, but with new label and new h.
            n_prime = Node(current_node.parent, current_node.position)
            n_prime.g = current_node.g

            n_prime.h = compute_h(current_node.l-1, agent.waypoints, end_node)

            n_prime.f = n_prime.g + n_prime.h
            n_prime.l = waypoint_label + 1

            waypoint_label += 1
            closed_list = set()
            open_list = []
            heappush(open_list, n_prime)
            continue
        if current_node.l == final_label and current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1] # Return reversed path
        # Loop through children
        collision = False
        for child in map(lambda x: Node(current_node, x),
                         maze.get_neighbours(current_node.position[0], current_node.position[1])):
            if child.position in reserved and current_node.g + 1 in reserved[child.position]:
                collision = True
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
                    if child.l != final_label:
                        wait_move.h = abs(child.position[0] - waypoint.position[0]) + abs(
                            child.position[1] - waypoint.position[1]) + compute_h(child.l - 1, agent.waypoints,
                                                                                  end_node)
                    else:
                        wait_move.h = abs(child.position[0] - end_node.position[0]) + abs(
                            child.position[1] - end_node.position[1])
                    wait_move.f = wait_move.g + wait_move.f
                    heappush(open_list, wait_move)
        for child in map(lambda x: Node(current_node, x), maze.get_neighbours(current_node.position[0], current_node.position[1])):

            # TODO USE HASHSET
            if child.position in reserved and current_node.g + 1 in reserved[child.position]:
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
            if child.l != final_label:
                child.h = abs(child.position[0] - waypoint.position[0]) + abs(
                    child.position[1] - waypoint.position[1]) + compute_h(child.l - 1, agent.waypoints, end_node)
            else:
                child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            # Add the child to the open list
            heappush(open_list, child)
    if start_wait == 0:
        return MLA(maze, agent, reserved, 1, resting)
    else:
        return MLA(maze, agent, reserved, start_wait * 2, resting)

def compute_h(current_waypoint_index, waypoints, end_node):
    #Computes the h value for a specific waypoint for an ordered set of waypoints. It equals the dist of each pair of subsequent waypoints, as well as the dist between the last waypoint and the end_node.
    dist = abs(waypoints[-1][0] - end_node.position[0]) + abs(waypoints[-1][1] - end_node.position[1])
    for i, location in enumerate(waypoints[current_waypoint_index:-1], 1):
        dist += abs(location[0] - waypoints[current_waypoint_index + i][0]) + abs(location[1] - waypoints[current_waypoint_index + i][1])
    return dist

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
        agents.append(Agent((problem.starts[i][0], problem.starts[i][1]), (problem.goals[i][0], problem.goals[i][1]), waypoints[i]))
    maze = Map2(problem.grid, problem.width, problem.height)
    for i in agents:
        for j in agents:
            if i.start == j.end:
                print(i, j)
    start = timeit.default_timer()

    moves = hbh(maze, agents)

    stop = timeit.default_timer()
    print('Time: ', (stop - start)*1000, "ms")

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
