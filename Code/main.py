import cv2
import argparse
import os
from queue import PriorityQueue
import map_generator as map_gen
import utils as ut
import animate_searching as anime


Action_sets = {(1, 0): 1.0, (0, 1): 1.0,
               (-1, 0): 1.0, (0, -1): 1.0,
               (1, 1): 1.4, (-1, 1): 1.4,
               (-1, -1): 1.4, (1, -1): 1.4}


class Node:
    def __init__(self, node_state, parent_node=None, cost_to_come=0):
        self.node_state = node_state
        self.parent_node = parent_node
        self.cost_to_come = cost_to_come


def get_next_nodes(curr_node, world_map, clearance):

    # get height and width of the world map
    h, w = world_map.shape

    # get the current node location
    location = curr_node.node_state

    # extract (x, y) from location
    x, y = location[0], location[1]

    # get current node cost to come
    cost_to_come = curr_node.cost_to_come

    # initialize an empty list to store the next possible nodes
    next_nodes = []

    # iterate through the Action set
    for key, value in Action_sets.items():

        # check for boundary of the world map and modify the next possible locations accordingly
        next_x, next_y = ut.shift_inside_boundary([x + key[0], y + key[1]], [w, h], clearance, clearance)

        # calculate the next node cost to come with respect to current node
        next_cost_to_come = value + cost_to_come

        # create a new Node object
        next_node = Node((next_x, next_y), curr_node, next_cost_to_come)

        # append the new Node object to the list
        next_nodes.append(next_node)

    # return the list containing next possible nodes with respect to the current node
    return next_nodes


def reached_goal_node(current_location, end_location):

    # check if current node location is equal to end location
    if current_location[0] == end_location[0] and current_location[1] == end_location[1]:
        print("[INFO]: Reached Goal Node")
        return True
    return False


def present_in_list(dic, current_node):

    # check if the current node is present in the dictionary
    if current_node.node_state in dic.keys():
        return True
    return False


def start_dijkstra(world_map, start_location, end_location, clearance):

    # create initial node object and set it to start location and parent Node to None
    inital_node = Node(start_location, None, 0)

    # create open list dictionary to store the nodes
    open_list = {}

    # create closed list dictionary to store the explored nodes
    closed_list = {}

    # create a priority queue object
    pq = PriorityQueue()

    pq.put([inital_node.cost_to_come, inital_node.node_state, inital_node])

    # append the initial node to the open list dictionary
    open_list[start_location] = [inital_node.cost_to_come, inital_node]

    # iterate through open list dictionary till its size is not 0
    while len(open_list):

        # pop a new node with lowest C2C from the open list
        # cur_node = pop_lowest_c2c_node(open_list)
        cur_node = pq.get()[2]

        # put current node in the closed list
        closed_list[cur_node.node_state] = [cur_node.cost_to_come, cur_node]

        # check if current node is the goal node
        if reached_goal_node(cur_node.node_state, end_location):

            # start visualization of node exploration
            world_map = anime.visualize_exploration(world_map, start_location, end_location, closed_list)

            #  start backtracking to the start location from end location
            anime.backtrack_to_start(world_map, cur_node, start_location, end_location)

            return True

        # generate new nodes from the current node
        next_nodes = get_next_nodes(cur_node, world_map, clearance)

        # iterate through the next possible nodes from the current node
        for node in next_nodes:

            # get the node location
            node_x, node_y = node.node_state[0], node.node_state[1]

            # check if node is in the closed list
            if not present_in_list(closed_list, node):

                # check if node is in the open list
                if not present_in_list(open_list, node):

                    # check if the node is in the obstacle space
                    if not ut.inside_obstacle_space(node_x, node_y, world_map, clearance):

                        # add to open list
                        open_list[node.node_state] = [node.cost_to_come, node]
                        pq.put([node.cost_to_come, node.node_state, node])

                else:
                    # compare c2c of the cur node with the existing node in the open list
                    curr_c2c = node.cost_to_come
                    existing_c2c = open_list[node.node_state][0]

                    # check if cur node c2c is lower than existing node c2c in the open list
                    if curr_c2c < existing_c2c:

                        # update the open list node c2c with the lower c2c
                        open_list[node.node_state] = [curr_c2c, node]
                        pq.put([node.cost_to_come, node.node_state, node])

    print("NO SOLUTION FOUND")
    return False


def solve():

    Parser = argparse.ArgumentParser()
    Parser.add_argument('--init', default=None, type=str, help='start location of the robot. Example - 10,12 [NOTE - without spaces], Default: Random')
    Parser.add_argument('--goal', default=None, type=str, help='end location of the robot. Example - 150,120 [NOTE - without spaces], Default: Random')
    Parser.add_argument('--clearance', default=5, type=int, help='clearance dist from obstacles, DEFAULT: 5')

    Args = Parser.parse_args()

    goal_state = Args.goal
    initial_state = Args.init
    clearance = int(Args.clearance)

    if clearance > 50:
        print("[WARN]: Clearance value cannot be greater than 50, setting back to 5")
        clearance = 5

    print("[INFO]: Clearance from obstacles set to ", clearance)

    map_file = "./final_map.png"

    if not os.path.exists(map_file):
        print("[ERROR]: MAP File Does Not Exists: {}".format(map_file))
        print("Do you want to generate a new world map? Press 1 for YES, 0 for NO")
        create_map = 1
        display_progress = 0
        try:
            create_map = int(input())
        except Exception as e:
            print("[ERROR]: Incorrect input, Try Again. Exiting!!!")
            exit()
        if create_map == 1:
            try:
                print("Do you want to see progress as the map is generated, Press 1 for YES, 0 for NO")
                display_progress = int(input())
            except Exception as e:
                print("[ERROR]: Incorrect input, Try Again. Exiting!!!")
                exit()
            map_gen.start_map_generation(display_progress)
        else:
            print("[INFO]: Exiting!!!")
            exit()

    world_map = cv2.imread(map_file, 0)

    if initial_state is None:
        print("[WARN]: User did not specify any initial location, selecting RANDOMLY")
        start_location = ut.get_random_location(world_map, clearance)
    else:
        try:
            start_location = tuple(map(int, initial_state.split(',')))
            if ut.inside_obstacle_space(start_location[0], start_location[1], world_map, clearance, False):
                print("[ERROR]: Start Location is Inside Obstacle Space, Please Try Again")
                exit()
        except Exception as e:
            print("[ERROR]: Incorrect Initial Location input, Try Again. Exiting!!!")
            exit()

    if goal_state is None:
        print("[WARN]: User did not specify any goal location, selecting RANDOMLY")
        end_location = ut.get_random_location(world_map, clearance)
    else:
        try:
            end_location = tuple(map(int, goal_state.split(',')))
            if ut.inside_obstacle_space(end_location[0], end_location[1], world_map, clearance, False):
                print("[ERROR]: End Location is Inside Obstacle Space, Please Try Again")
                exit()
        except Exception as e:
            print("[ERROR]: Incorrect End Location input, Try Again. Exiting!!!")
            exit()

    color_world_map = cv2.cvtColor(world_map, cv2.COLOR_GRAY2BGR)
    print("[INFO]: BLUE Circle is the initial location: ", start_location)
    cv2.circle(color_world_map, start_location, 3, (255, 0, 0), -1)

    print("[INFO]: RED Circle is the goal location: ", end_location)
    cv2.circle(color_world_map, end_location, 3, (0, 0, 255), -1)

    cv2.namedWindow("start_and_end_location", cv2.WINDOW_NORMAL)
    cv2.moveWindow("start_and_end_location", 10, 10)
    cv2.imshow('start_and_end_location', color_world_map)
    cv2.waitKey(0)

    print("[INFO]: Started Solving ...")
    success = start_dijkstra(world_map, start_location, end_location, clearance)
    print("SUCCESS: ", success)


if __name__ == "__main__":
    solve()