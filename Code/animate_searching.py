import cv2


def visualize_exploration(world_map, start_location, end_location, closed_list):

    print("[INFO]: Started Nodes Exploration Visualization from the INITIAL node to the GOAL node")

    # convert the gray scale world map to bgr for visualization
    color_world_map = cv2.cvtColor(world_map, cv2.COLOR_GRAY2BGR)

    # draw the start location with blue colored circle
    cv2.circle(color_world_map, start_location, 3, (255, 0, 0), -1)

    # draw the end location with red colored circle
    cv2.circle(color_world_map, end_location, 3, (0, 0, 255), -1)

    # create a window for visualization
    cv2.namedWindow("visualization", cv2.WINDOW_NORMAL)

    # move the window to (10, 10)
    cv2.moveWindow("visualization", 10, 10)

    # iterate through the closed list dictionary to get the explored nodes
    for idx, val in enumerate(closed_list.values()):

        # get the node state - location(x, y)
        node_state = val[1].node_state

        # draw the current node location with cyan colored circle
        cv2.circle(color_world_map, node_state, 1, (255, 255, 0), -1)

        # draw the start location with blue colored circle
        cv2.circle(color_world_map, start_location, 3, (255, 0, 0), -1)

        # draw the end location with red colored circle
        cv2.circle(color_world_map, end_location, 3, (0, 0, 255), -1)

        # for faster visualization, display only the 20th frame
        if idx % 20 == 0:

            # display the exploration in progress
            cv2.imshow("visualization", color_world_map)

            # wait for 1 millisecond and then continue to next frame
            cv2.waitKey(1)

    # return the explored nodes visualization colored world map
    return color_world_map


def backtrack_to_start(color_world_map, cur_node, start_location, end_location):

    print("[INFO]: Started Backtracking from the GOAL node to the INITIAL node")

    # create a window for visualization
    cv2.namedWindow("visualization", cv2.WINDOW_NORMAL)

    # move the window to (10, 10)
    cv2.moveWindow("visualization", 10, 10)

    # iterate through all the nodes till their parent is not None
    while cur_node.parent_node is not None:

        # draw the current node location with dark green colored circle
        cv2.circle(color_world_map, cur_node.node_state, 1, (0, 150, 0), -1)

        # draw the start location with blue colored circle
        cv2.circle(color_world_map, start_location, 3, (255, 0, 0), -1)

        # draw the end location with red colored circle
        cv2.circle(color_world_map, end_location, 3, (0, 0, 255), -1)

        # set current node to its parent node for next iteration
        cur_node = cur_node.parent_node

        # display the backtrack in progress
        cv2.imshow("visualization", color_world_map)

        # wait for 1 millisecond and then continue to next frame
        cv2.waitKey(1)

    # after visualization is complete, let the user close the window
    cv2.waitKey(0)