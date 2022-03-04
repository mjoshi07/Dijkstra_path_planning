import numpy as np
import random


def inside_obstacle_space(x, y, world_map, clearance, modify=True):

    # get height and width of the world map
    h, w = world_map.shape

    # check if point is beyond the boundary
    if beyond_boundary([x, y], [w, h], clearance):
        return True

    # check if (x, y) lie within an obstacle space
    if world_map[y, x] == 0:
        return True

    # loop through an area of clearance x clearance around the current location (x, y)
    for i in range(-clearance, clearance, 1):
        for j in range(-clearance, clearance, 1):
            point_x, point_y = x + i, y + j

            if modify:
                # shift points inside boundary of world map
                point_x, point_y = shift_inside_boundary([x + i, y + j], [w, h], clearance - i, clearance - j)

            else:
                # check if point is beyond the boundary
                if beyond_boundary([x, y], [w, h], clearance):
                    print(point_x, " ", point_y)
                    return True

            # check if the (x, y) lie inside an obstacle space
            if world_map[point_y, point_x] == 0:

                # (x, y) lie within an obstacle space, return True
                return True

    # (x, y) does not lie within any obstacle space, return False
    return False


def get_random_location(map, clearance):

    # get locations where pixel value is greater than 50, just to be on safe side
    Y, X = np.where(map > 50)

    # cast the values into a list
    X = list(X)
    Y = list(Y)

    while True:
        # randomly select a x location
        x_random = random.sample(X, 1)[0]

        # randomly select a y location
        y_random = random.sample(Y, 1)[0]

        # check if the (x, y) lie in the obstacle space
        if not inside_obstacle_space(x_random, y_random, map, clearance):
            # break out of while loop since (x, y) does not lie in the obstacle space
            break

    # return the randomly selected (x, y) location
    return x_random, y_random


def shift_inside_boundary(point, size, clearance_in_x, clearance_in_y):

    # extract x, y from point
    x, y = point[0], point[1]

    # extract width and height from size
    w, h = size[0], size[1]

    # check for boundary conditions in width
    x = min(w - clearance_in_x - 1, max(0 + clearance_in_x, x))

    # check for boundary conditions in height
    y = min(h - clearance_in_y - 1, max(0 + clearance_in_y, y))

    return x, y


def beyond_boundary(point, size, clearance):

    # extract x, y from point
    x, y = point[0], point[1]

    # extract width and height from size
    w, h = size[0], size[1]

    if x < 0 + clearance:
        return True
    if x >= w - clearance - 1:
        return True
    if y < 0 + clearance:
        return True
    if y >= h - clearance - 1:
        return True

    return False
