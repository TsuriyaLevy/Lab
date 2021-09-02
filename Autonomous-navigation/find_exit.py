#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import defines
import math
from Point import Point


def create_data(file_name):
    """
    Turn the file of the points map into a two-dimensional points map and put it in an array
    :param file_name: The points map file
    :return: The two-dimensional points array
    """
    points_array = np.array([0, 0])
    points_file = open(file_name)
    for point in points_file.readlines():
        values = point.split(',')
        points_array = np.vstack((points_array, np.array([float(values[0]), float(values[2])])))
    points_file.close()
    return points_array


def max_distance(map_matrix, mid_point):
    """
    Finds the farthest point, (i,j), from the mid_point such that map_matrix[j][i]=1 (farthest point that exist)
    :param map_matrix: two-dimensional matrix that consume 1 or 0 in each cell
    :param mid_point: A point in the two-dimensional space
    :return:
    """
    max_dist = 0
    farthest_point = [0, 0]
    for row in range(defines.NUM_BLOCKS_IN_LENGTH):
        for col in range(defines.NUM_BLOCKS_IN_WIDTH):
            if map_matrix[row][col] == 1:
                p = [col, row]
                dist = math.dist(mid_point, p)
                if max_dist < dist:
                    max_dist = dist
                    farthest_point = p
    return farthest_point


def find_exit(points):
    """
    Receives an array of points that describe a two-dimensional room and finds the exit by cleaning all unimportant
    points, finding the farthest point from the middle of the room and returning it as the exit point
    :param points: A two-dimensional array
    :return: The exit point
    """
    # preparations for mapping points to a block matrix
    block_array = np.zeros(defines.NUM_OF_BLOCKS)
    block_mat = block_array.reshape(defines.NUM_BLOCKS_IN_LENGTH, defines.NUM_BLOCKS_IN_WIDTH)

    max_x = max(points[:, 0])
    min_x = min(points[:, 0])
    max_y = max(points[:, 1])
    min_y = min(points[:, 1])

    map_width = max_x - min_x + 1
    map_length = max_y - min_y + 1

    block_width = (map_width + 1) / defines.NUM_BLOCKS_IN_WIDTH
    block_length = (map_length + 1) / defines.NUM_BLOCKS_IN_LENGTH

    for i in points:
        i[0] = i[0] - min_x
        i[1] = i[1] - min_y

    # mapping points into blocks in the matrix
    for i in points:
        col = int(i[0] / block_width)
        row = int(i[1] / block_length)
        block_mat[row][col] += 1

    # erase all the blocks that contain a small amount of points
    for row in range(defines.NUM_BLOCKS_IN_LENGTH):
        for col in range(defines.NUM_BLOCKS_IN_WIDTH):
            if block_mat[row][col] <= 2:
                block_mat[row][col] = 0
            else:
                block_mat[row][col] = 1

    # finds the middle of the room, and the cell in the block matrix that contains it
    tello_location = [0, 0]
    tello_location[0] = tello_location[0] - min_x
    tello_location[1] = tello_location[1] - min_y
    tello_location[0] = int(tello_location[0] / block_width)
    tello_location[1] = int(tello_location[1] / block_length)

    # finds the farthest cell from the cell that contain the middle of the room
    door_point = max_distance(block_mat, [tello_location[0], tello_location[1]])

    # calculates the place of the door_point in the original map
    door_point[0] = door_point[0] * block_width + 0.5 * block_width + min_x
    door_point[1] = door_point[1] * block_length + 0.5 * block_length + min_y

    return door_point
        

def find_the_exit(point_path):
    """
    This is the main function
    :param point_path: The path to the points map file
    :return: The exit point
    """
    map_points = create_data(point_path)
    exit_point = find_exit(map_points)
    exit_point_object = Point(x=exit_point[0], y=exit_point[1], z=0.1)

    return exit_point_object

