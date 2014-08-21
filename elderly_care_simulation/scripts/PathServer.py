#!/usr/bin/env python

import math
import time
import roslib; roslib.load_manifest('elderly_care_simulation')
import sys
import rospy
from geometry_msgs.msg import Point
from elderly_care_simulation.srv import *

MAP_WIDTH = 20
MAP_HEIGHT = 20

graph = {}

def get_x_location(x):
    return int(math.floor(x + MAP_WIDTH / 2))


def get_y_location(y):
    return int(math.floor(y + MAP_HEIGHT / 2))


def shortest_path(start, end):
    """Code for shortest path algorithm adapted from
       http://stackoverflow.com/questions/8922060/breadth-first-search-trace-path"""
    # maintain a queue of paths
    queue = []
    
    # Nodes that have already been visited
    visited = set()
    
    # push the first path into the queue
    queue.append([start])
    while queue:
        # get the first path from the queue
        path = queue.pop(0)
        # get the last node from the path
        node = path[-1]
        
        # add the current node to the visited nodes
        visited.add(node)
        
        # path found
        if node == end:
            return path
            
        rospy.loginfo(node)

        # enumerate all adjacent nodes, construct a new path and push it into the queue
        for adjacent in graph.get(node, []):
            new_path = list(path)
            new_path.append(adjacent)
            if adjacent not in visited:
                queue.append(new_path)


def create_return_message(path):
    formatted_path = []
    for current in reversed(path):
        formatted_path.append(Point(current[0] - MAP_WIDTH / 2, current[1] - MAP_HEIGHT / 2, 0))
    return formatted_path


def find_path(req):
    from_point = req.from_point;
    to_point = req.to_point;

    from_node = get_x_location(from_point.x), get_y_location(from_point.y) 
    to_node =  get_x_location(to_point.x), get_y_location(to_point.y)
    path = shortest_path(from_node, to_node)
    path.pop() # Remove the point that the robot already is
    return FindPathResponse(create_return_message(path))


def find_path_server():
    rospy.init_node('find_path_server')
    service = rospy.Service('find_path', FindPath, find_path)
    rospy.spin()
    

def check_vacancy_at_cell(house_map, cell):
	"""
	Return True if the given cell is vacant.

    Vacancy is defined as a '0' in the house map at the given coordinates.
    (i.e. there is no wall at that location)
	"""
	x = cell[0]
	y = cell[1]
	
	if not 0 <= x < MAP_WIDTH:
		return False
	
	if not 0 <= y < MAP_HEIGHT:
		return False
		
	return house_map[y][x] == '0'
    
    
def get_vacant_neighbours(house_map, cell):
	"""
    Get a list of coordinates that are vacant around the given cell coordinates.

    Only neighbours to the top, bottom, left and right are considered.
	
    'house_map': a list of lists. Each list is a row of 0s and 1s (vacant cells and walls respectively)
	'cell': a tuple: (x, y) with the origin at the bottom left

    return: a list of up to four (x, y) tuples, each representing a neighbour of 'cell' that is vacant
	"""
	x = cell[0]
	y = cell[1]
	
	neighbours = [(x, y + 1), (x + 1, y), (x, y - 1), (x - 1, y)]
	vacant_neighbours = []
	
	for neighbour in neighbours:
		if check_vacancy_at_cell(house_map, neighbour):
			vacant_neighbours.append(neighbour)
			
	return vacant_neighbours
    
    
def generate_graph(filename):
    """
    Read a house map's pgm file and generate an adjacency list that models available paths.

    Entries in the adjacency list (or dict) show the cells that are reachable from any given cell.
    Cells are represented as (x, y) tuples. e.g.:

    {
        (3, 4): [(3, 3), (2, 3)],
        (7, 2): [(7, 1)]
    }

    'filename': the name of a pgm file of the map 
    """
	house_map = []
	
	with open(filename, 'r') as f:
		lines = f.read().splitlines()
	
	# Remove pgm file information
	del lines[:3]
	
	for line in reversed(lines):
		house_map.append(line.split())
	
	for y, row in enumerate(house_map):
		for x, cell in enumerate(row):
			cell = (x, y)
			graph[cell] = get_vacant_neighbours(house_map, cell)
		

if __name__ == "__main__":
    if len(sys.argv) < 2:
        rospy.loginfo("No world file given")
    else:
        rospy.loginfo("Ready to start finding paths")
        generate_graph(sys.argv[1])
        find_path_server()
