#!/usr/bin/env python

#Used as reference for implementing A*:
#Red Blob Games, Implementation of A*, RedBlobGames, 07/06/2016, URL:www.redblobgames.com/pathfinding/a-star/implementation.html, Accesed 10/29/2016
import rospy
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import quaternion_from_euler
import tf
import numpy
import math
import rospy, tf, numpy, math, heapq

class Node:
	def __init__(self, coor, start, goal, grid, wall):
		self.coor = coor
		#self.parent = []
		self.start = start
		self.goal = goal
		#self.children = []
		self.grid = grid
		self.width = grid.info.width
		self.height = grid.info.height
		#self.path = []
		self.wall = wall
		#if(len(self.path) == 0):
		#	self.path.append(start)

	#makes sure the child is within the limits of the map
	def in_bounds(self, tup):
		(x, y, z) = tup
		return 0 <= x < self.width and 0 <= y < self.height
	
	#makes sure the child is not an obstacle
	def passable(self, tup):
		(x, y, z) = tup
		tup2 = (x, y)
		return tup2 not in self.wall

	#makes the child
	def neighbors(self, tup):
		(x, y, z) = tup
		results = [(x+1, y, 0), (x, y-1, 270), (x-1, y, 180), (x, y+1, 90)]
		results = filter(self.in_bounds, results)
		results = filter(self.passable, results)
		return results
		
		


class PriorityQueue:
	def __init__(self):
		self.element = []
	
	def empty(self):
		return len(self.element) == 0	
	
	def push(self, item, priority):
		heapq.heappush(self.element, (priority, item))
	
	def pop(self):
		return heapq.heappop(self.element)[1]


def heuristic(a, b):
		#Straight Line Distance
		return abs(a[0]- b[0]) + abs(a[1] - b[1])	

def Solve(node, start, goal):
	frontier = PriorityQueue()
	frontier.push(start, 0)
	came_from = {}
	cost_so_far = {}
	came_from[start] = None
	cost_so_far[start] = 0

	while not frontier.empty():
		current = frontier.pop()
		
		#If the Algorithm reached the goal and add the goal to the map in case the orientation is different
		if(current[0] == goal[0] and current[1] == goal[1]):
			came_from[goal] = current	
			break
		#Iterates through all possible children
		for i in node.neighbors(current):
			#print i
			#rospy.sleep(1)
			new_cost = cost_so_far[current] + 1
			#If the child has already been visited don't call it again
			if i not in cost_so_far or new_cost < cost_so_far[i]:
				#print i
				#rospy.sleep(.5)
				cost_so_far[i] = new_cost
				priority = new_cost + heuristic(goal, i)
				frontier.push(i, priority)
				came_from[i] = current
	return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
	#print came_from
	current = goal
	path = [current]
	#Go through the map until the path is reconstructed
	while (current != start):
		try:
			#print current 
			#rospy.sleep(1)
			current = came_from[current]
			path.append(current)
			print len(path)
		#If the current coordinates is not included within the map
		except KeyError:
			print came_from
	#path.append(start)
	path.reverse()
	return path

def aStar(start, goal, grid, wall):
	#Makes sure the values given are tuples
	init = (start[0], start[1], start[2])
	end = (goal[0], goal[1], goal[2])
	origin = Node(init, init, end, grid, wall)
	fron, prev = Solve(origin, init, end)
	print "aStar Solution"
	solution = reconstruct_path(fron, init, end)
	print solution
	#rospy.sleep(1)
	#print cost
	return solution, prev
	 
