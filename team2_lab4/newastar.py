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

	def in_bounds(self, tup):
		(x, y) = tup
		return 0 <= x < self.width and 0 <= y < self.height
	
	def passable(self, tup):
		return tup not in self.wall

	def neighbors(self, tup):
		(x, y) = tup

		results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
		results = filter(self.in_bounds, results)
		results = filter(self.passable, results)
		return results
		
		

	#def Children(self):
	#	v = self.value
	#	s = self.start
	#	g = self.goal
	#	m = self.grid
	#	w = self.wall
	#	c = self.neighbors(self.coor)

	#	for t in c:
	#		child = Node(t, s, g, m, w)
	#		self.children.append(child)
	#		child.parent.append(self)

		


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
		#Manhattan distance
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
		
		if(current == goal):
			break

		for i in node.neighbors(current):
			new_cost = cost_so_far[current] + 1
			if i not in cost_so_far or new_cost < cost_so_far[i]:
				cost_so_far[i] = new_cost
				priority = new_cost + heuristic(goal, i)
				frontier.push(i, priority)
				came_from[i] = current
	
	return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
	print came_from
	current = goal
	path = [current]
	while (current != start):
		current = came_from[current]
		path.append(current)
	path.append(start)
	path.reverse()
	return path

def aStar(start, goal, grid, wall):
	init = (start[0], start[1])
	end = (goal[0], goal[1])
	
	origin = Node(init, init, end, grid, wall)
	fron, cost = Solve(origin, init, end)
	solution = reconstruct_path(fron, init, end)
	#print solution
	#print cost
	return solution, cost
	 