#!/usr/bin/env python

import rospy, tf, numpy, math, roslib, time

from newastar import aStar
from nav_msgs.msg import GridCells, Path, Odometry, OccupancyGrid
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def checkerFrontier(maps, grid):
	frontiers = []
	width = maps.info.width
	height = maps.info.height
	resolution = maps.info.resolution
	offsetX = maps.info.origin.position.x
	offsetY = maps.info.origin.position.y
	x = 0
	for i in range(0, height): #height should be set to height of grid
		for j in range(0, width): #width should be set to width of grid
			#print k # used for debugging
			if (grid[i*width+j] == -1 and x == 0):
					point=Point()
					point.x=((j)*resolution)+offsetX + (.5 * resolution)
					point.y=((i)*resolution)+offsetY + (.5 * resolution)
					point.z=0
					
					frontiers.append(point)
					x = 5
			elif(x != 0):
				x -= 1
	return frontiers

def checkClosestFrontier(frontiers, res, map_origin, current, grid, wall):
	cX = 0
	cY = 0
	pX = 0
	pY = 0
	closest = 0
	goal = 0
	for i in frontiers:
		goal_cc = [int(i.x/res) + map_origin[0], int(i.y/res) + map_origin[1], 0]
		cX = abs(goal_cc[0] - current[0])
		cY = abs(goal_cc[1] - current[1])	
		#print cX, cY
		if(closest == 0):
			#print "yo"
			#print pX, pY
			pX = cX
			py = cY
			closest = (i.x, i.y)
		elif(cX <= pX and cY <= pY):
			print "hey"
			pX = cX
			py = cY
			closest = (i.x, i.y)
	return closest


# Add def for publishing frontiers, and make them go 5 cells out from closest known point
