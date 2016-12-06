#!/usr/bin/env python

import rospy, tf, numpy, math, roslib, time
from newastar import aStar
from nav_msgs.msg import GridCells, Path, Odometry, OccupancyGrid
from std_msgs.msg import String, Header

from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion

from kobuki_msgs.msg import BumperEvent

from tf.transformations import euler_from_quaternion, quaternion_from_euler


'''
def publishFrontier(grid):
	global pub_frontier
		# resolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution

	for node in grid:
		point=Point()
		point = worldToGrid(worldPoint, worldMap)
		cells.cells.append(point)
	pub_frontier.publish(cells)

	pub_frontier = rospy.Publisher('map_frontier', GridCells, queue_size=1)
'''


def checkerFrontier(grid):
	global frontiers
	frontiers = []
	x = 0
	for i in range(0,height): #height should be set to height of grid
		for j in range(0,width): #width should be set to width of grid
			#print k # used for debugging
			if (grid[i*width+j] == -1 and x == 0):
					point=Point()
					point.x=(j*resolution)+offsetX + (.5 * resolution)
					point.y=(i*resolution)+offsetY + (.5 * resolution)
					point.z=0
					frontiers.append(point)
					x = 5
			elif(x != 0):
				x -= 1

def checkClosestFrontier(current):
	global frontiers
	cX = 0
	cY = 0
	pX = 0
	pY = 0
	closest = 0
	
	for i in frontiers:
		cX = abs(i.x - current[0])
		cY = abs(i.y - current[1])
		if(closest == 0):
			closest = (i.x, i.y)
		elif(cX < pX and cY < pY):
			closest = (i.x, i.y)
		pX = cX
		pY = cY
	return closest 
	
