#!/usr/bin/env python

import rospy, tf, time, math, heapq, Queue, numpy, collections
from nav_msgs.msg import GridCells, Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import String, Header
from implementation import *


#I based the A* implementation off of this website's information
	#http://www.redblobgames.com/pathfinding/a-star/implementation.html#sec-1-4


class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]


def pullData (x, y, gridMap):
	width = gridMap.info.width
	height = gridMap.info.height
	if (x < 0 or x > width or y < 0 or y > height):
		return 1
	dataPlacement = (width * y) + x
	return gridMap.data[dataPlacement]


def GetNeighbors (a, gridMap):
	#inits empty neighbor
	neighbors = []

	#Up - (North)
	if (pullData(a.x, a.y + 1, gridMap) == 0):
		neighbors.append(Point(a.x, a.y + 1, 0))
	#Down - (South)
	elif (pullData(a.x, a.y - 1, gridMap) == 0):
		neighbors.append(Point(a.x, a.y - 1, 0))
	#Left - (West)
	elif (pullData(a.x - 1, a.y, gridMap) == 0):
		neighbors.append(Point(a.x - 1, a.y, 0))
	return neighbors
	#Right - (East)
	elif (pullData(a.x + 1, a.y, gridMap) == 0):
		neighbors.append(Point(a.x + 1, a.y, 0))
	


def MakeGridCellsFromList (cellList):
	gridCells = GridCells()
	gridCells.cell_width = 1
	gridCells.cell_height = 1
	gridCells.cells = cellList
	gridCells.header.frame_id = 'map'
	return gridCells


def Waypoints (pointList):

	print "Waypoints Away!"

	waypointpub = rospy.Publisher('waypoints', GridCells)
	WaypointCells = []

	i = 0
	current_state = 2

	for item in pointList:
		current_point = item
		if (i < len(pointList)-1):
			next_point = pointList[i+1]
			i += 1

			#Only if the x coordinate is changing
			if ((not(next_point.x - current_point.x == 0)) and (next_point.y - current_point.y == 0)):
				if (current_state == 1):

					WaypointCells.append(current_point)
					#Publish
					publishableWaypoints = MakeGridCellsFromList(WaypointCells)
					waypointpub.publish(publishableWaypoints)

				current_state = 0	
				
			#Only if the y coordinate is changing
			elif ((next_point.x - current_point.x == 0) and (not(next_point.y - current_point.y == 0))):
				if (current_state == 0):

					WaypointCells.append(current_point)
					#Publish
					publishableWaypoints = MakeGridCellsFromList(WaypointCells)
					waypointpub.publish(publishableWaypoints)
				current_state = 1
				
			else:
				print "Something Went Bump in the Code"
				print current_point
				print next_point

	WaypointCells.append(pointList[len(pointList)-1])

	#Publish
	publishableWaypoints = MakeGridCellsFromList(WaypointCells)
	waypointpub.publish(publishableWaypoints)

	print WaypointCells

	return WaypointCells


def heuristic (a, b):
	return ((abs(a.x - b.x) + abs(a.y - b.y))*2)

def theSamePoint (a, b):
	return (a.x == b.x and a.y == b.y)

def tPoints(start, goal, gridMap):

	tStart = start
	tGoal = goal

	tStart.x = int(round((tStart.x - gridMap.info.origin.position.x) * 10))
	tStart.y = int(round((tStart.y - gridMap.info.origin.position.y) * 10))

	tGoal.x = int(round((tGoal.x - gridMap.info.origin.position.x) * 10))
	tGoal.y = int(round((tGoal.y - gridMap.info.origin.position.y) * 10))

	return tStart, tGoal

def Point2String (point):
	return ("x"+str(point.x)+"y"+str(point.y)+"z"+str(point.z))
	
def SearchForGoal (start, goal, gridMap):
	print "Do the Thing!"
	
	print start, goal

	#Lots of data things
	parents = {}
	parents[start] = None
	costs = {}
	costs[Point2String(start)] = 0
	frontierList = [start]
	visited = []
	found = [start]

	#The Frontier
	frontier = Queue.PriorityQueue()
	frontier.put((0, start))

	#The Publishers
	frontierPublisher = rospy.Publisher('frontier', GridCells) 
	visitedPublisher = rospy.Publisher('visited', GridCells) 

	print "I'm ready, I'm ready"

	success = 0

	while not frontier.empty():
		
		#Pull from the frontier and update the lists
		p, currentNode = frontier.get()
		if currentNode not in visited:
			print currentNode
			frontierList.remove(currentNode)
			visited.append(currentNode)

			#Publish
			publishableFrontier = MakeGridCellsFromList(frontierList)
			frontierPublisher.publish(publishableFrontier)
			#Publish
			publishableVisited = MakeGridCellsFromList(visited)
			visitedPublisher.publish(publishableVisited)

			#Only if at the goal
			if (theSamePoint(currentNode, goal)):
				success = 1
				break

			for neighbor in GetNeighbors(currentNode, gridMap):
				costToNeighbor = costs[Point2String(currentNode)] + 1
			
				if neighbor not in found or costs[Point2String(neighbor)] > costToNeighbor:

					costs[Point2String(neighbor)] = costToNeighbor

					priority = costToNeighbor + heuristic(neighbor, goal)
					frontier.put((priority, neighbor))

					if neighbor not in found:
						frontierList.append(neighbor)
						found.append(neighbor)
					parents[neighbor] = currentNode
	
	if success:
		return parents, costs, currentNode
	else:
		print"No way in, No way out"


def aStar (start, goal, gridMap):
	pathPublisher = rospy.Publisher('path', GridCells) 

	tStart, tGoal = tPoints(start, goal, gridMap)

	parents, costs, currentNode = SearchForGoal(start, goal, gridMap)
	path = Path()
	poseStampedList = []
	currentIndex = 0
	pathList = []

	print "Getting the path to Grandma's House"

	while not theSamePoint(currentNode, tStart):
		print currentNode
		pathList.append(currentNode)
		currentNode = parents[currentNode]
		currentIndex += 1

	pathList.append(tStart)
	publishablePath = MakeGridCellsFromList(pathList)

	print "Found the Light!"

	#Publish
	pathPublisher.publish(publishablePath)

	return pathList