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


def noMoreFrontier(grid):
	if(canDriveTo and cells != -1):
		print "No more locations to explore"
		publishTwist(0, 0)



def canDriveTo(sometihng):

	drivable = False;

	if(not walls)
		print "Turtlebot can drive  to location"
		 drivable = True

	return drivable