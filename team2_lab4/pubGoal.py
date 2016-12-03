def pubGoal(grid):
	global pose
	global goal_pub
	
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution
	cells.cell_height = resolution

	for node in grid:
		#Need to add the goal point
		#point = gridToWorld('gridPoint', worldMap)
		point = gridToWorld(goal.pose.position, worldMap)
		cells.cells.append(point)
	goal_pub.publish(cells)


''' Make sure to add the global to the necessary locations in the main code '''

#goal_pub = rospy.Publisher('goal_point', PoseStamped, queue_size=1)


