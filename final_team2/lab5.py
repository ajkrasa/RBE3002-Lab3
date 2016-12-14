#!/usr/bin/env python

import rospy, tf, numpy, math, roslib, time

from newastar import aStar
from nav_msgs.msg import GridCells, Path, Odometry, OccupancyGrid
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from FrontierChecker import checkerFrontier, checkClosestFrontier
		


def readWorldMapCallback(data):
	global world_map
	global neworldMap_flag
	global start_pose
	global goal_pose
	global world_data
	global resolution
	global offsetX
	global offsetY
	global width
	global height

	world_map = data
	world_data = data.data
	resolution = data.info.resolution
	start_pose = None
	goal_pose = None
	neworldMap_flag = 1
	offsetX = data.info.origin.position.x
	offsetY = data.info.origin.position.y
	width = data.info.width
	height = data.info.height
	print data.info

def goalCallback(data):
	global goal_pose
	tmp_buf = []
	pos_buf = data.pose.position
	tmp_buf.append(pos_buf.x)
	tmp_buf.append(pos_buf.y)
	q = data.pose.orientation
	quat = [q.x, q.y, q.z, q.w]
	r, p, y = euler_from_quaternion(quat)
	tmp_buf.append(y)
	goal_pose = tmp_buf

def costMapCallback(data):
	global cost_map
	cost_map = data


def readStart(startPos):
	global startRead
	startRead = True
	global start_pose
	global startIndex
	
	tmp_pose = []
	temp_pos = startPos.pose.pose.position
	tmp_pose.append(temp_pos.x)
	tmp_pose.append(temp_pos.y)
	q2 = startPos.pose.pose.orientation
	quat2 = [q2.x, q2.y, q2.z, q2.w]
	r2, q2, y2 = euler_from_quaternion(quat2)
	tmp_pose.append(y2)
	start_pose = tmp_pose

	


'''-----------------------------------------Helper Functions--------------------------------------------'''

# (real world) -> (grid)
def worldToGrid(worldPoint, worldMap):
	res = worldMap.info.resolution
	gridPoint = Point()
	gridPoint.x = int((worldPoint[0] - worldMap.info.origin.position.x)/res) 
	gridPoint.y = int((worldPoint[1] - worldMap.info.origin.position.y)/res)
	gridPoint.z = 0
	return gridPoint

# (grid) -> (real world)
def gridToWorld(gridPoint, worldMap):
	#print gridPoint
	gridx = float(gridPoint[0])
	if(gridPoint[1] > 0):
		gridy = float(gridPoint[1])
	else:
		print "Tuple Trouble"
	res = worldMap.info.resolution
	#print res
	worldPoint = Point()
	worldPoint.x = ((gridx*res) + worldMap.info.origin.position.x) + (0.5*res)
	worldPoint.y = ((gridy*res) + worldMap.info.origin.position.y) + (0.5*res)
	worldPoint.z = 0
	return worldPoint

# Generate Waypoints
def genWaypoints(g_path_rev, initial, worldMap):
	global pubway
	path = Path()
	path.header.frame_id = 'map'
	poseStamped = PoseStamped()
	poseStamped.header.frame_id = 'map'
	pose = Pose()
	#cells = GridCells()
	#cells.header.frame_id = 'waypoints'
	#cells.cell_width = resolution 
	#cells.cell_height = resolution

	gridPoints = []
	w_ori = []
	g_path = list(reversed(g_path_rev))

	# Waypoints based on change in direction
	prev_pt = 0
	tmp_ctr = 0	
	for pt in g_path:
		tmp_ctr += 1
		#print pt
		#print tmp_ctr, len(g_path)
		if(prev_pt == 0):
			gridPoints.append(pt)
			prev_pt = pt
			yaw = math.radians(pt[2])
			q_t = tf.transformations.quaternion_from_euler(0, 0, yaw)
			w_ori.append(q_t)
			#print w_ori
		elif(tmp_ctr == len(g_path)):
			if(initial[2] == pt[2]):
				prev_pt = pt
			else:
				gridPoints.append(pt)
				prev_pt = pt
				yaw = math.radians(pt[2])
				q_t = tf.transformations.quaternion_from_euler(0, 0, yaw)
				w_ori.append(q_t)
				#print w_ori
		elif(prev_pt[2] == pt[2]):
			prev_pt = pt
		elif(prev_pt[2] != pt[2]):
			gridPoints.append(pt)
			prev_pt = pt
			yaw = math.radians(pt[2])
			q_t = tf.transformations.quaternion_from_euler(0, 0, yaw)
			w_ori.append(q_t)
			
		else:
			prev_pt = pt
		

	# Convert grid points to world coordinates
	c = 0
	worldPoints = []
	print len(gridPoints)
	for point in gridPoints:
		#point2=Point()
		#point2 = gridToWorld(point, worldMap) 
		#cells.cells.append(point2)
		pose.position = gridToWorld(point, worldMap)
		pose.orientation.x = w_ori[c][0]
		pose.orientation.y = w_ori[c][1]
		pose.orientation.z = w_ori[c][2]
		pose.orientation.w = w_ori[c][3]
		poseStamped.pose = pose
		path.poses.append(poseStamped)
		c += 1
	# Create actual Path()
	#print path
	#pubway.publish(cells)
	return path

'''-----------------------------------------Update Grid Functions---------------------------------------'''

def rvizPath(cell_list, worldMap):
	global path_pub	
	path_GC = GridCells()
	path_GC.cell_width = worldMap.info.resolution
	path_GC.cell_height = worldMap.info.resolution
	path_GC.cells = []
	for cell in cell_list:
		path_GC.cells.append(gridToWorld(cell, worldMap))
	path_GC.header.frame_id = 'map'
	path_pub.publish(path_GC)

'''----------------------------------------Update Cells Function-----------------------------------------'''

def publishFrontier(grid):
	global pub_frontier
		# resolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution
	
	for node in grid:
		#print node
		point = Point()
		#point = worldToGrid(node, worldMap)
		point.x = grid[0]
		point.y = grid[1]
		point.z = 0
		cells.cells.append(point)
	pub_frontier.publish(cells)


def publishCells(grid):
	global pub
	global wall
	wall = None
	wall = []
	# resolution and offset of the map
	k=-2
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution
	cY = 0
	cX = 0
	for i in range(0,height): #height should be set to height of grid
		for j in range(0,width): #width should be set to width of grid
			#print k # used for debugging
			if (grid[i*width+j] == 100):
				while(cY != 3):
					cX = 0
					while(cX != 3):	
						case1 = (i+cY)*width+(j+cX)
						case2 = (i-cY)*width+(j-cX)
						if(0 <= case1 and case1 <= (height * width - 1)):
							point=Point()
							point.x=((j+cX)*resolution)+offsetX + (.5 * resolution)
							point.y=((i+cY)*resolution)+offsetY + (.5 * resolution)
							point.z=0
							wall.append((j+cX,i+cY))
							cells.cells.append(point)
						elif(0 <= case2 and case2 <= (height * width - 1)):
							point=Point()
							point.x=((j-cX)*resolution)+offsetX + (.5 * resolution)
							point.y=((i-cY)*resolution)+offsetY + (.5 * resolution)
							point.z=0
							wall.append((j-cX,i-cY))
							cells.cells.append(point)
						cX += 1
					cY += 1
				cY = 0
	pub.publish(cells)

'''----------------------------------------Navigation Functions-----------------------------------------'''

# Publish Twist msgs
def publishTwist(lin_vel, ang_vel):
	global prev_twist
	global nav_pub
	
	twist_msg = Twist();				#Create Twist Message
	if lin_vel == 0 and ang_vel == 0:
		twist_msg.linear.x = (prev_twist.linear.x)/3
		twist_msg.angular.z = (prev_twist.angular.z)/3
		while twist_msg.linear.x > 0.05 and twist_msg.angular.z > 0.05:
			twist_msg.linear.x = (prev_twist.linear.x)/3
			twist_msg.angular.z = (prev_twist.angular.z)/3
			prev_twist = twist_msg
			nav_pub.publish(twist_msg)			  #Send Message
			rospy.sleep(rospy.Duration(0.2, 0))
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		nav_pub.publish(twist_msg)
		prev_twist.linear.x = 0
		prev_twist.angular.z = 0
	else:
		twist_msg.linear.x = (2*lin_vel + prev_twist.linear.x)/3
		twist_msg.angular.z = (2*ang_vel + prev_twist.angular.z)/3
		prev_twist = twist_msg
		nav_pub.publish(twist_msg)		  #Send Message

# Drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	global pose
	#rospy.sleep(1)
	

	x0 = pose.position.x		#Set origin
	y0 = pose.position.y
	q0 = (pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w)
	x2 = goal.pose.position.x
	y2 = goal.pose.position.y
	q2 = (goal.pose.orientation.x,
			goal.pose.orientation.y,
			goal.pose.orientation.z,
			goal.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(q0)
	euler2 = tf.transformations.euler_from_quaternion(q2)
	theta0 = math.degrees(euler[2])
	theta2 = math.degrees(euler2[2])

	dx = x2 - x0
	dy = y2 - y0
	#theta1 = math.atan2(dy, dx)

	#dtheta0 = theta1 - theta0
	#dtheta1 = theta2 - theta1
	#dtheta0 = theta2 - theta0
	distance = math.sqrt(dx**2 + dy**2)
	
	print "distance: %d" % distance
	print "theta2: %d" % theta2
	
	rotate(theta2)
	driveStraight(0.1, distance)
	#rotate(dtheta1)

def rotate(angle):
	global odom_list
	global pose
	global nav_pub
	pose = Pose()

	if(angle > 180 or angle < -180):
		print "angle is too large or small"
	
	#set rotation direction 
	error = angle - math.degrees(pose.orientation.z)

	if(angle < 0):
		publishTwist(0,-.5)
	else:
		publishTwist(0, .5) 

	while((abs(error) >= 2) and not rospy.is_shutdown()):
		ang_vel = error/45
		if(ang_vel < .1 and ang_vel > 0):
			ang_vel = .1
	   	elif(ang_vel > -.1 and ang_vel < 0):
			ang_vel = -.1
		elif(ang_vel > 1):
			ang_vel = 1
		elif(ang_vel < -1):
			ang_vel = -1
		publishTwist(0, ang_vel)
		error = angle - math.degrees(pose.orientation.z)
	publishTwist(0, 0)
	rospy.sleep(.5)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global pose

	x0 = pose.position.x   #Set origin
	y0 = pose.position.y

	done = False
	while (not done and not rospy.is_shutdown()):
		x1 = pose.position.x
		y1 = pose.position.y
		d = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)	  #Distance formula
		if (d >= distance):
			publishTwist(0, 0)
			done = True
		else:
			publishTwist(speed, 0)
	publishTwist(0,0)
	rospy.sleep(.5)

#Bumper Event Callback function
def readBumper(msg):
	"""Bumper event callback"""
	if (msg.state == 1):
        # When pressed the wheels will immediatley stop moving and will not move until the button is no longer being pressed
		print "Bumper pressed!"
		#Replace       
		executeTrajectory()


# Odometry Callback function.
def readOdom(event):
	global pose

	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))	 
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other 
	pose.position.x = position[0]
	pose.position.y = position[1]

	odomW = orientation
	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	#Convert yaw to degrees
	pose.orientation.z = yaw


'''
Main Setup
'''

def initial():
	rotate(180)
	rotate(-180)


def run():	
	global pub

	# Create Node
	rospy.init_node('lab5')

	# Global Variables
	global neworldMap_flag
	global world_map
	global goal_pose
	global start_pose
	global cost_map
	global world_data
	global wall
	global frontiers
	
	# Global Variables for Navigation
	global pose
	global odom_tf
	global odom_list
	global prev_twist
	global pubway

	# Initialize Global Variables
	wall = []
	pose = Pose()
	neworldMap_flag = 0
	world_map = None
	start_pose = None
	goal_pose = None
	cost_map = OccupancyGrid()

	# Initialize Navigation GV
	prev_twist = Twist();
	prev_twist.linear.x = 0
	prev_twist.angular.z = 0

	odom_list = tf.TransformListener()
	odom_tf = tf.TransformBroadcaster()
	odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

	# Subscribers
	world_map_sub = rospy.Subscriber('/map', OccupancyGrid, readWorldMapCallback)
	odom_sub = rospy.Subscriber('/odom', Odometry, readOdom)
	# cost_map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, costMapCallback)

	goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalCallback, queue_size=1) #change topic for best results
	navgoal_sub = rospy.Subscriber('/move_base_simple/2goal', PoseStamped, goalCallback, queue_size=1) #change topic for best results
	goal_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
	cost_pub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, costMapCallback)
	#bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1)#

	
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
	dispathpub = rospy.Publisher("/BS_topic", Path, queue_size=1)
	# Publishers
	global path_pub
	global waypoints_pub
	global nav_pub
	global pub_frontier

	path_pub = rospy.Publisher('/lab4/path', GridCells, queue_size=1)
	waypoints_pub = rospy.Publisher('/lab4/waypoints', Path, queue_size=1)
	nav_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
	pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
	pub_frontier = rospy.Publisher('map_frontier', GridCells, queue_size=1)
	pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired

	rospy.sleep(1)
	
	rospy.Timer(rospy.Duration(.01), readOdom)
	
	print "Waiting for map"
	while world_map == None and not rospy.is_shutdown():
		pass
	map_cache = world_map
	#world_map = None
	print "Received map"

	print "initial"
	initial()
	print "end initial"

	frontiers = checkerFrontier(map_cache, world_data)
	tolerance = 23
	
	while not rospy.is_shutdown() and frontiers != None:
		path = Path()
		publishCells(world_data)

		if world_map != None:
			map_cache = world_map
			#world_map = None
			print "Updated map cache"

		start_cache = (pose.position.x, pose.position.y, pose.orientation.z)
		# Prepping for A*
		tmp_wp_ctr = 0
		origin_cache = []
		origin_cache.append(map_cache.info.origin.position.x)
		origin_cache.append(map_cache.info.origin.position.y)
		q_tmp = map_cache.info.origin.orientation
		quat_tmp = [q_tmp.x, q_tmp.y, q_tmp.z, q_tmp.w]
		r_t, p_t, y_t = euler_from_quaternion(quat_tmp)
		d_t = math.degrees(y_t)
		origin_cache.append(d_t)
		res = map_cache.info.resolution
		print start_cache
		print origin_cache
		map_origin = (int(-origin_cache[0]/res), int(-origin_cache[1]/res), 0)
		print map_origin
		start_cc = [int(start_cache[0]/res) + map_origin[0], int(start_cache[1]/res) + map_origin[1], 0]
		goal_cache = checkClosestFrontier(frontiers, start_cache, map_cache, wall)
		print goal_cache
		goal_cc = [int(goal_cache[0]/res) + map_origin[0], int(goal_cache[1]/res) + map_origin[1], 0]
		
		# Running A*
		print start_cc, goal_cc
		rospy.sleep(1)
		publishFrontier(goal_cache)

		generated_path, prev = aStar(start_cc, goal_cc, map_cache, wall)
		print "Finished running A* algorithm"

		if generated_path != None and not rospy.is_shutdown():
			print "Updated RViz with path"
			
			rvizPath(generated_path, map_cache)
			path = genWaypoints(generated_path, start_cc, map_cache)
			waypoints_pub.publish(path)

			print "Published generated path to topic: [/lab5/waypoints]"

		print "Waypoints:"
		tmp_wp_ctr = 0
		for waypoint in path.poses:
			tmp_wp_ctr += 1

		at_goal = False
		tmp_wp_ctr = 0
		while not at_goal and not rospy.is_shutdown():
			curr_cc = []
			#print path.poses
			for waypoint in path.poses:
				print "Navigating to waypoint: ", tmp_wp_ctr
				tmp_wp_ctr += 1

				# Replanning
				navToPose(waypoint)
				print "stuck on NavToPose"
				curr_cc = [int(waypoint.pose.position.x/res) + map_origin[0], int(waypoint.pose.position.y/res) + map_origin[1], 0]
				print world_map.info
				if(world_map != None):
					print "so far so good"
					publishCells(world_data)
					map_cache = world_map
					start_cache = (pose.position.x, pose.position.y, pose.orientation.z)
					start_cc = [int(start_cache[0]/res) + map_origin[0], int(start_cache[1]/res) + map_origin[1], 0]
					generated_path, prev = aStar(curr_cc, goal_cc, map_cache, wall)
					rvizPath(generated_path, map_cache)
					path = genWaypoints(generated_path, start_cc, map_cache)
					waypoints_pub.publish(path)
					print "Published generated path to topic: [/lab4/waypoints]"
					print curr_cc, goal_cc
					if (goal_cc[0] - tolerance <= curr_cc[0] <= goal_cc[0] + tolerance and goal_cc[1] - tolerance <= curr_cc[1] <= goal_cc[1] + tolerance):
						at_goal = True
						break
		
		frontiers = checkerFrontier(map_cache, world_data)






if __name__ == '__main__':
	try:
		run()
		print "done"
	except rospy.ROSInterruptException:
		pass
