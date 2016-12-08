#!/usr/bin/env python

import rospy, tf, numpy, math, roslib, time

from newastar import aStar
from nav_msgs.msg import GridCells, Path, Odometry, OccupancyGrid
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from lab4_final import *
from FrontierChecker import checkerFrontier, checkClosestFrontier






def initial():
	rotate(180)
	rotate(180)
		


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

	initial()

	frontiers = checkerFrontier(map_data)
	
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
		map_origin = (int(-origin_cache[0]/res), int(-origin_cache[1]/res))
		start_cc = [int(start_cache[0]/res) + map_origin[0], int(start_cache[1]/res) + map_origin[1], 0]
		goal_cache = checkClosestFrontier(frontiers, start_cc)
		goal_cc = [int(goal_cache[0]/res) + map_origin[0], int(goal_cache[1]/res) + map_origin[1], 0]
		
		# Running A*
		print start_cc, goal_cc
		rospy.sleep(1)
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
			for waypoint in path.poses:
				print "Navigating to waypoint: ", tmp_wp_ctr
				tmp_wp_ctr += 1

				# Replanning
				
				navToPose(waypoint)
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
					publishFrontier(prev, map_cache)
					path = genWaypoints(generated_path, start_cc, map_cache)
					waypoints_pub.publish(path)
					print "Published generated path to topic: [/lab4/waypoints]"
					print curr_cc, goal_cc
					if (goal_cc[0] - tolerance <= curr_cc[0] <= goal_cc[0] + tolerance and goal_cc[1] - tolerance <= curr_cc[1] <= goal_cc[1] + tolerance):
						at_goal = True
						break
		
		frontiers = checkerFrontier(map_data)






if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
