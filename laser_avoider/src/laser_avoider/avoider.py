#!/usr/bin/env python3

#from Avoider import Avoider

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
from datetime import datetime
import math

def sign(v):
	if v < 0:
		return -1
	if v > 0:
		return 1
	return 0

class Avoider():
	''' This class provides simple obstacle avoidance functionalities to a ROS robot '''

	# This dict keeps track of the distance measures for each region
	Regions_Report = {
	                     "front_C": [], "front_L": [], "left_R" : [],
	                     "left_C" : [], "left_L" : [], "back_R" : [],
	                     "back_C" : [], "back_L" : [], "right_R": [],
	                     "right_C": [], "right_L": [], "front_R": [],
	                 }
	# These are the costs to deviate from each region to the goal region (front_C)
	Regions_Distances = {
	                     "front_C":  0, "front_L":  1, "left_R" :  2,
	                     "left_C" :  3, "left_L" :  4, "back_R" :  5,
	                     "back_C" :  6, "back_L" : -5, "right_R": -4,
	                     "right_C": -3, "right_L": -2, "front_R": -1,
	                 	}

	def __init__(self, vel_obj, obstacle_threshold=1.75, 
				       regional_angle=30, normal_lin_vel=1, 
				       trans_lin_vel=-0.09, trans_ang_vel=1.75):
		'''
		:param vel_obj           : Velocity object; will contain velocity commands(data); Twist()
		:param obstacle_threshold: Objects a this distance or below are considered obstacles
		:param regional_angle    : The angle on which each region extends
		:param normal_lin_vel    : When there's no obstacles, the robot will move with this linear velocity
		:param trans_lin_vel     : After detecting an obstacle, the robot will back up (negative) 
								   while rotating to help in case it can't perform a stationary rotation
		:param trans_ang_vel 	 : The robot always rotates with the same value of angular velocity
		'''
		self.vel_obj        = vel_obj
		self.OBSTACLE_DIST  = obstacle_threshold
		self.REGIONAL_ANGLE = regional_angle
		self.NORMAL_LIN_VEL = normal_lin_vel
		self.TRANS_LIN_VEL  = trans_lin_vel
		self.TRANS_ANG_VEL  = trans_ang_vel
		self.last_user_input = datetime.now()
		self.waiting = True
		self.thrasing_adj = 1
		self.last_steering_direction = 0
		self.last_dir_change = rospy.get_time()
		self.thrasing_start = rospy.get_time()

	def user_input(self, twist_msg):
		
		if not self.waiting:
			print("pausing avoidance.")
			self.waiting = True
		self.last_user_input = datetime.now()
			
		

	def indentify_regions(self, scan):
		'''
		:param scan: Scan object that contains the lidar data 
		'''

		#print(scan)
		#print("")
		
		# This list keeps track of the order in which the regions' readings are obtained
		REGIONS = [
		             "front_C", "front_L", "left_R" ,
		             "left_C" , "left_L" , "back_R" ,
		             "back_C" , "back_L" , "right_R",
		             "right_C", "right_L", "front_R",
				  ]

		# The front central region necessitate getting the last and first 15 points of the ranges
		#print(scan.ranges)
		#print(" ")
		intermediary = scan.ranges[:int(self.REGIONAL_ANGLE/2)]\
					 + scan.ranges[(len(scan.ranges)-1)*int(self.REGIONAL_ANGLE/2):]
		#print(intermediary)
		#print("")
		self.Regions_Report["front_C"] = [x for x in intermediary if x <= self.OBSTACLE_DIST and x != 'inf']
		
		# Enumerate all regions but the first
		for i, region in enumerate(REGIONS[1:]):
			# Only objects at a distance less than or equal to the threshold are considered obstacles
			self.Regions_Report[region] = [x for x in scan.ranges[self.REGIONAL_ANGLE*i:self.REGIONAL_ANGLE*(i+1)]\
												   if x <= self.OBSTACLE_DIST and x != 'inf']
		#print(self.Regions_Report)
		#print(" ")


	def avoid(self):
		act, ang_vel = self._clearance_test()
		# If act is False, and_vel is set to 0
		#print(act, ang_vel)
		self._steer(act, (act*ang_vel))

		#if self._steer triggers 3 times in 2 seconds, self._steer(act, -(act*ang_vel))



		return self.vel_obj

	def _clearance_test(self):

		goal = "front_C"
		closest = 10e6
		regional_dist = 0
		maxima = {"destination": "back_C", "distance": 10e-6}
		for region in self.Regions_Report.items():
			regional_dist = abs(self.Regions_Distances[region[0]]-self.Regions_Distances[goal])
			#if there're no obstacles in that region
			if not len(region[1]):
				#check if it's the cheapest option
				if (regional_dist < closest):
					closest = regional_dist
					maxima["distance"] = self.OBSTACLE_DIST
					maxima["destination"] = region[0]
			#check if it's the clearest option
			elif(max(region[1]) > maxima["distance"]):
				maxima["distance"] = max(region[1])
				maxima["destination"] = region[0]
		#calculate the cost to the chosen orientation
		regional_dist = self.Regions_Distances[maxima["destination"]]-self.Regions_Distances[goal]
		
		# Return whether to act or not, and the angular velocity with the appropriate sign
		#print(closest)
		return (closest != 0), (regional_dist/[abs(regional_dist) if regional_dist != 0 else 1][0])*self.TRANS_ANG_VEL

	def _steer(self, steer=False, ang_vel=0):
		'''
		:param steer  : Whether to avoid and obstacle or keep on going straigt
		:param ang_vel: The angular velocity of the robot
		'''
		#print("steering", ang_vel)
	
		#time = datetime.now()
		time = rospy.get_time()


		if not steer:
			self.vel_obj.linear.x = self.NORMAL_LIN_VEL
		else:
			self.vel_obj.linear.x = self.TRANS_LIN_VEL


		if (time-self.thrasing_start) < 4 :
			ang_vel = math.copysign(ang_vel, self.vel_obj.angular.z)
		else:
			ang_vel *= self.thrasing_adj
			steering_direction = sign(ang_vel)	#(ang_vel !=0)
			if (steering_direction != self.last_steering_direction):
				self.last_steering_direction = steering_direction
				if steering_direction:
					if (time-self.last_dir_change) < 1:
						self.thrasing_adj *= -1
						self.thrasing_start = time
						print("starting emergency turn.", self.thrasing_adj)
				self.last_dir_change = time
			#on the off chance it gets smashed face first directly into a wall, this will force evarobot to backup.
			elif (ang_vel != 0) and time-self.last_dir_change > 4:
				print("backup, backup")
				self.vel_obj.linear.x = -0.5
				#one of the linears or something go backwards. probably -x
		#if time-self_thasing_start and turn is constant for 10 seconds, set linear.




			
		self.vel_obj.linear.y  = 0
		self.vel_obj.linear.z  = 0
		self.vel_obj.angular.x = 0
		self.vel_obj.angular.y = 0
		self.vel_obj.angular.z = ang_vel 

def main():

    vel = Twist()
  
    # Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    print("making node")

    # Instanciate our avoider object
    avoider = Avoider(vel)
  
 
    # Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/lidar", LaserScan, avoider.indentify_regions)
    
    rospy.Subscriber("/user_input", Twist, avoider.user_input) #subscribing to our new user_input topic. 
    
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(10) #10Hz
    
    #keep running while the ros-master isn't shutdown
    while not rospy.is_shutdown():
        if avoider.waiting:
        	if (datetime.now() - avoider.last_user_input).total_seconds() > 5:
        		avoider.waiting = False
        		print("auto avoidance engaged, commander")
        else:
	        vel = avoider.avoid()
        	pub.publish(vel)
        
        rate.sleep()

#if __name__ == "__main__":
#    try:
#        main()
#    except rospy.ROSInterruptException:
#        pass
