#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
import math
import time
from geometry_msgs.msg import PoseArray

class predict_loc():
	"""docstring for predict_loc"""
	def __init__(self):

		rospy.init_node('predict_motion')

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		
		self.new_loc_pub = rospy.Publisher('new_loc', String, queue_size = 10)
		self.acceleration_pub = rospy.Publisher('acceleration', Float32, queue_size = 10)
		self.velocity_pub = rospy.Publisher('velocity', Float32, queue_size = 10)
		
		self.runner_x = 0
		self.runner_y = 0
		self.runner_z = 0

		self.runner_x_prev = 0
		self.runner_y_prev = 0
		self.runner_z_prev = 0

		self.runner_x_new = 0
		self.runner_y_new = 0
		self.runner_z_new = 0

		self.last_time = 0
		self.frame_time = 0.2

		self.dist = 0.0
		self.velocity_final = 0.0
		self.velocity_initial = 0.0
		self.acceleration = 0.0

		self.startVar = 1


	def distance_calculation(self, x2, x1, y2, y1):
		distance = math.sqrt( ( (x2 - x1) ** 2) + ( (y2 - y1) ** 2) )
		return distance
		
	def compute_mot(self):
		#To allow subscriber function get values, else runner variables stay 0 at initial iterations.
		#Affects dist
		rospy.sleep(1)
		while 1:
			self.current_time = time.time()
			time_change = (self.current_time - self.last_time)
			if (time_change >= self.frame_time):
				
				# #Find dt
				self.dt = round(time_change,2) 

				#Since initial value of runner_prev is 0
				#For first iteration get current runner loaction
				if self.startVar == 1:
					self.runner_x_prev = self.runner_x
					self.runner_y_prev = self.runner_y
					self.runner_z_prev = self.runner_z
					self.startVar = 0
				
				# #Calculate distance between the two points
				self.dist = self.distance_calculation(self.runner_x, self.runner_x_prev, self.runner_y, self.runner_y_prev)
				
				if self.dist < 0.1:
					self.dist = 0.0

				#Velocity
				self.velocity_final = round((self.dist/self.dt),1)
				
				#acceleration = (change in velocity)/time
				self.acceleration = (self.velocity_final - self.velocity_initial) / self.dt

				#maintain velocity value
				self.velocity_initial = self.velocity_final
				
				#calculate next probable position
				self.new_dist = (self.velocity_final * self.dt) + (0.5 * self.acceleration * self.dt * self.dt)

				if(self.new_dist <= .16 and self.new_dist >= -.16):
					self.new_dist = 0.0

				#As per quadrant system
				if self.runner_x > self.runner_x_prev:
					self.runner_x_new = self.runner_x + self.new_dist
				elif self.runner_x < self.runner_x_prev:
					self.runner_x_new = self.runner_x - self.new_dist

				if self.runner_y > self.runner_y_prev:
					self.runner_y_new = self.runner_y + self.new_dist
				elif self.runner_y < self.runner_y_prev:
					self.runner_y_new = self.runner_y - self.new_dist

				self.new_loc_pub.publish(str(round(self.runner_x_new,2)) + "," + str(round(self.runner_y_new,2)))
				self.velocity_pub.publish(round(self.velocity_final,2))
				self.acceleration_pub.publish(round(self.acceleration,2))

				self.runner_x_prev = self.runner_x
				self.runner_y_prev = self.runner_y
				self.runner_z_prev = self.runner_z
				
				self.last_time = self.current_time

	#subscriber
	def get_pose(self, value):
		self.runner_x = round(value.poses[0].position.x,2)
		self.runner_y = round(value.poses[0].position.y,2)
		self.runner_z = round(value.poses[0].position.z,2)


if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = predict_loc()
		test.compute_mot()
		rospy.spin()
