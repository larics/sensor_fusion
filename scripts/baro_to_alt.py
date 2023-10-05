#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import FluidPressure
from nav_msgs.msg import Odometry

BASE_TEMPERATURE = 296.15
K1 = 153.8462
K2 = 0.190259

class Baro2Alt():
	def __init__(self):
		rospy.Subscriber("mavros/imu/static_pressure", FluidPressure, self.baro_callback)
		self.pub_height = rospy.Publisher('height_from_pressure', Odometry, queue_size=10)
		self.base_pressure = 0
		self.height = 0
		self.first_pressure_callback = 1

	def baro_callback(self, msg):
		pressure = msg.fluid_pressure
		if self.first_pressure_callback == 1:
			self.base_pressure = pressure
			self.first_pressure_callback = 0
		self.height = self.baro_to_alt(pressure, self.base_pressure)
		print(self.height)
		height_msgs = Odometry()
		height_msgs.header = msg.header
		height_msgs.pose.pose.position.z = self.height
		self.pub_height.publish(height_msgs)

	def baro_to_alt(self, pressure, base_pressure):
		return K1*BASE_TEMPERATURE*(1.0-math.exp(K2*math.log(pressure/base_pressure)))

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('baro_to_alt', anonymous=True, log_level=rospy.WARN)
		altitude = Baro2Alt()
		altitude.run()
	except rospy.ROSInterruptException:
		pass