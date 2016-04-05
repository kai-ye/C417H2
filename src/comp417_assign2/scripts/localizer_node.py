#! /usr/bin/env python
import rospy

class Localizer:
	def __init__(self):
		rospy.init_node("localizer", anonymous = True)
		
		self.pub = rospy.Publisher()
		self.pub_rate = rospy.Rate(10)

		rospy.Subscriber()


	def shutdown(self):
		self.pub.publish(
		rospy.sleep(1)



if __name__ == "__main__":
	try:
		Localizer()
	except rospy.ROSInterruptException:
		pass
