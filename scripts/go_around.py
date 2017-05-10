#!/usr/bin/env python
import rospy, math, sys, random
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from raspimouse_ros_2.msg import LightSensorValues

class GoAround():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.decision = rospy.Publisher('/decision',Twist,queue_size=100)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self,messages):
        self.sensor_values = messages

    def turn(self):
        rate = rospy.Rate(20)
        data = Twist()

        data.linear.x = 0.0
        data.angular.z = math.pi/2
        if random.uniform(0.0,1.0) > 0.5:
            data.angular.z *= -1

        while not rospy.is_shutdown():
            print self.sensor_values
            if self.sensor_values.sum_forward < 1000:
                return

            self.cmd_vel.publish(data)
            rate.sleep()

    def turn(self):
        rate = rospy.Rate(20)
        data = Twist()

        data.linear.x = 0.0
        data.angular.z = math.pi/2

        while not rospy.is_shutdown():
            if self.sensor_values.sum_forward < 200:
                return

            self.cmd_vel.publish(data)
            rate.sleep()

    def run(self):
        rate = rospy.Rate(20)
        data = Twist()

	stop_counter = 0
	
        while not rospy.is_shutdown():
            if stop_counter > 10:
                self.turn()
                stop_counter = 0
                continue

	    diff = self.sensor_values.right_side - self.sensor_values.left_side
            forward_max = max([self.sensor_values.right_forward,self.sensor_values.left_forward])

            data.linear.x = 0.15 * (1000 - forward_max)/1000.0
            data.angular.z = math.pi / 180.0 * (diff * 0.08) 

            if data.linear.x < 0.03 and math.fabs(data.angular.z) < 0.2:
                stop_counter += 1

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('go_around')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()
    GoAround().run()
