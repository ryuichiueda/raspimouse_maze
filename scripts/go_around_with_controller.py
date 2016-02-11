#!/usr/bin/env python
import rospy, math, sys, random
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from raspimouse_ros_2.msg import LightSensorValues
from raspimouse_maze.msg import Decision

class GoAround():
    def __init__(self):
        self.decision = rospy.Publisher('/decision',Decision,queue_size=100)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

        self.cmd_vel = Twist()
        rospy.Subscriber('/cmd_vel', Twist, self.callback2)

    def callback(self,messages):
        self.sensor_values = messages

    def callback2(self,messages):
        self.cmd_vel = messages

    def output_decision(self,d,s):
        dc = Decision()

        dc.left_side = s.left_side
        dc.right_side = s.right_side
        dc.left_forward = s.left_forward
        dc.right_forward = s.right_forward
        dc.linear_x = d.linear.x
        dc.angular_z = d.angular.z

        self.decision.publish(dc)

    def run(self):
        rate = rospy.Rate(20)
        data = Twist()

	stop_counter = 0
	
        while not rospy.is_shutdown():
            s = self.sensor_values
            data = self.cmd_vel

            if stop_counter > 10:
                stop_counter = 0
                continue

            if data.linear.x < 0.03 and math.fabs(data.angular.z) < 0.2:
                stop_counter += 1

            self.output_decision(data,s)
            #self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('go_around')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()
    GoAround().run()
