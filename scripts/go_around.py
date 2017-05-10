#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class GoAround():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self,messages):
        self.sensor_values = messages

    def turn180(self):
        rate = rospy.Rate(20)
        data = Twist()

        data.linear.x = 0.0
        data.angular.z = math.pi

        while not rospy.is_shutdown():
            if self.sensor_values.sum_forward < 100:
                return

            self.cmd_vel.publish(data)
            rate.sleep()
    
    def run(self):
        rate = rospy.Rate(20)
        data = Twist()

	stop_counter = 0
	
        while not rospy.is_shutdown():
            if stop_counter > 10:
                self.turn180()
                stop_counter = 0
    
	    diff = self.sensor_values.right_side - self.sensor_values.left_side
            data.linear.x = 0.1 * (2500 - self.sensor_values.sum_forward)/2500.0
            data.angular.z = math.pi / 180.0 * (diff * 0.08) 

            if math.fabs(data.linear.x) < 0.03 and \
               math.fabs(data.angular.z) < 0.1:
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
