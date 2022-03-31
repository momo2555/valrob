#!/usr/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

def sendVelocityReq(vel):
    angular = vel.angular.z
    linear = vel.linear.x
    gcode = "G13 I{0:.2f} J{1:.2f}".format(linear, angular)
    odomReqPub.publish("gcode="+gcode)
    pass

if __name__ == "__main__":
    #init node
    rospy.init_node("motor_cmd")
    #motor control request topic sender
    odomReqPub = rospy.Publisher('robotcontrol_req', String, queue_size=10)
    #get the velocity
    rospy.Subscriber("cmd_vel", Twist, sendVelocityReq)
    rospy.spin()
