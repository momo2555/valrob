import rospy
from geometry_msgs.msg import Twist, Vector3

def sendVelocityReq(vel: Twist):
    angular = vel.angular.z
    linear = vel.linear.x
    gcode = "G13 I{0:.2f} J{1:.2f}".format(linear, angular)
    odomReqPub.publish("gcode="+gcode)
    pass

if __name__ == "__main__":
    #motor control request topic sender
    odomReqPub = rospy.Publisher('robot_consign', Twist, queue_size=10)
    #get the velocity
    rospy.Subscriber("robotcontrol_req", Twist, sendVelocityReq)
    rospy.spin()
