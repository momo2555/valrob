import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('odomRequest')
    odomReqPub = rospy.Publisher('robot_consign', String, queue_size=10)
    frq = rospy.Rate(100)
    while not rospy.is_shutdown():
        odomReqPub.publish("odom")
        frq.sleep()
