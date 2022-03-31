#!/usr/env python3
import serial
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import time
from tf.transformations import quaternion_from_euler



class MotSerial(serial.Serial):
    def __init__(self, serialName):
        serial.Serial.__init__(self, serialName, 115200, timeout=0)
        self.__serialBusy = False
    
    def sendGcode(self, gcode):
        sended = False
        while not sended:
            time.sleep(0.01)
            self.write(gcode.encode("utf8"))
            print(gcode)
            sended = True
                

    def sendWithResponse(self, gcode):
        getit = True
        #envoie de la commande
        self.write(gcode.encode("utf8"))
        
        #reccuperation de la valeur des encodeurs
        i = 0
        while getit:
            by = self.readline()
            sr=by.decode('utf-8')
            getit = not "R="in sr #a finir
            i+=1
            if i >2:
                getit = False
                sr = "response_failed"
           
                    
        return sr.replace("R=", "")


def execRequest(req):
    if req.data == "odom":
        #send the velocity in odom topic
        res = motSer.sendWithResponse("M404 \n")
        res = res.replace('\n', '').replace('(', '').replace(')', '').split(';')
        if(len(res)==2):
            publishVelocity(float(res[0]), float(res[1]))
        
    elif "gcode=" in req.data:
        #send gcode
        gcode = req.data.replace("gcode=", "")
        gcode = gcode + " \n"
        motSer.sendGcode(gcode)
        


def publishVelocity(linear, angular):
    odomQuat = quaternion_from_euler(0, 0, 0)
    message = Odometry()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "odom"
    message.pose.pose = Pose(Point(0, 0, 0), Quaternion(*odomQuat))
    
    message.child_frame_id = "base_link"
    message.twist.twist = Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular))
    print(message)
    encOdom.publish(message)


if __name__ == "__main__":
    #define the serial motor 
    serialName = rospy.get_param("motor_controller_port", "/dev/ttyACM1")
    motSer = MotSerial(serialName)

    #execution server position
    rospy.init_node('motorcontrol')

    #initit topic to send encoders odom
    encOdom = rospy.Publisher("enc_odom", Odometry, queue_size=10)

    #montorcontrol message request
    rospy.Subscriber("robotcontrol_req", String, execRequest)
    rospy.spin()

