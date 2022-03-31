#!/usr/env python3
import time
from BMI160_i2c import Driver
import math
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

print('Initialisation du capteur ...')
sensor = Driver(0x69)
print('Done!')
sensor.setAccelOffsetEnabled(True)
sensor.setGyroOffsetEnabled(True)

sensor.autoCalibrateXAccelOffset(0)
sensor.autoCalibrateYAccelOffset(0)
sensor.autoCalibrateZAccelOffset(1)
sensor.autoCalibrateGyroOffset()
print(sensor.getFullScaleGyroRange())

#init ros
rospy.init_node("imu_driver")
imu_publisher = rospy.Publisher("imu_data", Imu, queue_size=10)

while True:
    data = sensor.getMotion6()
    gyro = {
        'x' : data[0] / 262.4 * math.pi / 180,
        'y' : data[1] / 262.4 * math.pi / 180,
        'z' : data[2] / 262.4 * math.pi / 180,
    }
    accel = {
        'x' : data[3] / 16383 * 9.81,
        'y' : data[4] / 16383 * 9.81,
        'z' : data[5] / 16383 * 9.81
    }
    print(gyro)
    print(accel)
    message = Imu()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "base_link"
    
    message.linear_acceleration = Vector3(accel['x'], accel['y'], accel['z'])
    message.angular_velocity = Vector3(gyro['x'], gyro['y'], gyro['z'])
    imu_publisher.publish(message)
            #attente 1 seconde
    time.sleep(0.1)