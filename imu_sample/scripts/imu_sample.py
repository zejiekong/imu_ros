#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu 
from std_msgs.msg import Header

def publisher():
    pub = rospy.Publisher("imu_true",Imu,queue_size=10)
    rospy.init_node("imu_sample",anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        imu_msgs = Imu()
        h = Header()
        h.stamp = rospy.Time.now()
        imu_msgs.header = h
        imu_msgs.angular_velocity.x = 3
        imu_msgs.angular_velocity.y = 3
        imu_msgs.angular_velocity.z = 3
        imu_msgs.linear_acceleration.x = 3
        imu_msgs.linear_acceleration.y = 3
        imu_msgs.linear_acceleration.z = 3
        pub.publish(imu_msgs)
        rate.sleep

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass