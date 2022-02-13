#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
from imu_gen_model import imu_model
from std_msgs.msg import Header

class imu_noise_model:
    def __init__(self):
        self.imu = imu_model(rospy.get_param("accuracy"),rospy.get_param("fs"))
        self.pub = rospy.Publisher("imu_real",Imu,queue_size=10)
        self.rate = None
    
    def imu_callback(self,data):
        imu_msg = Imu()
        h = Header()
        h.stamp = rospy.Time.now()
        imu_msg.header = h
        #real_orient = data.orientation
        real_ang_vel = data.angular_velocity
        real_linear_acc = data.linear_acceleration
        real_accel = self.imu.accel_gen(np.array([real_linear_acc.x,real_linear_acc.y,real_linear_acc.z]))
        real_gyro = self.imu.gyro_gen(np.array([real_ang_vel.x,real_ang_vel.y,real_ang_vel.z]))
        imu_msg.linear_acceleration.x = real_accel[0]
        imu_msg.linear_acceleration.y = real_accel[1]
        imu_msg.linear_acceleration.z = real_accel[2]
        imu_msg.angular_velocity.x = real_gyro[0]
        imu_msg.angular_velocity.y = real_gyro[1]
        imu_msg.angular_velocity.z = real_gyro[2]
        self.pub.publish(imu_msg)
        self.rate.sleep()
    
    def listener(self):
        rospy.init_node("imu_noise",anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.Subscriber("imu_true",Imu,self.imu_callback)
        rospy.spin()
        
    def run(self):
        self.imu.set_sensor_bias_param()
        self.listener()
        
if __name__ == "__main__":
    imu_obj = imu_noise_model()
    imu_obj.run()
    
