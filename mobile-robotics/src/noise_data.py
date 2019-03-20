#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
from math import atan2, pi, asin, tanh, atan, sin, cos, e, ceil, floor
from std_msgs.msg import Int64, String, Float64, Float64MultiArray
import numpy as np
from nav_msgs.msg import Odometry
import tf

noise_size=1000

class noise_class():
    def __init__(self):
        rospy.init_node('noise')
        self.noise_v = rospy.get_param('~noise_v')
        self.noise_s = rospy.get_param('~noise_s')
        self.wheel_velocity=Twist()
        self.noisev=np.random.normal(0, 0.01, noise_size)
        self.noisew=np.random.normal(0,0.01*pi/180, noise_size)
        self.noise_sonar=np.random.normal( 0, 0.00999999977648258, [noise_size, 16])
        self.flag=1
        self.i=0
        self.roll = 0    
        self.pitch = 0
        self.yaw = 0
        self.sonardata=np.zeros(16)
        self.odomdata=Odometry()

    def sonar0_callback(self, scan):      #sonar subscriber                                      
        self.sonardata[0]=scan.range

    def sonar1_callback(self, scan):
        self.sonardata[1]=scan.range

    def sonar2_callback(self, scan):
        self.sonardata[2]=scan.range

    def sonar3_callback(self, scan):
        self.sonardata[3]=scan.range
  
    def sonar4_callback(self, scan):
        self.sonardata[4]=scan.range

    def sonar5_callback(self, scan):
        self.sonardata[5]=scan.range

    def sonar6_callback(self, scan):
        self.sonardata[6]=scan.range

    def sonar7_callback(self, scan):
        self.sonardata[7]=scan.range

    def sonar8_callback(self, scan):
        self.sonardata[8]=scan.range

    def sonar9_callback(self, scan):
        self.sonardata[9]=scan.range

    def sonar10_callback(self, scan):
        self.sonardata[10]=scan.range

    def sonar11_callback(self, scan):
        self.sonardata[11]=scan.range

    def sonar12_callback(self, scan):
        self.sonardata[12]=scan.range

    def sonar13_callback(self, scan):
        self.sonardata[13]=scan.range

    def sonar14_callback(self, scan):
        self.sonardata[14]=scan.range

    def sonar15_callback(self, scan):
        self.sonardata[15]=scan.range

    def odometry_callback(self, scan):          #odometry subscriber, read wheel velocities and first odom scan only
        self.wheel_velocity=scan.twist.twist
        if self.flag:
            self.odomdata=scan
            quaternion = (
                scan.pose.pose.orientation.x,
                scan.pose.pose.orientation.y,
                scan.pose.pose.orientation.z,
                scan.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion) #transforming quaternions to euler 
            self.roll = euler[0]       
            self.pitch = euler[1]
            self.yaw = euler[2]
            self.flag=0

    def run(self):
        self.sub= rospy.Subscriber('/robot0/sonar_0',Range, self.sonar0_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_1',Range, self.sonar1_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_2',Range, self.sonar2_callback)       #defining the subscribers and publishers
        self.sub= rospy.Subscriber('/robot0/sonar_3',Range, self.sonar3_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_4',Range, self.sonar4_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_5',Range, self.sonar5_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_6',Range, self.sonar6_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_7',Range, self.sonar7_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_8',Range, self.sonar8_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_9',Range, self.sonar9_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_10',Range, self.sonar10_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_11',Range, self.sonar11_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_12',Range, self.sonar12_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_13',Range, self.sonar13_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_14',Range, self.sonar14_callback)
        self.sub= rospy.Subscriber('/robot0/sonar_15',Range, self.sonar15_callback)
        self.sub= rospy.Subscriber('/robot0/odom',Odometry, self.odometry_callback)
        pub_sonar = rospy.Publisher('/robot0/sonar_data', Float64MultiArray,queue_size=10)
        pub_odom = rospy.Publisher('/robot0/odom_drift',Odometry, queue_size=10)
        r=rospy.Rate(30)
        try:
            while not rospy.is_shutdown():
                self.sonardata+=self.noise_sonar[self.i][:]*self.noise_s           #add noise to sonars
                if abs(self.wheel_velocity.linear.x)>1e-6: #noise velocities if you move
                    self.wheel_velocity.linear.x+=self.noisew[self.i]*self.noise_v
                if abs(self.wheel_velocity.angular.z)>1e-6:
                    self.wheel_velocity.angular.z+=self.noisew[self.i]*self.noise_v
                
                self.odomdata.twist.twist=self.wheel_velocity
                self.i=(self.i+1)%noise_size  #increase counter

                self.yaw+=self.wheel_velocity.angular.z/30
                self.odomdata.pose.pose.position.x+=self.wheel_velocity.linear.x*cos(self.yaw)/30   #integrate velocities for odometry
                self.odomdata.pose.pose.position.y+=self.wheel_velocity.linear.x*sin(self.yaw)/30

                quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)  #back to quaternion
                self.odomdata.pose.pose.orientation.x = quaternion[0]
                self.odomdata.pose.pose.orientation.y = quaternion[1]
                self.odomdata.pose.pose.orientation.z = quaternion[2]
                self.odomdata.pose.pose.orientation.w = quaternion[3]

                pub_odom.publish(self.odomdata)             #publish data
                sonardata=Float64MultiArray()
                sonardata.data=self.sonardata.tolist()
                pub_sonar.publish(sonardata)
                r.sleep()
        except rospy.ROSInterruptException:                        
            pass                                            

if __name__ == '__main__':         
    noise = noise_class()                                                             
    try:
        noise.run()
    except rospy.ROSInterruptException:
        pass
