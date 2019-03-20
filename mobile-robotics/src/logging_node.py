import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
from std_msgs.msg import Int64, String, Float64, Float64MultiArray

class logging_node(object):
    
    def __init__(self):
        rospy.init_node('logging_node')
        self.first_measurement1 = False
        self.first_measurement2 = False
        self.first_measurement3 = False
        self.first_measurement4 = False

    def odom_cb(self, scan):
        quaternion = (
            scan.pose.pose.orientation.x,
            scan.pose.pose.orientation.y,
            scan.pose.pose.orientation.z,
            scan.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion) #transforming quaternions to euler 
        #self.roll = euler[0]       
        #self.pitch = euler[1]
        self.yaw_odom = euler[2]
        self.x_odom=scan.pose.pose.position.x
        self.y_odom=scan.pose.pose.position.y 
        
        self.first_measurement1 = True
        
    def odom_drift_cb(self, scan):
        quaternion = (
            scan.pose.pose.orientation.x,
            scan.pose.pose.orientation.y,
            scan.pose.pose.orientation.z,
            scan.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion) #transforming quaternions to euler 
        #self.roll = euler[0]       
        #self.pitch = euler[1]
        self.yaw_odom_drift = euler[2]
        self.x_odom_drift = scan.pose.pose.position.x
        self.y_odom_drift = scan.pose.pose.position.y

        self.first_measurement2 = True

    def mse_cb(self,scan):
        self.mse = scan.data

        self.first_measurement4 = True

    def odom_corr_cb(self, scan):
        quaternion = (
            scan.pose.pose.orientation.x,
            scan.pose.pose.orientation.y,
            scan.pose.pose.orientation.z,
            scan.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion) #transforming quaternions to euler 
        #self.roll = euler[0]       
        #self.pitch = euler[1]
        self.yaw_odom_corr = euler[2]
        self.x_odom_corr = scan.pose.pose.position.x
        self.y_odom_corr = scan.pose.pose.position.y

        self.first_measurement3 = True

    def log_all(self):

        with open("resources/logs/log_odom.txt", "a") as file:
            file.write("{}, {}, {}\n".format(self.x_odom, self.y_odom, self.yaw_odom))

        with open("resources/logs/log_odom_drift.txt", "a") as file:
            file.write("{}, {}, {}\n".format(self.x_odom_drift, self.y_odom_drift, self.yaw_odom_drift))
        
        with open("resources/logs/log_odom_corr.txt", "a") as file:
            file.write("{}, {}, {}\n".format(self.x_odom_corr, self.y_odom_corr, self.yaw_odom_corr))

        with open("resources/logs/log_mse.txt", "a") as file:
            file.write("{}\n".format(self.mse))



    def run(self):

        self.sub= rospy.Subscriber('/robot0/odom',Odometry, self.odom_cb)
        self.sub= rospy.Subscriber('/robot0/odom_drift',Odometry, self.odom_drift_cb)
        self.sub= rospy.Subscriber('/robot0/odom_corr',Odometry, self.odom_corr_cb)
        self.sub= rospy.Subscriber('/robot0/mse', Float64, self.mse_cb)
        r = rospy.Rate(10)
        try:

            # Wait for the first measurement from odometry callback
            while (not(self.first_measurement1 and 
                self.first_measurement2 and
                self.first_measurement3 and
                self.first_measurement4)):

                print("Waiting for first measurement")
                continue

            while not rospy.is_shutdown():
                self.log_all()
                r.sleep()

        except rospy.ROSInterruptException:                  
            pass 

if __name__ == '__main__':         
   
    ln = logging_node()

    try:
        ln.run()
    except rospy.ROSInterruptException:
        pass