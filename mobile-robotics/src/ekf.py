#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
from math import atan2, pi, asin, tanh, atan, sin, cos, e, ceil, floor
from std_msgs.msg import Int64, String, Float64, Float64MultiArray
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
import math
from numpy.linalg import multi_dot, inv
from pprint import pprint as pp
import time
import matplotlib.pyplot as plt
from Astar_lib import *
import copy

class ekf_class():
    def __init__(self):
        rospy.init_node('ekf_node')
        self.sonar_coordinates=[ [10, 0],   [10, 5],  [10, 10], 
                                 [5, 10],   [0, 10],  [-5, 10],  [-10, 10],
                                 [-10, 5],  [-10, 0], [-10, -5], [-10, -10],                
                                 [-5, -10], [0, -10], [5, -10],  [10, -10],
                                 [10, -5]]
        self.sonar_thetas=[]
        for i in range(9):
            self.sonar_thetas.append(i*pi/8)                                                
        for i in range(7):
            self.sonar_thetas.append(-(7-i)*pi/8) 
        self.sonardata=[]
        self.yaw=0
        self.x=0
        self.y=0
        self.wheel_velocity=Twist()
        
        # Initialize actual map data
        self.map = []
        self.map_width = -1 
        self.map_height = -1
        self.map_resolution = -1

        #SIMULATION PARAMETERS#
        self.Rmax= 3              # sonar max range in m
        self.rov = 100            # visibility radius in cm
        self.th3db = 0.5          # half-width of the sensor beam in radians
        
        # Sleep and discretization rates
        self.sleep_rate = 30
        self.sleep_time= 1.0 / self.sleep_rate #self.sleep_rate

        # Initialize a - posteriori orientation variables
        # X_HAT will be initialized with proper values run() function
        # ... current values are [0 0 0]
        self.x_hat = [self.x, self.y, self.yaw]
        self.P_k = np.matrix(
            [
                [10, 0, 0], 
                [0, 10, 0],
                [0, 0, 100]
            ])

        # Initialize variance
        self.sigma2_dth = 2 * (pi / 180.0) ** 2  # deg ^ 2 -> rad ^ ^2 check 
        self.sigma2_D = 1.0 / (100 ** 2) # cm ^ 2 -> m ^ 2
        self.sigma2_ri = 100.0 / (100 ** 2) # cm ^ 2 -> m ^ 2

        # Initialize corrected odometry
        self.corrected_odometry = Odometry()
        self.corrected_odometry.child_frame_id = "robot0"
        self.corrected_odometry.header.frame_id = "map_static"

        # Transform broadcaster
        self.tb = tf.TransformBroadcaster()

        self.first_measurement = False
        self.first_time = -1

    def sonar_callback(self, scan):     #sonar subscriber                                       
        self.sonardata=scan.data

    def odometry_callback(self, scan):          #odometry subscriber
        self.wheel_velocity=scan.twist.twist
        quaternion = (
            scan.pose.pose.orientation.x,
            scan.pose.pose.orientation.y,
            scan.pose.pose.orientation.z,
            scan.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion) #transforming quaternions to euler 
        #self.roll = euler[0]       
        #self.pitch = euler[1]
        self.yaw = euler[2]
        self.x=scan.pose.pose.position.x
        self.y=scan.pose.pose.position.y

        self.first_measurement = True


    def map_callback(self, scan):
        """
            Update actual map information.
        """

        self.map = scan.data
        self.map_width = scan.info.width
        self.map_height = scan.info.height
        self.map_resolution = scan.info.resolution


    def print_matrix(self, A, name):
        print(name)
        for i in A:
            print(i)

    def sonarpredict(self, eps, x_prior):
        """
        ## TASK A ## 
        # write the function which will calculate the H matrix

            eps - allowed sensor measurement error
            Return - tuple (H, V, Rk, y(k+1), number of rows determined by number 
                     of adequate sensor measurements within given limit
                   - Rk is a diagonal covariance matrix with self.sigma2_ri values
                   - y(k+1) - all adequate REAL measurements
        """

        H = []
        measurement_difference = []
        for i in range(len(self.sonar_thetas)):#[0, 2, 4, 6, 8, 10, 12, 14]:

            # Caluculate nearest obstacle position
            nearest_obstacle = self.sonar_simulation(i)

            # Calculate sensor measurement
            # sim_measurement = y_prior
            sim_measurement = self.sensor_equation(x_prior, nearest_obstacle, 0)

            # Get actual sensor measurement 
            sensor_distance = math.sqrt(
                self.sonar_coordinates[i][0] ** 2 + 
                self.sonar_coordinates[i][1] ** 2) / 100 # CONVERT TO METERS !

            actual_measurement = self.sonardata[i] + sensor_distance

            #print("\n")
            #print(i, "-th sensor")
            #print("nearest_obstacle pos: ", nearest_obstacle)
            #print("sensor measurement: ", sim_measurement)
            #print("actual measurement: ", actual_measurement)

            # If difference is above the threshold do not use that sonar
            if (abs(sim_measurement - actual_measurement) > eps):
                #print(i, " sensor mesasurement discarder.")
                continue

            # Add new sonar row to the H matrix - izraz (3 - 17)
            const = 1.0 / sim_measurement
            new_row = [
                        ( x_prior[0] - nearest_obstacle[0] ) * const, 
                        ( x_prior[1] - nearest_obstacle[1] ) * const, 
                        0
                    ]

            H.append(new_row)
            measurement_difference.append(actual_measurement - sim_measurement)


        R_k = np.eye(len(H)) * self.sigma2_ri
        V = np.eye(len(H)) 

        return (np.matrix(H), np.matrix(V), np.matrix(R_k), (measurement_difference))

    def correct(self):
        ## TASK B ##

        # using the newest (noised) odometry and sonar data call self.sonarpredict and do the correction

        # Calculate current linear and angular velocity

        # Update discretization rate 
        discretization_time = -1 
        current_time = int(round(time.time() * 1000))
        if (self.first_time == -1):
            self.first_time = current_time
            discretization_time = self.sleep_time

        else:
            discretization_time = self.sleep_time + (current_time - self.first_time) / 1000.0
            self.first_time = current_time


        v_k = self.wheel_velocity.linear.x
        w_k = self.wheel_velocity.angular.z

        # x- (k + 1) --- prior estimate
        u_k = [v_k * discretization_time, w_k * discretization_time]
        x_prior = self.orientation_function(self.x_hat, u_k, [0, 0])
        
        # Calculate H / V / Rk+1 / measurement difference matrices
        matrices = self.sonarpredict(0.8, x_prior)
        H = matrices[0]
        V = matrices[1]
        R_K_1 = matrices[2] 
        y_diff = np.transpose(np.matrix(matrices[3])) # (y(k+1) - h(x_hat-(k+1), 0)) - 3-25 eqn

        # Calculate prior error covariances
        P_prior = self.calc_Pprior(self.x_hat, u_k)

        # If H is empty, use prior estimate
        if (H.size == 0): 
            self.x_hat = np.transpose(np.matrix(x_prior))
            self.P_k = P_prior
        
        else:
             # Do the correction step
            S_k_1 = (
                multi_dot( [H, P_prior, np.transpose(H)] ) + 
                multi_dot( [V, R_K_1, np.transpose(V)] )
                )
            K_k_1 = multi_dot( [P_prior, np.transpose(H), inv(S_k_1)] )
            
            # Calculate corrected P_k+1 and x_hat_k+1
            self.P_k = P_prior - multi_dot([K_k_1, S_k_1, np.transpose(K_k_1)])


            #print("Gain: ", np.dot(K_k_1, y_diff))
            self.x_hat = (
                    np.transpose(np.matrix(x_prior)) +
                    np.dot(K_k_1, y_diff) 
                    )

        self.corrected_odometry.pose.pose.position.x = np.asscalar(self.x_hat[0])
        self.corrected_odometry.pose.pose.position.y = np.asscalar(self.x_hat[1])
        quaternion = tf.transformations.quaternion_from_euler(0, 0, np.asscalar(self.x_hat[2]))
        self.corrected_odometry.pose.pose.orientation.x = quaternion[0]
        self.corrected_odometry.pose.pose.orientation.y = quaternion[1]
        self.corrected_odometry.pose.pose.orientation.z = quaternion[2]
        self.corrected_odometry.pose.pose.orientation.w = quaternion[3]

        #self.print_matrix(np.transpose(np.matrix(x_prior)), "X_prior")
        #self.print_matrix(self.x_hat, "x_hat_k+1")
        #print("u_k: ", u_k)
        #self.print_matrix(H, "H")

        #self.print_matrix(V, "V")
        #self.print_matrix(R_K_1, "R_k+1")
        #self.print_matrix(P_prior, "P_prior")
        #self.print_matrix(S_k_1, "S_k+1")
        #self.print_matrix(K_k_1, "K_k+1")
        #self.print_matrix(y_diff, "y_difference")
        #self.print_matrix(self.P_k, "P_k+1")

        #print("Wheel velocity: ", v_k)
        #print("Wheel angular vel: ", w_k)
        #print("u_k: ", u_k)
        #print("x prior: ", x_prior)

        #raw_input("__")
        

    def calc_Pprior(self, x_hat_k, u_k):
        """
            Returns prior calculation of error covariance matrix P-(k+1)
        """

        x_pos = x_hat_k[0]
        y_pos = x_hat_k[1]
        theta = x_hat_k[2]

        D = u_k[0]
        delta_theta = u_k[1]

        # F(x_hat_k, u_k, w_k) linearization

        # A matrix
        A = np.matrix(
            [
                [ 1, 0, -D * sin(theta + delta_theta) ], 
                [ 0, 1,  D * cos(theta + delta_theta) ],
                [ 0, 0, 1]
            ])

        # W matrix
        W = np.matrix(
            [
                [ -D * sin(theta + delta_theta), cos(theta + delta_theta) ],
                [  D * cos(theta + delta_theta), sin(theta + delta_theta) ],
                [1, 0]
            ])

        # Q matrix
        Q = np.matrix(
            [
                [self.sigma2_dth * delta_theta ** 2, 0], 
                [0, self.sigma2_D]
            ])

        return (
            multi_dot( [A, self.P_k, np.transpose(A)] ) + 
            multi_dot( [W, Q, np.transpose(W)] )
            )




    def orientation_function(self, x_hat_k, u_k, w_k):
        """
            f(x(k), w(k))
            Orientation function calculating updated a priori values x'_k+1

            x_k - 3x1 vector containing [x_k, y_k, theta_k]
            u_k - 2x1 vector containing [delta_theta_k, D]
            w_k - 2x1 vector containing [w_theta, w_D]

            returns - 3x1 vector containing updated odometry values
        """

        x_pos = x_hat_k[0]
        y_pos = x_hat_k[1]
        theta = x_hat_k[2]

       
        D = u_k[0]
        delta_theta = u_k[1]

        w_theta = w_k[0]
        w_D = w_k[1]

        # Construct output vector
        out = np.zeros(3)
        out[2] = theta + delta_theta + w_theta
        out[0] = x_pos + ( D + w_D ) * cos( out[2] )
        out[1] = y_pos + ( D + w_D ) * sin( out[2] )

        return np.transpose(out)


    def sensor_equation(self, x_prior, obstacle_pos, v_k):
        """
            Sensor equation for i-th sensor. 
            h(x(k+1), v(k+1))
            All inputs are scalar values.
            Returns - scalar value - distance from the obstacle
        """

        return (
            math.sqrt(
                (x_prior[0] - obstacle_pos[0])**2 + 
                (x_prior[1] - obstacle_pos[1])**2) 
            + v_k
            )


    def sonar_simulation(self, sonar_index):
        """
            Simulate a sonar hitting an obstacle. Search through
            the actual grid map for coordinates of the nearest obstacle
            in the sensor cone. 

            Returns - (x_pi, y_pi) tuple
                    - Coordinates of the neareast obstacle to the sonar
        """

        # Total angle of the current sonar sonar - take robot rotation in account !
        total_angle = self.sonar_thetas[sonar_index] + self.yaw

        # Current robot position 
        robotPos_x = self.x
        robotPos_y = self.y

        # Calculate current sensor distance from center of robot
        sensor_distance = math.sqrt(
            self.sonar_coordinates[sonar_index][0] ** 2 + 
            self.sonar_coordinates[sonar_index][1] ** 2) / 100 # CONVERT TO METERS !

        # Calculate current sonar coordinates 
        sonar_x = robotPos_x + sensor_distance * math.cos(total_angle)
        sonar_y = robotPos_y + sensor_distance * math.sin(total_angle)

        # Initial minimum values
        minimum_distance = 10e10
        x_min = -1
        y_min = -1

        # Go through all obstacles and find the closest one
        for obstacle_pos in self.obstacle_list:

            # Actual obstacle position
            actual_x = obstacle_pos[0]
            actual_y = obstacle_pos[1]
    
            # Calculate angle from sonar to grid tile
            distance_x = actual_x - sonar_x
            distance_y = actual_y - sonar_y

            # Distance to the obstacle
            obstacle_distance = math.sqrt(distance_x **2 + distance_y**2)

            # If outside of max sonar range discard
            # Discard from distance first, easier to calculate
            if (obstacle_distance > self.Rmax):
                continue

            distance_angle = math.atan2(distance_y, distance_x)
            current_theta = - total_angle + distance_angle 

            # Check if obstacle is inside i-th sonar cone
            if (abs(current_theta) > self.th3db):
                continue

            
            # If current obstacle distance is less than minimimum found distance
            # ... update the minimum distance and coordinates
            if (obstacle_distance < minimum_distance):
                minimum_distance = obstacle_distance
                x_min = actual_x
                y_min = actual_y 
                    

        #print("Sonar-sim - minDistance: ", minimum_distance)
        return (x_min, y_min)


    def get_obstacle_list(self, o_map):
        """
            Returns a list of obstacle grid positions.
        """

        obstacle_list = []
        for i, value in enumerate(o_map):

            if (value <= 0):
                continue

            grid_y = i // self.map_width
            grid_x = i - grid_y * self.map_width

            # Calculate actual map position of the grid tile center
            # + self.map_resolution / 2 - CENTER of the grid tile
            actual_x = grid_x * self.map_resolution + self.map_resolution/2
            actual_y = grid_y * self.map_resolution + self.map_resolution/2

            obstacle_list.append([actual_x, actual_y])

        return obstacle_list

    def initialize_my_map(self):
        """
            Initialize my own map.
        """

        self.my_map = np.load('resources/my_map_final.npy')
        self.my_map = np.reshape(self.my_map, (150,155)).T
        width = np.shape(self.my_map)[0]
        height = np.shape(self.my_map)[1]

        for i in range(width):
            for j in range(height):

                if (self.my_map[i][j] < 70):
                    self.my_map[i][j] = 0

                else:
                    self.my_map[i][j] = 1

        self.my_map = self.expand_obstacle_map(self.my_map, 1)
        show_path(self.my_map)
        
        # Doktor zaboravio promijeniti sirinu i visinu ---- DOTURREEEEEEEE
        self.map_width = width
        self.map_height = height
        self.map_resolution = 0.1

        self.my_map_arr = []
        for j in range(height):
            for i in range(width):
                self.my_map_arr.append(self.my_map[i][j])


    def expand_obstacle_map(self, in_map, square_count):
        """
            Expand map obstacles by 2 squares around each obstacle
        """

        # Copy map 
        expanded_map = copy.deepcopy(in_map)

        max_width = in_map.shape[0]
        max_height = in_map.shape[1]

        for i in range(max_width):
            for j in range(max_height):

                # If empty continue
                if (in_map[i][j] == 0):
                    continue

                indices_list = []
                for i_add in range(-square_count, square_count+1):
                    for j_add in range(-square_count, square_count+1):

                        i_pos =  i + i_add if ((i+i_add) >= 0 and (i+i_add) < max_width) else None
                        j_pos =  j + j_add if ((j+j_add) >= 0 and (j+j_add) < max_height) else None

                        if (i_pos and j_pos):
                            indices_list.append((i_pos, j_pos))

                # Make obstacles on new indices
                for i_, j_ in indices_list:
                    expanded_map[i_][j_] = 1

        return expanded_map



    def run(self):

        #defining the subscribers 
        self.sub= rospy.Subscriber('/robot0/sonar_data',Float64MultiArray, self.sonar_callback)       
        self.sub= rospy.Subscriber('/robot0/odom_drift',Odometry, self.odometry_callback)
        
        # Subscribe to the actual map topic - used for simulating sonar hits 
        self.sub= rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Corrected odometry publisher
        self.corr_publisher = rospy.Publisher("/robot0/odom_corr", Odometry, queue_size=10)

        # MSE publisher 
        self.mse_publisher = rospy.Publisher("/robot0/mse", Float64, queue_size=10)

        # Initialize obstacle list 
        self.obstacle_list = []
        self.tb.sendTransform((1, 2, 0),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "robot0",  # child
                              "map_static"  # parent
                              )

        r=rospy.Rate(self.sleep_rate)
        try:
            # Wait for actual map to start getting values
            while (len(self.map) == 0):
                print("Waiting for map...")

            # Initialize obstacle list
            print("Initialize obstacles list")
            
            # Uncomment to use own map
            #self.initialize_my_map()
            self.obstacle_list = self.get_obstacle_list(self.map) #self.my_map_arr - to use own map

            # Wait for first measurement
            while (not self.first_measurement):
                print("Waiting for first measurement...")

            print("initialize X_hat")
            self.x_hat = [[self.x], [self.y], [self.yaw]]
            self.x_hat = np.matrix(self.x_hat)

            print(len(self.obstacle_list))

            raw_input("Press any key..")

            while not rospy.is_shutdown():

                self.correct()
                
                # Publish corrected odometry
                self.corr_publisher.publish(self.corrected_odometry)

                # Publish MSE 
                self.mse_publisher.publish(np.trace(self.P_k))

                r.sleep()

        except rospy.ROSInterruptException:                        
            pass                                            

if __name__ == '__main__':         
    ekf = ekf_class()                                                             
    try:
        ekf.run()
    except rospy.ROSInterruptException:
        pass
