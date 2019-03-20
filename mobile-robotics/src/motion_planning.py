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
from Astar_lib import astar, show_path
import matplotlib.pyplot as plt
import copy 

map_width=1550    #real map dimensions in cm
map_height=1492
resolution=10.0   #grid tiles are 10x10 cm

class motion_planning(object):

    def __init__(self):
        rospy.init_node('motion_planning')

        # Robot kinematic restrictions
        self.omega_max = 0.1 #0.05   
        self.omega_min = -0.1 #-0.05
        self.v_max = 0.8 #0.5
        self.v_min = -0.8 #-0.5

        # Motion regulator constants
        self.k_ro = 3.0 / 10
        self.k_alpha = 8.0 / 10
        self.k_beta = -1.5 / 10

        # Map resolution 
        self.map_resolution = resolution/100
        self.map_width = int(ceil(map_width/resolution))
        self.map_height = int(ceil(map_height/resolution))

        self.first_measurement = False
        self.rospy_rate = 30

        # Criterium for reaching point
        self.epsilon = self.map_resolution / 4.0

        # Control values
        self.control_values = Twist()


    def initialize_working_map(self):
        # Initialize working map
        # TODO - expand obstacle borders with robot size
        self.working_map = np.load("resources/example_map.npy").T
        self.working_map[self.working_map > 0] = 1 
        self.working_map = self.expand_obstacle_map(2)

    def initialize_my_map(self):
        """
            Initialize my own map.
        """

        self.working_map = np.load('resources/my_map_final.npy')
        self.working_map = np.reshape(self.working_map, (150,155))

        for i in range(np.shape(self.working_map)[0]):
            for j in range(np.shape(self.working_map)[1]):

                if (self.working_map[i][j] < 50):
                    self.working_map[i][j] = 0

                else:
                    self.working_map[i][j] = 1

        self.working_map = self.working_map.T
        self.working_map = self.expand_obstacle_map(2)

    def odometry_callback(self, scan):
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

    def L2distance(self, point_1, point_2):
        """
            Return L2 distance between 2 given points.
        """

        return (
                math.sqrt(
                    (point_1[0] - point_2[0])**2 +
                    (point_1[1] - point_2[1])**2
                    )
                )


    def expand_obstacle_map(self, square_count):
        """
            Expand map obstacles by 2 squares around each obstacle
        """

        # Copy map 
        expanded_map = copy.deepcopy(self.working_map)

        max_width = self.working_map.shape[0]
        max_height = self.working_map.shape[1]

        for i in range(max_width):
            for j in range(max_height):

                # If empty continue
                if (self.working_map[i][j] == 0):
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


    def move_robot_to_point(self, x_end, y_end):
        
        # Calculate actual end coordinates
        x_end_actual = x_end * self.map_resolution + self.map_resolution / 2.0
        y_end_actual = y_end * self.map_resolution + self.map_resolution / 2.0

        # Check if position is already reached
        if (
            self.L2distance(
                [x_end_actual, y_end_actual], 
                [self.x, self.y]
            ) < self.epsilon):
            print("Robot already at position: ", x_end, y_end)
            return

        # Convert end point coordinates to grid indices
        curr_grid_x = int(floor(self.x / self.map_resolution))
        curr_grid_y = int(floor(self.y / self.map_resolution))

        print("End grid pos: ", x_end, y_end)

        # Calculate path 
        path = astar(self.working_map, (x_end, y_end), (curr_grid_x, curr_grid_y))
        
        # Check if path is valid
        if (not path):
            print("Dear user, please select another grid position...")
            return 


        # Append last point
        path.append([x_end, y_end])
        show_path(self.working_map, path)

        for i, next_position in enumerate(path[1:]):

            print("Next position: ", next_position)
            # Move to the next path segment
            if (i == len(path) - 1):
                self.control_loop(next_position, None)

            else:
                self.control_loop(next_position, path[i+1])    
            
            self.control_values.linear.x = 0
            self.control_values.angular.z = 0
            self.cmd_vel_publisher.publish(self.control_values)
        

    def control_loop(self, next_position, position_after_that):

        # Get next position coordinates
        x_next = next_position[0] * self.map_resolution + self.map_resolution / 2.0
        y_next = next_position[1] * self.map_resolution + self.map_resolution / 2.0
            

        # Calculate theta_g - angle with respect to position after the next position
        theta_g = 0
        if (position_after_that is not None):

            x_nn = position_after_that[0] * self.map_resolution + self.map_resolution / 2.0
            y_nn = position_after_that[1] * self.map_resolution + self.map_resolution / 2.0

            theta_g = math.atan2(
                    (y_nn - y_next),
                    (x_nn - x_next)
                )

        # While point is not reached do the control loop
        while (True):

            curr_x = self.x
            curr_y = self.y

            rho = self.L2distance([curr_x, curr_y], [x_next, y_next])


            # Break if position is reached
            if (rho < self.epsilon):
                print(next_position, " reached!")
                # Stop the robot after destination is reached
                self.control_values.linear.x = 0
                self.control_values.angular.z = 0
                self.cmd_vel_publisher.publish(self.control_values)
                break

            theta = self.yaw
            alpha = - theta + math.atan2((y_next - curr_y), (x_next - curr_x))
            beta = - theta - alpha + theta_g


            new_omega = self.deadzone(self.k_alpha * alpha + self.k_beta * beta, -1e-2, 1e-2)

            # If omega is big enough, only turn robot
            if ((abs(new_omega)) > 0.5):
                self.control_values.linear.x = 0
                self.control_values.angular.z = self.saturation(
                    new_omega, self.omega_min, self.omega_max)

            else:
                self.control_values.linear.x = self.saturation(
                    self.k_ro * rho, self.v_min, self.v_max)
                self.control_values.angular.z = self.saturation(
                    new_omega, self.omega_min, self.omega_max)

            # If omega is big enough publish only omega not the speed
            #elif (new_omega != 0):
            #    

            # Publish only speed if omega is 0
            #else:
            #    self.control_values.linear.x = self.saturation(
            #        self.k_ro * rho, self.v_min, self.v_max)
            #    self.control_values.angular.z = 0

            # Publish new values
            self.cmd_vel_publisher.publish(self.control_values) 


    def saturation(self, value, min_val, max_val):
        """
            Saturation function.
        """

        if (value > max_val):
            return max_val

        elif (value < min_val):
            return min_val

        else:
            return value


    def deadzone(self, value, min_val, max_val):
        """
            Deadzone function.
        """

        if (value > max_val):
            return value

        elif (value < min_val):
            return value

        else:
            return 0

    def run(self):

        # Initialize publishers / subscribers
        self.sub= rospy.Subscriber('/robot0/odom_drift',Odometry, self.odometry_callback)
        #self.sub= rospy.Subscriber('/robot0/odom_corr',Odometry, self.odometry_callback)
        self.cmd_vel_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)

        r = rospy.Rate(self.rospy_rate)
        try:

            # Wait for the first measurement from odometry callback
            while (not self.first_measurement):
                print("Waiting for first measurement")
                continue

            while not rospy.is_shutdown():
                
                self.initialize_my_map()

                # Specify end point
                pos = show_path(self.working_map, None, None)

                try:
                    x_end = int(pos[1])
                    y_end = int(pos[0])
                except IndexError:
                    print("Dear user, please click on map!")
                    continue
            
                # Go to the endpoint
                self.move_robot_to_point(x_end, y_end)

        except rospy.ROSInterruptException:                  
            pass  


if __name__ == '__main__':         
    
    mp = motion_planning()

    try:
        mp.run()
    except rospy.ROSInterruptException:
        pass