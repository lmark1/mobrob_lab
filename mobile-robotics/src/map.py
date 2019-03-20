#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
from math import atan2, pi, asin, tanh, atan, sin, cos, e, ceil
from std_msgs.msg import Int64, String, Float64, Float64MultiArray
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
import math

map_width=1550    #real map dimensions in cm
map_height=1492
resolution=10.0   #grid tiles are 10x10 cm

class map_class():
    
    def __init__(self):
        rospy.init_node('mapping_node')
        self.occupancy_grid = OccupancyGrid()                                           #defining output occupancy grid
        self.occupancy_grid.info.resolution= resolution/100
        self.occupancy_grid.info.width=int(ceil(map_width/resolution))
        self.occupancy_grid.info.height=int(ceil(map_height/resolution))
        
        # Initialize occupancy grid data with zeros (or -1 ?)
        self.occupancy_grid.data=np.ones(
            self.occupancy_grid.info.width * self.occupancy_grid.info.height) * 50;

        self.o_grid = np.zeros(
            self.occupancy_grid.info.width * self.occupancy_grid.info.height);

        # Sample width / height
        self.sample_width = self.occupancy_grid.info.width
        self.sample_height = self.occupancy_grid.info.height
        self.resolution_meters = self.occupancy_grid.info.resolution

        # Record first measurement happening
        self.first_happened = False;

        self.yaw=0
        self.x=0
        self.y=0

        #SIMULATION PARAMETERS#
        self.Rmax= 3              # sonar max range in m
        self.rov = 100            # visibility radius in cm
        self.th3db = 0.5          # half-width of the sensor beam in radians
        self.pE = 0.2             # lower limit of the conditional probability
        self.pO = 0.8             # upper limit of the conditional probability
        self.deltark = 10.0 / 100       # parameter which designates the area in which the sensor measurement r takes the average value

        self.sonardata=[]
        self.noised_sonardata=np.zeros(16)
        self.sonar_coordinates=[ [10, 0],   [10, 5],  [10, 10],                 #relative sonar coordinates in cm
                                 [5, 10],   [0, 10],  [-5, 10],  [-10, 10],
                                 [-10, 5],  [-10, 0], [-10, -5], [-10, -10],                
                                 [-5, -10], [0, -10], [5, -10],  [10, -10],
                                 [10, -5]]
        self.sonar_thetas=[]
        for i in range(9):
            self.sonar_thetas.append(i*pi/8)                #sonar orientations in radians                              
        for i in range(7):
            self.sonar_thetas.append(-(7-i)*pi/8) 

    def sonar_callback(self, scan):     #sonar subscriber                                       
        self.sonardata=scan.data
        self.first_happened = True

    def odometry_callback(self, scan):          #odometry subscriber
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

    def calculate_conditional_probability(self, i, sensor_reading):
        ## TASK A ##
        
        # write the function which will calculate the conditional probability of all the cells affected 
        # by single sonar measurment according to the inverse sensor model
        
        # Total angle of the current sonar sonar - take robot rotation in account !
        total_angle = self.sonar_thetas[i] + self.yaw

        # Current robot position 
        robotPos_x = self.x
        robotPos_y = self.y

        # Calculate current sensor distance from center of robot
        sensor_distance = math.sqrt(
            self.sonar_coordinates[i][0] ** 2 + 
            self.sonar_coordinates[i][1] ** 2) / 100 # CONVERT TO METERS !

        # Calculate current sonar coordinates 
        sonar_x = robotPos_x + sensor_distance * math.cos(total_angle)
        sonar_y = robotPos_y + sensor_distance * math.sin(total_angle)

        sonar_probabilities = np.ones(self.sample_height * self.sample_width) * 0.5

        # Go through all the grid and calculate 
        for grid_y in range(self.sample_height):
            for grid_x in range(self.sample_width):

                # Calculate actual map position of the grid tile center
                # + self.resolution_meters / 2 - CENTER of the grid tile
                actual_x = grid_x * self.resolution_meters + self.resolution_meters/2
                actual_y = grid_y * self.resolution_meters + self.resolution_meters/2

                # Calculate distance from sonar to the grid position
                distance_x = actual_x - sonar_x
                distance_y = actual_y - sonar_y
                
                # Calculate probability function parameters
                current_rho = math.sqrt(distance_x **2 + distance_y**2)
                
                # Continue if grid is outside of max sonar range
                if (current_rho > self.rov/100):
                    continue

                distance_angle = math.atan2(distance_y, distance_x)
                current_theta = - total_angle + distance_angle 

                # Calculate sensor model 
                probability = self.sensor_model(sensor_reading, current_rho, current_theta)

                # Calculate occupancy grid index based on current actual position
                current_ogrid_index = grid_y * self.sample_width + grid_x
                sonar_probabilities[current_ogrid_index] = probability                

        return sonar_probabilities


    def do_mapping(self):       
        ##TASKS B and C##

        # add the recursive cell occupancy calculation with which a new occupancy probability 
        # will be calculated for the cells which were
        # affected by the measurement i and update the values of the occupancy grid map
    
        # Go through all sonars and calculate probabilities
        for sonar_index in range(len(self.sonar_thetas)):

            current_reading = self.sonardata[sonar_index]
            #print("Updating sensor: ", sonar_index, ", reading: ", current_reading)

            # Calculate Conditional probabilites
            conditional_probabilities = self.calculate_conditional_probability(
                sonar_index, current_reading)

            # Update occupancy grid map
            for i, c_prob in enumerate(conditional_probabilities):

                self.o_grid[i] = self.update_occupancy_grid(
                    self.o_grid[i], c_prob)


        # Update occupancy grid probabilities       
        for i in range(len(self.o_grid)):
            self.occupancy_grid.data[i] = 100 / (1 + math.exp(-self.o_grid[i]))   

                

    def update_occupancy_grid(self, old_occupancy, conditional_probability):
        """
        old_occupancy - old occupancy grid value
        conditional_probability - new calculated conditional probability for that field

        Updates occupancy grid map using the gaussian formula.
        """

        l_k = math.log(conditional_probability / (1 - conditional_probability)) + old_occupancy
        
        return l_k


    def sensor_model(self, r, rho, theta):
        """
        This method represents sensor model conditional probability function.

        r - sensor measurement
        rho - distance from sensor to field Cij
        theta - angle of Cij fiel with respect to the middle of the sensor beam

        returns - P(Sij = 0 | Rk = r)
        """

        if (rho < r - 2 * self.deltark):
            #print("1")
            return (
                0.5 + (self.pE - 0.5) * 
                self.alpha_function(theta) * 
                self.delta_function(rho)
                )

        elif ( ( (r - 2 * self.deltark) <= rho ) and ( rho < (r - self.deltark) ) ):
            #print("2")
            return ( 
                0.5 + (self.pE - 0.5) * 
                self.alpha_function(theta) * 
                self.delta_function(rho) *
                (1 - (2 + (rho - r) / self.deltark)**2) 
                )

        elif ( ( (r - self.deltark) <= rho ) and ( rho < (r + self.deltark) ) ):
            #print("3")
            return ( 
                0.5 + (self.pO - 0.5) * 
                self.alpha_function(theta) * 
                self.delta_function(rho) *
                (1 - ( (rho - r) / self.deltark )**2 ) 
                )

        elif (rho > (r + self.deltark) ):
            #print("4")
            return 0.5

        return None


    def alpha_function(self, theta):
        """
        Alhpa angle modulation function.
        """

        if (abs(theta) > self.th3db): 
            return 0

        else:
            return ( 1 - (theta / self.th3db)**2 )


    def delta_function(self, rho):
        """
        Delta function.
        """

        return ( 1 - (1 + math.tanh(2 * (rho - self.rov/100))) / 2)


    def run(self):

        self.sub= rospy.Subscriber('/robot0/sonar_data',Float64MultiArray, self.sonar_callback)       #defining the subscribers and publishers
        self.sub= rospy.Subscriber('/robot0/odom_drift',Odometry, self.odometry_callback)
        #self.sub= rospy.Subscriber('/robot0/odom',Odometry, self.odometry_callback)              #you can choose will you use real or noised odom data by commenting the subscriber code

        self.OG_publisher = rospy.Publisher('OG_map', OccupancyGrid, queue_size=10)     #occupancy grid publisher
        r=rospy.Rate(30)

        try:
            while not rospy.is_shutdown():

                if (not self.first_happened):
                    print("Waiting for first measurement")
                    continue;
                
                self.do_mapping()
                self.OG_publisher.publish(self.occupancy_grid)    #run the code and publish the occupancy grid

                r.sleep()

        except rospy.ROSInterruptException:                        
            pass                                            

if __name__ == '__main__':         
    mapping = map_class()                                                             
    try:
        mapping.run()
    except rospy.ROSInterruptException:
        pass
