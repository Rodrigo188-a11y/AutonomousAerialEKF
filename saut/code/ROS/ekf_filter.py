#!/usr/bin/env python

import time
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt
import os





#initializing variables

#change number of landmarks to correspond to the number of landmarks in your map
numLandmarks = 5
#numLandmarks = 8
teoric_distances = np.zeros(2*numLandmarks)
landmark_radius = 0.5
error_distances = np.full(numLandmarks, -1, dtype=float)
error_max_distances = np.zeros(numLandmarks)
error_max_angles = np.zeros(numLandmarks)
error_angles = np.zeros(2*numLandmarks)
distance_to_landmark = np.full(numLandmarks*2, -1, dtype=float)


#introduce manual noise
sigma_x = 0.005
sigma_y = 0.005
sigma_teta = np.deg2rad(0.00001)
sigma_v = 0.00001
sigma_w = np.deg2rad(0.00001)


ksi_x = sigma_x * np.random.normal()
ksi_y = sigma_y * np.random.normal()
ksi_teta = sigma_teta * np.random.normal()
ksi_v = sigma_v * np.random.normal()
ksi_w = sigma_w * np.random.normal()

dt = 0.15

#Pv = np.array([[6, 3, 1.3]])

Qk = np.matrix([[sigma_x**2, 0, 0], 
                [0, sigma_y**2, 0], 
                [0, 0, sigma_teta**2]])

M = np.matrix([[sigma_v**2, 0], 
               [0, sigma_w**2]])

#Pe = np.array([[6, 3, 1.3]])

Pk = Qk


sigma_z_x = 0.1
sigma_z_y = 0.1






#class to localize robot in the map
class Localization():
    

    def __init__(self):
                
        #subscriber to needed topics (laser measurements, odometry readings and velocities)
        self.laserSub = rospy.Subscriber("/robot0/laser_0", LaserScan, self.laserCallback)
        self.odomSub = rospy.Subscriber("/robot0/odom", Odometry,self.odomCallback)
        self.velSub = rospy.Subscriber("/robot0/cmd_vel", Twist, self.velCallback)


        #initialize needed variables inside the class
        self.loop_rate = rospy.Rate(10)

        

        self.odom_x = 0
        self.odom_y = 0
        self.odom_rot_z = 0
        self.init_laser = None
        self.ranges = 0

        self.angle_min_sensor = 0
        self.angle_max_sensor = 0
        self.deltaAngle = 0

        self.v = 0
        self.w = 0

        self.R = 0

        self.delta_x = 0
        self.delta_y = 0
        self.delta_rot_z = 0


        #adept landmark positon to be conform to the position of the landmarks in the map
        #self.landmarks_position = np.array([1.5, 1.5, 8.5, 1.5, 1.5, 8.5, 8.5, 8.5, 5, 6.5, 5, 3.5, 3.5, 5, 6.5, 5])
        self.landmarks_position = np.array([1.5, 1.5, 8.5, 1.5, 1.5, 8.5, 8.5, 8.5, 5, 5])

        self.Pe = np.array([[3, 3, 1.7]])
        self.Re = self.Pe

        self.Pv = np.array([[3, 3, 1.7]])
        self.Rv = self.Pv
        

    #callback function for the laser sensor subscriber
    #get information about sensor and sensor readings
    def laserCallback(self,data):
            self.init_laser = data
            self.angle_min_sensor = self.init_laser.angle_min
            self.angle_max_sensor = self.init_laser.angle_max
            self.deltaAngle = self.init_laser.angle_increment

            # laser_timestamps = np.empty([1,2])
            # laser_timestamps = np.append(laser_timestamps,[data.header.stamp.secs, data.header.stamp.nsecs],axis=0)
            
            
            self.ranges = np.array(data.ranges)
            
            
            

    #callback function for the odometry readings
    #get x and y values and convert z and w form quaternion to rad
    def odomCallback(self, msg):
        # odom_timestamp = np.empty([1,2])
        # odom_timestamp = np.append(odom_timestamp,[msg.header.stamp.secs, msg.header.stamp.nsecs], axis=0)
        
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        odom_quat_z = msg.pose.pose.orientation.z
        odom_quat_w = msg.pose.pose.orientation.w

        t3 = +2.0 * (odom_quat_w * odom_quat_z)
        t4 = +1.0 - 2.0 * (odom_quat_z * odom_quat_z)
        self.odom_rot_z = math.atan2(t3, t4)
        if self.odom_rot_z < 0:
            self.odom_rot_z = 2*math.pi+self.odom_rot_z

        #calculate a delta positon to the last measurement
        # self.delta_x = msg.pose.pose.position.x - self.Pv[0,0]
        # self.delta_y = msg.pose.pose.position.y - self.Pv[0,1]
        # self.delta_rot_z = self.odom_rot_z - self.Pv[0,2]

        

    #get velocity
    def velCallback(self,data):
        self.v = data.linear.x
        
        self.w = data.angular.z
        
       

        

        

    def calculate_teoretic_distances_to_landmarks(self):
        # Calculates the distance and the angle between the robot and each of the landmarks
        z=0
        for x in range(0, numLandmarks * 2, 2):
            teoric_distances[x] = math.sqrt(math.pow(self.landmarks_position[x] - self.Pe[0,0], 2) +
                                            math.pow(self.landmarks_position[x + 1] - self.Pe[0,1], 2))
            teoric_distances[x + 1] = math.atan2(self.landmarks_position[x + 1] - self.Pe[0,1], self.landmarks_position[x] -
                                                self.Pe[0,0])
            teoric_distances[x] = teoric_distances[x]-landmark_radius

            

            if teoric_distances[x+1] < 0:
                teoric_distances[x + 1] += 2*math.pi
            error_max_distances[z] = abs(math.sqrt(math.pow(landmark_radius, 2) + math.pow(teoric_distances[x] +
                                      landmark_radius, 2)) - teoric_distances[x])+0.3
            error_max_angles[z] = math.atan(landmark_radius/(landmark_radius+teoric_distances[x]))
            z += 1

            
        return teoric_distances


    def check_landmarks(self):  # compares error between teoretic distances and the measures
        #time.sleep(0.1)
        # numRays = (abs(self.angle_max_sensor) + abs(self.angle_min_sensor))//self.deltaAngle
        # numRays = int(numRays)+1
        
        numRays = 151
        
        angle_min_robot = self.odom_rot_z + self.angle_min_sensor
        point_list = self.ranges
        
        

        # calculates the error between each ray distance and angle with the landmarks
        for x in range(numRays):  # goes through all the laser measurments
            z = 0
            if angle_min_robot < 0:
                angle_min_robot += 2*math.pi
            if angle_min_robot >= 2*math.pi:
                angle_min_robot -= 2 * math.pi
            

            if point_list[x] != float('inf'):  # the laser detect a wall
                for y in range(0, numLandmarks * 2, 2):  # compares the ray measurment with each landmark
                    errord = abs(teoric_distances[y] - point_list[x])
                   
                    if error_distances[z] == -1:
                        

                        if angle_min_robot - error_max_angles[z] <= teoric_distances[y+1] <= angle_min_robot + error_max_angles[z] and errord <= error_max_distances[z]:
                            error_distances[z] = errord
                            distance_to_landmark[y] = point_list[x] + 0.5  # stores the distance to the landmark taking account the
                            distance_to_landmark[y+1] = angle_min_robot
                            
                    else:
                        if errord < error_distances[z]:
                            if angle_min_robot - error_max_angles[z] <= teoric_distances[y + 1] <= angle_min_robot + error_max_angles[z]:
                                error_distances[z] = errord
                                distance_to_landmark[y] = point_list[x] + 0.5  # stores the distance to the landmark taking account the
                                distance_to_landmark[y+1] = angle_min_robot
                                
                    z += 1
                angle_min_robot += self.deltaAngle
            else:
                angle_min_robot += self.deltaAngle
        
        #returns a list of distances and angles to the landmarks
        #print(distance_to_landmark)
        return distance_to_landmark

        


   

    #calculates the H Matrix and scips landmarks that don't get sensed
    def laser_jacobian(self):
        i = 0
        dist = 0
        
        values = np.array([[0,0,0],[0,0,0]])
        H = np.empty((2,3))
        begin = 0
        for i in range(0,2*numLandmarks,2):
            if distance_to_landmark[i] == -1:
                continue
            else:
                dist = distance_to_landmark[i]
                values = np.matrix([[(-1)*((self.landmarks_position[i]-self.Pv[0,0])/dist), (-1)*((self.landmarks_position[i+1]-self.Pv[0,1])/dist), 0], 
                        [((self.landmarks_position[i+1]-self.Pv[0,1])/dist**2),(-1)*((self.landmarks_position[i]-self.Pv[0,0])/dist**2) , -1]])
                if begin == 0:
                    H = values
                    begin +=1
                else:
                    H = np.append(H, values,axis=0)
        return H 


    #updates the kalmanfilter
    def filter_update(self, Pk, dt):

        ksi_x = sigma_x * np.random.normal()
        ksi_y = sigma_y * np.random.normal()
        ksi_teta = sigma_teta * np.random.normal()
        ksi_v = sigma_v * np.random.normal()
        ksi_w = sigma_w * np.random.normal()

        #real position of the robot
        self.Pv = np.array([[self.odom_x,
                            self.odom_y, 
                            self.odom_rot_z]])

        
        #store real position for plotting
        self.Rv = np.append(self.Rv, self.Pv, axis=0)
        
        #calculate theoretic and measured distance to the landmarks 
        #adjusting the size of the matrizes to the number of sensed landmarks
        S = []
        Y = []
        num_seen_landmarks = 0
        for i in range(0,numLandmarks*2,2):
            if distance_to_landmark[i] == -1:
                
                continue
            else:
                S = np.append(S,distance_to_landmark[i])
                S = np.append(S,distance_to_landmark[i+1])
                Y = np.append(Y, teoric_distances[i])
                Y = np.append(Y, teoric_distances[i+1])
                num_seen_landmarks += 1
        

        self.R = np.eye(num_seen_landmarks*2)*(sigma_z_x**2)
        

        
        
        #estimated position of the robot 
        #introducing noise
        self.Pe = np.array([[self.Pe[0,0] + ksi_x + ((self.v + ksi_v)*math.cos(self.Pe[0,2]+ksi_teta)*dt),
                            self.Pe[0,1] + ksi_y + ((self.v + ksi_v)*math.sin(self.Pe[0,2]+ksi_teta)*dt), 
                            self.Pe[0,2] + ksi_teta + (self.w + ksi_w)* dt]])

        # self.Pe = np.array([[self.Pe[0,0] + ksi_x + self.delta_x * ksi_v,
        #                     self.Pe[0,1] + ksi_y + self.delta_y * ksi_v, 
        #                     self.Pe[0,2] + ksi_teta + self.delta_rot_z* ksi_w]])
                 


        #storing estimated positions for plotting
        self.Re = np.append(self.Re, self.Pe, axis=0)

        
        #Jacobian of the model
        F = F = np.matrix([[1, 0, self.v*(-1)*math.sin(self.Pe[0,2])],
                   [1, 0, self.v*math.cos(self.Pe[0,2])], 
                   [0, 0, 1]])


        #Jacobian of the control
        G = np.matrix([[math.cos(self.Pe[0,2])*dt, 0],
                   [math.sin(self.Pe[0,2])*dt, 0], 
                   [0, dt]])

        #Jacobian of the sensor
        H = self.laser_jacobian()
        

        #Calculate Covariance 
        Pk = np.dot(np.dot(F,Pk),F.T)+np.dot(np.dot(G,M),G.T)+Qk
        

        Z = S
        #print(S)
        
        #getting the Kalman
        K = np.dot(np.dot(Pk,H.T),np.linalg.inv(np.dot(np.dot(H,Pk),H.T)+self.R))
        
        #print(np.dot((y-z).T,K.T))
        #correcting the estimated position
        self.Pe = self.Pe + np.dot((Y-Z).T,K.T)
        
        #re Calculate Covariance
        Pk = np.dot((np.eye(3))-np.dot(K,H),Pk)

        
        
        

        


    #plot the corected postion against the real position
    def plot1(self):

        plt.cla()
        plt.clf()
        plt.subplots_adjust(left=0.1,
                        bottom=0.1, 
                        right=0.9, 
                        top=0.9, 
                        wspace=0.4, 
                        hspace=0.7)
        
        for i in range(0,numLandmarks*2,2):
            plt.plot(self.landmarks_position[i], self.landmarks_position[i+1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")

        plt.plot(self.Rv[:,0], self.Rv[:,1],label="Real Position",color="red")
        plt.plot(self.Rv[-1,0], self.Rv[-1,1], marker="*", markersize=5, markeredgecolor="red", markerfacecolor="red")
        #plt.plot(Y[:,0], Y[:,1],label="Sensor Values")
        plt.plot(self.Re[:,0], self.Re[:,1],label="Mean Correct of Real Position",color="blue")
        plt.plot(self.Re[-1,0], self.Re[-1,1], marker="*", markersize=5, markeredgecolor="blue", markerfacecolor="blue")
        #plt.plot(Z[:,0], Z[:,1],label="Mean Sensor")
        # naming the x axis
        plt.xlabel('X(m)')
        # naming the y axis
        plt.ylabel('Y(m)')
        plt.legend()
        # giving a title to my graph
        plt.title('Compare Graph')

        
        
        plt.pause(0.000001)
        










#robot class to control the robot movement
class robot():

    def __init__(self):
        self.velpub = rospy.Publisher("/robot0/cmd_vel", Twist, queue_size=10)
        self.vel_msg = Twist()

        
    #function to publish to control input
    def move(self,linear_speed, angular_speed): #time
        
        

        
        
        self.vel_msg.linear.x = linear_speed
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = angular_speed

        self.velpub.publish(self.vel_msg)

        #diffrent version to move the robot
        #easier to create more complex paths
        #add time to function input


        # t0 = rospy.Time.now().to_sec()
        # stop_timer = rospy.Time.now().to_sec()
        # current_distance = 0
        # while (stop_timer-t0 < time):
        #     #Publish the velocity
        #     velpub.publish(vel_msg)
        #     #Takes actual time to velocity calculus
        #     stop_timer = rospy.Time.now().to_sec()                
        #     t1=rospy.Time.now().to_sec()
        #      #Calculates distancePoseStamped
        #     current_distance= linear_speed*(t1-t0)
        # #After the loop, stops the robot
        # vel_msg.linear.x = 0            
        # vel_msg.angular.z = 0
        # #Force the robot to stop
        # velpub.publish(vel_msg)






#initializing the filter
#running the update in a loop
#and moving the robot


if __name__ == '__main__':

    rospy.init_node("localization")
    loc = Localization()
    robot0 = robot()
   
    loc.loop_rate.sleep()
    

    while not rospy.is_shutdown():
        time1 = time.time()
        loc.calculate_teoretic_distances_to_landmarks()
        loc.check_landmarks()
        loc.filter_update(Pk, dt)
        robot0.move(0.5,-0.5)
        loc.plot1()
        time2 = time.time()
        dt = time2-time1
        #print(time2-time1)

        loc.loop_rate.sleep()
        
        
    
    
    
