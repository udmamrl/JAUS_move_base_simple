#!/usr/bin/env python
# Jaus Navigation for UDM IGVC 2013
# Author: Yu-Ting Wu , Cheng-Lung Lee
# Data: Jan/23/2013 
# Abstract:
# This is for JAUS of IGVC 2013.
# Update 
# Data: Mar/29/2013  fix move_base caculation with player's VFH+ calculation
   
import roslib; roslib.load_manifest('JAUS_move_base_simple')
import rospy
import PyKDL
import math
import time

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#from move_base_msgs.msg import MoveBaseAction  #include <move_base_msgs/MoveBaseAction.h>
from geometry_msgs.msg import PoseStamped

def wrapTo2PI(theta):
    '''Normalize an angle in radians to [0, 2*pi]
    '''
    return theta % (2.*math.pi)

def wrapToPI(theta):
    '''Normalize an angle in radians to [-pi, pi]
    '''
    return (wrapTo2PI(theta+math.pi) - math.pi)

def limits(Hi,Lo,D_in):
    if (D_in > Hi ):
        return Hi
    elif ( Lo > D_in ):
        return Lo
    else:
        return D_in

class JAUS_move_base_simple(object):
    def __init__(self):
        rospy.init_node('JAUS_move_base_simple')
        # Parameters
        self.max_speed                      =rospy.get_param("~max_speed",0.5)
        self.max_speed_at_max_turn_rate     =rospy.get_param('~max_speed_at_max_turn_rate',0.2) # m/s
        self.max_acceleration   =rospy.get_param('~max_acceleration',1.0)
        self.min_acceleration   =rospy.get_param('~min_acceleration',1.0)
        
        self.distance_epsilon   =rospy.get_param('~distance_epsilon',0.4)
        
        # Use degree for input then convert to radius
        self.max_turn_rate        =rospy.get_param('~max_turn_rate',90       ) *math.pi/180.
        self.angle_error_treshold =rospy.get_param('~angle_error_treshold',50) *math.pi/180.
        
        print self.max_speed,self.max_speed_at_max_turn_rate,self.max_acceleration,self.distance_epsilon,self.max_turn_rate,self.angle_error_treshold
        
        # Set up publishers/subscribers
        rospy.Subscriber('/odom', Odometry, self.HandleOdom)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.HandlePose)
        rospy.Subscriber('/move_base_simple/max_acceleration', Float32 , self.HandleAcceleration)

        # Initialize odometry message
        self.theta=0
        self.x=0
        self.y=0
        self.Vx=0
        self.Goal_x=0 
        self.Goal_y=0 
        self.waypoint_enable=False
        self.V_speed=0
        self.Omega=0

    def HandlePose(self,Pose):        
        self.Goal_x = Pose.pose.position.x
        self.Goal_y = Pose.pose.position.y
        self.waypoint_enable = True
        rospy.loginfo('Got New Goal:[ x:%s , y:%s ]' %(self.Goal_x,self.Goal_y))
        
    def HandleAcceleration(self,Acc):
    		#rosmsg show std_msgs/Float32
				#float32 data
        
        # make sure the Acc.data with in range , max 10 m/s^2, self.min_acceleration (1 m/s^2)
        self.max_acceleration=limits(10,self.min_acceleration,Acc.data)

        rospy.loginfo('Got updated Acceleration:[ %s ]' %(self.max_acceleration))

    def HandleOdom(self,odom):
        o = odom.pose.pose.orientation
        cur_heading = PyKDL.Rotation.Quaternion(o.x,o.y,o.z,o.w).GetEulerZYX()
        self.theta=cur_heading[0]
        self.x=odom.pose.pose.position.x
        self.y=odom.pose.pose.position.y
        self.Vx=odom.twist.twist.linear.x
        #rospy.loginfo('Got odometry data[ x:%s , y:%s , theta:%s degree]' %(self.x,self.y,self.theta*180./math.pi))
        
        VelocityCommandPublisher = rospy.Publisher("cmd_vel", Twist)
        velocityCommand = Twist()
        if (self.waypoint_enable):
                self.move_base()
                #self.move_base()
                velocityCommand.linear.x  = self.V_speed
                velocityCommand.angular.z = self.Omega
        else:
                velocityCommand.linear.x  = 0
                velocityCommand.angular.z = 0
        #print ('(V_speed, Omega, dtheta, theta_error, distance):({0}, {1}, {2}, {3}, {4}) \n' .format(V_speed, Omega, dtheta, theta_error, distance))
        VelocityCommandPublisher.publish(velocityCommand)

    def move_base(self):
        
        distance = math.sqrt(pow(self.Goal_x-self.x,2.0) + pow(self.Goal_y-self.y,2.0))
        dtheta = math.atan2(self.Goal_y-self.y,self.Goal_x-self.x)  
        theta_error= wrapToPI(dtheta-self.theta)

        self.turn_and_drive(distance,theta_error)

        print ('(Goal_x , Goal_y)  : ({0}    , {1})' .format(self.Goal_x ,self.Goal_y)) 
        print ('(robot_x, robot_y) : ({0}    , {1})' .format(self.x ,self.y)) 
        print ('(dist, theta_error): ({0} m  , {1} degree/s)' .format(distance    ,theta_error*180/math.pi)) 
        print ('(Speed, Omega)     : ({0} m/s, {1} degree/s)' .format(self.V_speed, self.Omega*180/math.pi)) 
        print (' Acceleration      : ({0} m/s^2 )' .format(self.max_acceleration))
        
    def turn_and_drive(self,Dist2goal,Angle2goal):
        if (Dist2goal<=self.distance_epsilon): # if we are in the goal , stop the vehicle
            self.V_speed=0
            self.Omega=0
            print ('case[1]') 
        else:
            # find the speed base on the angle_error
            if(abs(Angle2goal) > (self.angle_error_treshold )): # if angle to goal is bigger then treshold, slow down vehicle
                self.V_speed=self.max_speed_at_max_turn_rate; # in big turn, limit the driving speed
                print ('case[2]')
            else:
                self.V_speed=self.get_vx(Dist2goal) # calculate the speed base on distance to goal
                print ('case[3]') 
            
            # if angle error is close to 180 degree make sure it do max turn in one side only
            if (abs(Angle2goal) > math.pi*0.75 ): # if error is around +/- 180 degree , +/- pi , must stick to one side and keep turning
                print ('case[4-0]') 
                if (abs(self.Omega)!=self.max_turn_rate): # only update omega if is not in max turn rate
                    self.Omega=(Angle2goal/(self.angle_error_treshold))*self.max_turn_rate # caliculate the turn rate
                    self.Omega=limits(self.max_turn_rate,-self.max_turn_rate,self.Omega) # limit the Omega to max_turn_rate
                    print ('case[4-1]') 
                
            else:
                self.Omega=(Angle2goal/(self.angle_error_treshold))*self.max_turn_rate # caliculate the turn rate
                self.Omega=limits(self.max_turn_rate,-self.max_turn_rate,self.Omega) # limit the Omega to max_turn_rate
                print ('case[5]') 

    def get_vx(self,Dist2goal):
        # distance to slow down , S= -Vmax^2/a , a = - max_acc. 
        dist_slow=pow(self.max_speed,2.0)/self.max_acceleration
        if (Dist2goal> dist_slow) : 
            return (self.max_speed)
        else :
            return (Dist2goal/dist_slow)*self.max_speed
if __name__ == "__main__":
 
    obj = JAUS_move_base_simple()

    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




