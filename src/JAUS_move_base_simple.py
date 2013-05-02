#!/usr/bin/env python
# Jaus Navigation for UDM IGVC 2013
# Author: Yu-Ting Wu , Cheng-Lung Lee
# Data: Jan/23/2013 
# Abstract:
# This code is not correct, but robot also can reach to object.
# Update 
# Data: Mar/29/2013  fix move_base caculation with player's VFH+ calculation
   
import roslib; roslib.load_manifest('JAUS_move_base_simple')
import rospy
import PyKDL
import math
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
        self.max_speed          =rospy.get_param('~max_speed',1.5)
        self.max_turn_speed     =rospy.get_param('~max_turn_speed',0.2) # m/s
        self.max_acceleration   =rospy.get_param('~max_acceleration',1.0)
        self.max_turn_rate      =rospy.get_param('~max_turn_rate',0.5)
        self.distance_epsilon   =rospy.get_param('~distance_epsilon',0.4)
        
        # Use degree for input then convert to radius
        self.angle_epsilon    =rospy.get_param('~angle_epsilon',5) *math.pi/180.
        
        # Set up publishers/subscribers
        rospy.Subscriber('/odom', Odometry, self.HandleOdom)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.HandlePose)

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
        #self.theta = wrapToPI(self.theta)  
        dtheta = math.atan2(self.Goal_y-self.y,self.Goal_x-self.x)  
        theta_error= wrapToPI(dtheta-self.theta)


        self.turn_and_drive(distance,theta_error)
        self.Omega=limits(self.max_turn_rate,-self.max_turn_rate,self.Omega)




        print ('(Goal_x, Goal_y): ({0}, {1})' .format(self.Goal_x ,self.Goal_y)) 
        print ('(robot_x, robot_y): ({0}, {1})' .format(self.x ,self.y)) 
        print ('(dist, theta_error): ({0}, {1})' .format(distance,theta_error)) 
        print ('(Speed, Omega): ({0} m/s, {1} degree/s)' .format(self.V_speed, self.Omega*180/math.pi)) 
        #return (V_speed, Omega)
        
        
    def turn_and_drive(self,Dist2goal,Angle2goal):
        if (Dist2goal<=self.distance_epsilon):
            self.V_speed=0
            self.Omega=0
        elif (abs(Angle2goal) > (self.angle_epsilon*10 )):
            self.V_speed=self.max_turn_speed;
            #self.Omega=self.Omega+0.01*self.max_turn_rate
            if ((Angle2goal)>0) :
                self.Omega=self.max_turn_rate;
            else:
                self.Omega=-self.max_turn_rate;
        else:
            self.V_speed=self.get_vx(Dist2goal)
            self.Omega=(Angle2goal/(self.angle_epsilon*10))*self.max_turn_rate;
            #self.Omega=self.Omega+0.01*Angle2goal
           
        #return (V_speed, Omega)
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




