#!/usr/bin/env python3
import time 
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan,cos,sin,pi,tan

#from yaml
Lr=rospy.get_param("/Lr")
Lf=rospy.get_param("/Lf")
#Inputs and arguments preparation
lin_vel=float(input("Enter the car velocity=\n") )       #where lin_vel=0.5r(wl+wr) as the radius of each wheel is the same 
delta= float(input("Enter the steering angle= \n"))
sim_time=float(input("The simulation time= "))
'''----------------------------------subscriber--------------------------------------'''
#once the subscriber get a new data save it in current position
curr_pose=Pose()
def callback_pose(data):
    curr_pose.x=data.x
    curr_pose.y=data.y
    curr_pose.theta=data.theta


#subscriber node to get the current pose 
def listener():
    rospy.init_node("position_listener",anonymous=True)
    pose_listener=rospy.Subscriber("/turtle1/pose",Pose,callback_pose)
'''----------------------------------publisher------------------------------------------'''
#publisher node which publishes the linear and angle velociety
def publishing():
 
    beta=atan((Lr/(Lf+Lr))*tan(delta))
    start_time=time.time()
    vel_publisher=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    while (1):
        msg=Twist()
        msg.linear.x,msg.linear.y=linear_velociety(beta,lin_vel)
        msg.linear.z=0
        msg.angular.x=0
        msg.angular.y=0
        msg.angular.z=angular_velociety(beta,delta,lin_vel)
        vel_publisher.publish(msg)
        if(time.time()-start_time-sim_time>0.2):
           break
     


#Function to calculate the X and Y components of the linear velocity
def linear_velociety (beta,lin_vel):
    Vx=lin_vel*cos(beta+curr_pose.theta)
    Vy=lin_vel *sin(beta+curr_pose.theta)
    return Vx,Vy

    
#Fuction to calc the angular velocity
def angular_velociety (beta,delta,lin_vel):
    ang =(lin_vel*cos(beta)*tan(delta))/(Lf+Lr)
    return ang


#main function
if __name__=="__main__":
    try:
        listener()
        publishing()
    except rospy.ROSInterruptException:
        pass
