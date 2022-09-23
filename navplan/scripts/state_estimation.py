#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from vehicle_node import noised_state
from math import atan,cos,sin,pi,tan
rospy.init_node("state_estimation")

gps_observation = [0,0,0] # x, y, theta
steering = 0
velocity = 0
# We start at time t=1     
# Time interval in seconds
dt = 1


#Callbacks
def gps_callback(data):
    gps_observation[0]=data[0]
    gps_observation[1]=data[1]
    gps_observation[2]=data[2]
    print("GPS Observations",gps_observation)

def steer_callback(data):
    global steering
    steering = data
    print("Steering Data", steering)

def vel_callback(data):
    global velocity
    velocity = data
    print("Steering Data", velocity)

# Subscribe to the gps, steering, and velocity topics named below and update the global variables using callbacks
# /gps
gps_subscriber = rospy.Subscriber("/gps",Float64MultiArray, gps_callback)
# /car_actions/steer
steer_subscriber = rospy.Subscriber("/car_actions/steer",Float64, steer_callback)
# /car_actions/vel
vel_subscriber = rospy.Subscriber("/car_actions/vel",Float64, vel_callback)


# Publisher for the state
state_pub = rospy.Publisher('vehicle_model/state', Float64MultiArray, queue_size=10)

r = rospy.Rate(10)

# Initialize the start values and matrices here
control_matrix=[velocity,steering]

# A matrix
# 3x3 matrix -> number of states x number of states matrix
# Expresses how the state of the system [x,y,yaw] changes 
# from k-1 to k when no control command is executed.
A_t_minus_1 = np.array([[1.0,  0,   0],
                        [  0,1.0,   0],
                        [  0,  0, 1.0]])
#Noised state is imported from vehicle_node

# Measurement matrix H_t
# Used to convert the predicted state estimate at time t
# into predicted sensor measurements at time t.
# In this case, H will be the identity matrix since the 
# estimated state maps directly to state measurements from the 
# gps data [x, y, yaw]
H_t = np.array([    [1.0,  0,   0],
                    [  0,1.0,   0],
                    [  0,  0, 1.0]])

# State model noise covariance matrix Q_k
# When Q is large, the Kalman Filter tracks large changes in 
# the sensor measurements more closely than for smaller Q.
# Q is a square matrix that has the same number of rows as states.
Q_t = np.array([[1.0,   0,   0],
                [  0, 1.0,   0],
                [  0,   0, 1.0]])

# State covariance matrix P_k_minus_1
# This matrix has the same number of rows (and columns) as the 
# number of states (i.e. 3x3 matrix). P is sometimes referred
# to as Sigma in the literature. It represents an estimate of 
# the accuracy of the state estimate at time k made using the
# state transition matrix. We start off with guessed values.
P_t_minus_1 = np.array([[0.1,  0,   0],
                        [  0,0.1,   0],
                      [  0,  0, 0.1]])

# Sensor measurement noise covariance matrix R_k
# Has the same number of rows and columns as sensor measurements.
# If we are sure about the measurements, R will be near zero.
R_t = np.array([[1.0,   0,    0],
                                [  0, 1.0,    0],
                                [  0,    0, 1.0]])  

def getB(yaw, delta_time):
    """
    Calculates and returns the B matrix
    3x2 matix -> number of states x number of control inputs
    The control inputs are the forward speed and the
    rotation rate around the z axis from the x-axis in the 
    counterclockwise direction.
    [v,yaw_rate]
    Expresses how the state of the system [x,y,yaw] changes
    from k-1 to k due to the control commands (i.e. control input).
    :param yaw: The yaw angle (rotation angle around the z axis) in rad 
    :param deltak: The change in time from time step k-1 to k in sec
    """
    B = np.array([  [np.cos(yaw)*delta_time, 0],
                                    [np.sin(yaw)*delta_time, 0],
                                    [0, delta_time]])
    return B
#Function to calculate the X and Y components of the linear velocity
def linear_velociety (beta,lin_vel):
    Vx=lin_vel*cos(beta+gps_observation[2])
    Vy=lin_vel *sin(beta+gps_observation[2])
    return Vx,Vy

beta=atan((2/(1+2))*tan(steering)) 

#Fuction to calc the angular velocity
def angular_velociety (beta,delta,lin_vel):
    ang =(lin_vel*cos(beta)*tan(delta))/(1+2)
    return ang


while not rospy.is_shutdown():
    # Create the Kalman Filter here to estimate the vehicle's x, y, and theta
    ######################### Predict #############################
    # Predict the state estimate at time k based on the state 
    # estimate at time k-1 and the control input applied at time k-1.
    vx,vy=  linear_velociety (beta,velocity)
    theta=angular_velociety (beta,steering,velocity)
    state_dot = np.array([vx,vy,theta])
    state_estimate_t = A_t_minus_1 @ (
            gps_observation+state_dot) + (
            getB(gps_observation[2],dt)) @ (
            control_matrix) + (
            noised_state)
             #bicycle model
    print(f'State Estimate Before EKF={state_estimate_t}')

    # Predict the state covariance estimate based on the previous
    # covariance and some noise
    P_t = A_t_minus_1 @ P_t_minus_1 @ A_t_minus_1.T + (
            Q_t)

    ################### Update (Correct) ##########################
    # Calculate the difference between the actual sensor measurements
    # at time k minus what the measurement model predicted 
    # the sensor measurements would be for the current timestep k.
    measurement_residual_y_t = gps_observation - (H_t @ state_estimate_t) 

    # Calculate the measurement residual covariance
    S_t = H_t @ P_t @ H_t.T + R_t
 
    # Calculate the near-optimal Kalman gain
    # We use pseudoinverse since some of the matrices might be
    # non-square or singular.
    K_t = P_t @ H_t.T @ np.linalg.inv(S_t)

    # Calculate an updated state estimate for time k
    state_estimate_t = state_estimate_t + (K_t @ measurement_residual_y_t)
     
    # Update the state covariance estimate for time k
    P_t = P_t - (K_t @ H_t @ P_t)
     
    # Print the best (near-optimal) estimate of the current state of the robot
    print(f'State Estimate After EKF={state_estimate_t}')






    car_state = gps_observation

    # Create msg to publish#
    current_state = Float64MultiArray()
    layout = MultiArrayLayout()
    dimension = MultiArrayDimension()
    dimension.label = "current_state"
    dimension.size = 3
    dimension.stride = 3
    layout.data_offset = 0
    layout.dim = [dimension]
    current_state.layout = layout
    current_state.data = car_state

    state_pub.publish(current_state)
    r.sleep()