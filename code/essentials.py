import numpy as np
#will be using Python ready-to-use modern_robotics library for this project
import modern_robotics as MR
import csv
from scipy.linalg import logm #matrices logarithm
import math as mt
import matplotlib.pyplot as plt
from IPython import get_ipython
from datetime import datetime
import sys


l = .47/2
w = .3/2
r = .0475
z = .0963
sin = np.sin
cos = np.cos
pi = np.pi


F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])#Kinematic modeling of the mobile robot with 4 mecanum wheels (u = H_0@Twistb), pseudoinverse of H(0) (see 13.33)

norm = np.linalg.norm #for plotting norm of the error vectors at each iteration of the robot manipulation
inv = np.linalg.inv
pinv = np.linalg.pinv #pseudo-inverse for singular configuration matrices


ScrewTrajectory = MR.ScrewTrajectory #Screw-Axis Trajectory calculator for segments 1 and 5!
JacobianBody = MR.JacobianBody#Body Jacobian function
FKinBody = MR.FKinBody #Forward Kinematics to represent the end-effector frame for given home configuration of it
se3ToVec = MR.se3ToVec #for extracting Xerr from se(3) matrix [Xerr]
Adjoint = MR.Adjoint


Blist = np.array([[0,0,1,0,0.033,0],
                  [0,-1,0,-0.5076,0,0],
                  [0,-1,0,-0.3526,0,0],
                  [0,-1,0,-0.2176,0,0],
                  [0,0,1,0,0,0]]).T#Manipulator's home position using Screw Axes expressed in End Effector's frame (From wiki page)

Tsc_0 = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, .025], [0, 0, 0, 1]])#default initial and goal block configurations
Tsc_goal_0 = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, .025], [0, 0, 0, 1]])

Tb0 = np.array([[1, 0, 0, .1662], [0, 1, 0, 0], [0, 0, 1, .0026], [0, 0, 0, 1]])
M0e = np.array([[1, 0, 0, .033], [0, 1, 0, 0], [0, 0, 1, .6546], [0, 0, 0, 1]]) #Robt arm home-configuration, used in the FKinBody function...

#Tce_grasp and Tce_standoff, my choices

Tce_grasp = np.array([[0, 0, 1, .025], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]) 
Tce_standoff = np.array([[0, 0, 1, .025], [0, 1, 0, 0], [-1, 0, 0, .07], [0, 0, 0, 1]]) 

# Creating two rows of two zeroes for use in Jbase calculations for Je (F6 matrix...)
zeroes_above = np.zeros((2, 4))
zeroes_below = np.zeros((1, 4))

Ki_list = [ np.zeros((6, 6)), .05*np.eye(6), .05*np.eye(6)] #To use in main script for dealing with 3 cases as required: overshoot, best and newTask in order 

Kp_list = [.1*np.eye(6), .8*np.eye(6), .8*np.eye(6)]#same


initial_robot_configs_list = [[0, 0, 0, 0, 0, .2, -1.6, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, .2, -1.6, 0, 0, 0, 0, 0], [3.14/6, .5, 0, 0, 0, .5, -2.2, 0, 0, 0, 0, 0]] #first two lists default initial robot configuration as suggested in the wiki page(for "best" and "overshoot" cases) second one for the "new_task"


Tsc_newtask = np.array([[1, 0, 0, 2], [0, 1, 0, 0], [0, 0, 1, .025], [0, 0, 0, 1]])#new task initial and goal block configurations

Tsc_goal_newtask = np.array([[0, 1, 0, 1], [-1, 0, 0, -1.75], [0, 0, 1, .025], [0, 0, 0, 1]])

Tsc_list = [Tsc_0, Tsc_0, Tsc_newtask] #Same Tsc for first 2 cases, new one for newTask

Tsc_goal_list = [Tsc_goal_0, Tsc_goal_0, Tsc_goal_newtask]