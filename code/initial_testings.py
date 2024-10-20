from essentials import *
from nextstate import NextState
from trajectorygeneration import Trajectory_Generation
from feedbackcontrol import *


###Testing nextstate function

initial_robot_configuration_test = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ] #chassis configuration, joint angles, wheel angles
wheels_joints_control_test = [-10, 10, 10, -10, 100, 0, 0, 0, 0] #wheel angular velocities, joints angular velocities



latest_configuration = initial_robot_configuration_test







for i in range(100):
      M = latest_configuration.copy()
      latest_configuration = NextState(M, wheels_joints_control_test)
    



print(latest_configuration)

###Testing Trajectory Generation function
Phi0 = initial_robot_configuration_test[0]
x_0 = initial_robot_configuration_test[1]
y_0 = initial_robot_configuration_test[2]

Tsb_0 = np.array([[cos(Phi0), -sin(Phi0), 0,  x_0], [sin(Phi0), cos(Phi0), 0, y_0], [0, 0, 1, .0963], [0, 0, 0, 1]])

Tb0 = np.array([[1, 0, 0, .1662], [0, 1, 0, 0], [0, 0, 1, .0026], [0, 0, 0, 1]])

M0e = np.array([[1, 0, 0, .033], [0, 1, 0, 0], [0, 0, 1, .6546], [0, 0, 0, 1]])



Tse_0 = Tsb_0 @ Tb0 @ M0e #calculating initial end effector configuration matrix relative to s frame (spatial)
#M0e is equal to calc_Tse(initial_robot_configuration_test) as all arm angles are 0


Trajectory_Generation(Tse_0 = Tse_0, Tsc_0=Tsc_0, Tsc_goal=Tsc_goal_0, Tce_grasp=Tce_grasp, Tce_standoff=Tce_standoff, filename="trajectoryGeneration_test.csv")


###Testing FeedBackControl function


initial_config = [0, 0, 0, 0, 0, .2, -1.6, 0, 0, 0, 0, 0]

Tse_current = calc_Tse(initial_config)

print("X = ", Tse_current, end = "\n")



Xd = np.array([[0, 0, 1, .5], [0, 1, 0, 0], [-1, 0, 0, .5], [0, 0, 0, 1]])
Xd_next = np.array([[0, 0, 1, .6], [0, 1, 0, 0], [-1, 0, 0, .3], [0, 0, 0, 1]])


Kp = np.zeros((6, 6)) #feedforward control (Ki = Kp = 0)
Ki = np.zeros((6, 6))



velocity_controls, Xerr= FeedBackControl(current_config= initial_config, Tse_r = Tse_current, Tse_f =Xd , Tse_fnext = Xd_next, Kp=Kp, Ki=Ki )

print(np.array(velocity_controls), np.array((Xerr)))




