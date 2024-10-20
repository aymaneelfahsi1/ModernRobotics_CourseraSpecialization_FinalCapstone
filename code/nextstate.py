#NextState function

#I defined a function "sign()" to be used for setting joint angle limits to 3*pi/2 or -3*pi/2 ;)



from essentials import *




def sign(value:float):
    if value > 0:
        return 1
    elif value < 0:
        return -1
    else:
        return 0



def NextState(current_robot_configuration, wheels_joints_controls, Dt = .01, speed_limit = 12.5) -> list:
      global F
      new_robot_configuration = [0]*len(current_robot_configuration) #we initialize output vector with zeros
      #we start by determining next joint and wheels configurations, simple first-order Euler step
      Delta_thetas = [] #where we'd save change in wheel's driving angles (to be used for odometry)

      speeds_temp = wheels_joints_controls[:4] #be only limiting wheel velocities...
      speeds_temp = np.array(speeds_temp)

      #Masks for handling joint velocity limits...
      positive_mask = speeds_temp > speed_limit
      negative_mask = speeds_temp < - speed_limit

      speeds_temp[positive_mask] = speed_limit
      speeds_temp[negative_mask] = -speed_limit

      wheels_joints_controls[:4] = speeds_temp









      for i in range(4):

        Delta_thetas.append(Dt*wheels_joints_controls[i])


      #new wheel angles
      new_robot_configuration[8:] = [(current_robot_configuration[8:][h] + Delta_thetas[h]) for h in range(4)]


      #new joint angles  if we were to take into account maximal joint angles allowed! (maximum angle allowed per joint: 3*pi/2 or -3*pi/2)
      new_robot_configuration[3:7] = [current_robot_configuration[3+o] + Dt*wheels_joints_controls[3:][o+1] if abs(current_robot_configuration[3+o] + Dt*wheels_joints_controls[3:][o+1])  <= 3*pi/2 else sign(current_robot_configuration[3+o] + Dt*wheels_joints_controls[3:][o+1])* 3*pi/2 for o in range(4)]#new joints angles with limit 3pi/4 angles, without counting joints 1 and 5 having no limit (reasonably...)
   


      #we now go for the odometry (chapter 13.4)
      DeltaTheta = np.array(Delta_thetas)
      Vb = F @ DeltaTheta
      if Vb[0] <  .000000000000001: DeltaQb = np.array([0,  Vb[1],  Vb[2] ]) #CAreful! we need to account for possible non-rotationnary movement around z-axis, as indicated in 13.35
      else: DeltaQb = np.array([Vb[0], Vb[1]*sin(Vb[0]) +  Vb[2]*(cos(Vb[0])-1)/Vb[0], Vb[2]*sin(Vb[0]) -  Vb[1]*(cos(Vb[0])-1)/Vb[0] ])

      Phi_k = current_robot_configuration[0]#see 13.3*
      Qold = current_robot_configuration[:3] #Qk, current configuration of chassis

      DeltaQ = np.array([[1, 0, 0], [0, cos(Phi_k), -sin(Phi_k)], [0, sin(Phi_k), cos(Phi_k)]]) @ DeltaQb


      Q_new = [Qold[k] + list(DeltaQ)[k] for k in range(3)]

      new_robot_configuration[:3] = Q_new

      return list(new_robot_configuration)