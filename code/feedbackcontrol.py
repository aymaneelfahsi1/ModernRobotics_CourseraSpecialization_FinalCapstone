from essentials import *

#we'll start by defining two functions to help us calculate current end effector Tse at each iteration during the feedback controlling

def calc_Tsb(current_robot_config:list):



    phi = current_robot_config[0]
    x = current_robot_config[1]
    y = current_robot_config[2]



    Tsb = np.array([[cos(phi),-sin(phi),0,x],
                    [sin(phi),cos(phi),0,y],
                    [0,0,1,z],
                    [0,0,0,1]])

    return Tsb


def calc_Tse(current_robot_config:list):

  T0e = MR.FKinBody(M0e, Blist, current_robot_config[3:8])
  Tsb = calc_Tsb(current_robot_config)
  Teb = np.dot(pinv(T0e), pinv(Tb0))
  Tse = np.dot(Tsb,pinv(Teb))

  return Tse

#I choose the FeedBackControl function to return a tuple with veolcity controls as a list and Xerr (error vectors) as a list too

def FeedBackControl(current_config, Tse_r, Tse_f, Tse_fnext, Kp, Ki, Dt=.01):


   Xerr_Dt_se3 = logm(pinv(Tse_r, 1e-3) @ Tse_f ) # se(3) matrix [Xerr]
   Xerr_Dt = se3ToVec(Xerr_Dt_se3)

#  print("Xerr = ",np.round_(Xerr_Dt, decimals=3))

   Vd_Dt__se3 = (1/Dt)* logm(pinv(Tse_f, 1e-3) @  Tse_fnext) #se(3) matrix [Vd_Dt]
   Vd_Dt = se3ToVec(Vd_Dt__se3)
  #  print("Vd = ", np.round_(Vd_Dt, decimals=3))



   V_Dt = Adjoint(pinv(Tse_r, 1e-3) @ Tse_f) @ Vd_Dt + Kp @ Xerr_Dt + Dt * (Ki @ Xerr_Dt)
  #  print("V = " , np.round(V_Dt, decimals=3), end = "\n")



   Fsix = np.vstack((zeroes_above, F, zeroes_below))



   Tsb_current = calc_Tsb(current_config)

   Jbase_Dt = Adjoint(pinv(Tse_f, 1e-3) @  Tsb_current ) @ Fsix #Jbase
  #  print(np.round_(Jbase_Dt, decimals=3))


   Jarm_Dt = JacobianBody(Blist=Blist, thetalist=current_config[3:8]) #Jarm
  #  print(np.round_(Jarm_Dt, decimals=3))


   Je_Dt = np.hstack((Jbase_Dt, Jarm_Dt))

  #  print("Je_Dt = ", np.round_(Je_Dt, decimals=5), end = "\n")
  #  print("Je_1 = ", np.round_(Je_Dt[:, :4], decimals=5), end = "\n")
  #  print("Je_2 = ", np.round_(Je_Dt[:, 4:], decimals=5), end = "\n")

   velocity_controls = pinv(Je_Dt, 1e-3) @ V_Dt



   return (list(np.round(velocity_controls, decimals=3)), list(np.round(Xerr_Dt, decimals=3)))

