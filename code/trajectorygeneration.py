#functions for enhancing end-effector speeds along the 6 segements, as suggested in the wiki
from essentials import *

def distance(p1, p2):

    d = mt.sqrt(((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2)+((p1[2]-p2[2])**2))

    return d

#function for regulating satial speed of updates the end effector frame configuration takes
def find_appropriate_speed(Tstart, Tend, max_speed , k=1 ):
     D = distance(Tstart[:-1,-1:].flatten(), Tend[:-1,-1:].flatten())
     Dt = D/max_speed
     N = Dt*k/0.01

     return Dt, N

#Function "Trajectory_Generation" will be writing the full trajectory into a file named "filename.csv" to be later processed during the robot manipulation





def Trajectory_Generation(Tse_0, Tsc_0, Tsc_goal, Tce_grasp, Tce_standoff, filename, max_speed = .25, k = 1, Dt = .01 ):

  #First segement, using screw motion (see 9.6), gripper state will be 0 all along this one....
  Tf1, N1 = find_appropriate_speed(Tse_0, Tsc_0@Tce_standoff, max_speed = .25)
  Tf2, N2 = find_appropriate_speed(Tsc_0@Tce_standoff, Tsc_0@Tce_grasp, max_speed = 1)
  Tf3, N3 = find_appropriate_speed(Tsc_0@Tce_grasp, Tsc_0@Tce_standoff, max_speed = 1)
  Tf4, N4 = find_appropriate_speed(Tsc_0@Tce_standoff, Tsc_goal@Tce_standoff, max_speed = .25)
  Tf5, N5 = find_appropriate_speed(Tsc_goal@Tce_standoff, Tsc_goal@Tce_grasp, max_speed = 1)
  Tf6, N6 = find_appropriate_speed(Tsc_goal@Tce_grasp, Tsc_goal@Tce_standoff, max_speed = 1)

  N = max(N1,N2, N3, N4, N5, N6)#I foud this helpful to ensure conmore satisfying convergence to desired end effector final configuration



  segment1 = ScrewTrajectory(Tse_0, Tsc_0@Tce_standoff, Tf=Tf1, N = N, method = 5)# Start and End configurations RELATIVE TO FIXED FRAME, we set method to 5: fifth polynomial time scaling by default...


  #Second segement, standoff to grasping position
  
  segment2 = ScrewTrajectory(Tsc_0@Tce_standoff, Tsc_0@Tce_grasp, Tf=Tf2, N = N, method = 5)


  #Third segment, grasping to standoff
  
  segment3 = ScrewTrajectory(Tsc_0@Tce_grasp, Tsc_0@Tce_standoff, Tf=Tf3, N = N, method = 5)


  #Fourth segment, old standoff to new cube standoff position
  segment4 = ScrewTrajectory(Tsc_0@Tce_standoff, Tsc_goal@Tce_standoff, Tf=Tf4, N = N, method = 5) #just replace Tsc_0 with Tsc_goal for correct s frame linking


  #Fifth segment,new standoff to new release position
  
  segment5 = ScrewTrajectory(Tsc_goal@Tce_standoff, Tsc_goal@Tce_grasp , Tf=Tf5, N = N, method = 5)


  #Sixth and final segment, moving gripper back to final standoff position
  
  segment6 = ScrewTrajectory(Tsc_goal@Tce_grasp, Tsc_goal@Tce_standoff , Tf=Tf6, N = N, method = 5)



  csv_file = filename

  with open(csv_file, 'w', newline='') as file:
    writer = csv.writer(file)

    for M in segment1:
      r11, r12, r13, r21, r22, r23, r31, r32, r33 =   [M[i][j] for i in range(M[:3, :3].shape[0]) for j in range(M[:3, :3].shape[1])]
      px, py, pz = [M[o][-1] for o in range(3)]
      writer.writerow([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 0])# the last 0 is for the gripper state, open for segment 1

    L = []

    for M in segment2:
      r11, r12, r13, r21, r22, r23, r31, r32, r33 =   [M[i][j] for i in range(M[:3, :3].shape[0]) for j in range(M[:3, :3].shape[1])]
      px, py, pz = [M[o][-1] for o in range(3)]
      writer.writerow([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 0])# the last 0 is for the gripper state, open for segment 1. It until the end of this segment that we'll intitiate the .625 seconds of closing the gripper while maitaining the end effector in graspin posture
      L = [ r11, r12, r13, r21, r22, r23, r31, r32, r33,  px, py, pz]

    for z in range(int(.625/Dt)+1):## Writing the same last row of the file for int(.625/Dt) times....  for closing the gripper
        writer.writerow(L+[1])

    for M in segment3:
      r11, r12, r13, r21, r22, r23, r31, r32, r33 =   [M[i][j] for i in range(M[:3, :3].shape[0]) for j in range(M[:3, :3].shape[1])]
      px, py, pz = [M[o][-1] for o in range(3)]
      writer.writerow([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 1])

    for M in segment4:
      r11, r12, r13, r21, r22, r23, r31, r32, r33 =   [M[i][j] for i in range(M[:3, :3].shape[0]) for j in range(M[:3, :3].shape[1])]
      px, py, pz = [M[o][-1] for o in range(3)]
      writer.writerow([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 1])

    for M in segment5:
      r11, r12, r13, r21, r22, r23, r31, r32, r33 =   [M[i][j] for i in range(M[:3, :3].shape[0]) for j in range(M[:3, :3].shape[1])]
      px, py, pz = [M[o][-1] for o in range(3)]
      writer.writerow([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 1])
      L = [ r11, r12, r13, r21, r22, r23, r31, r32, r33,  px, py, pz]

    for u in range(int(.625/Dt)+1):## Writing the same last row of the file for int(.625/Dt) times.... for opening the gripper
        writer.writerow(L+[0])

    for M in segment6:
      r11, r12, r13, r21, r22, r23, r31, r32, r33 =   [M[i][j] for i in range(M[:3, :3].shape[0]) for j in range(M[:3, :3].shape[1])]
      px, py, pz = [M[o][-1] for o in range(3)]
      writer.writerow([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 0])


