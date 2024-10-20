#This is the main script, which will deal with the three cases: 2 for "best" and "overshoot" cases, and one for "new_task"
#This script supposes the results 3 directories are already created... just the folders
from essentials import *
from nextstate import NextState
from trajectorygeneration import Trajectory_Generation
from feedbackcontrol import *

Tse_initial_list = [calc_Tse(initial_robot_configs_list[i]) for i in range(3)] #Calculating intial Tse's, entries of trajectory generation functions

names = ["overshoot", "best", "newTask"]

def plot_Xerrs(Xerrs, Kp, Ki, caseName:str):
    
    labels = []

    for n in range(Xerrs.shape[1]):
        plt.plot(Xerrs[:,n])
        labels.append(n)

    plt.legend(labels)
    plt.title("Kp = " + str(Kp[0,0]) + " and Ki = " + str(Ki[0, 0]) +" for:" + caseName ) #Kp ad Ki are matrices, with diagonal elements equal to constant gains
    plt.savefig("results/" + caseName + "/"'plottingFor'+  caseName + '.png')


for o in range(3): #robot manipulation for the 3 required cases

    #Writing log file...
    log_ = open("results/"+ names[o] + "/" +  names[o]+ "_logfile.txt", 'w')
    sys.stdout = log_

    print(datetime.now())


    initial_robot_config = initial_robot_configs_list[o]
    current_robot_config = initial_robot_config

    Kp = Kp_list[o]
    Ki = Ki_list[o]

    Xerrs = np.array([[0, 0, 0, 0, 0, 0]]) #For storing Xerrs lists by stacking vertically for ease of later plotting


    csv_reference_trajectory =  "results/" + names[o] + "/"+ names[o]+ '_reference_trajectory.csv'
    csv_final =  "results/" + names[o] + "/"+ names[o] + '_final_config.csv'
    csvXerrs =  "results/" + names[o] + "/"+ names[o] + '_Xerrs.csv'
    print("Generating Reference Trajectory for " +  names[o] + " case!")

    Trajectory_Generation(Tse_0=Tse_initial_list[o], Tsc_0 = Tsc_list[o], Tsc_goal = Tsc_goal_list[o], Tce_grasp = Tce_grasp, Tce_standoff = Tce_standoff, filename= "results/" + names[o] + "/"+ names[o]+ '_reference_trajectory.csv')
    
    print("Reference Trajectory for " +  names[o] + " case generated!")


    with open(csv_final, 'w', newline='') as file_final, open(csv_reference_trajectory, 'r') as reference_trajectory:
        plt.clf() 
        reader = csv.reader(reference_trajectory)
        writer = csv.writer(file_final)

        ref_traj = [[float(l) for l in row[:-1]] + [float(row[-1])] for row in reader] #conversions conversions....
        print("Generating csv animation for " + names[o] + " case!" )
        for i in range(len(ref_traj)-1):

                Tse_r = calc_Tse(current_robot_config) #Current actual configuration of end-effector frame relative to s frame...
                #RECALL THAT in Reference_Trajectory, we had put Tse matrices components in list format! so we gotta go back to the 4*4 Tse matrix format ;)
                f = ref_traj[i][:-1]
                fnext =  ref_traj[i+1][:-1]

                Tse_f_i = np.zeros((4, 4))
                Tse_fnext_i = np.zeros((4, 4))


                #reconstructing Tse_f
                Tse_f_i[:3, :3] = np.array([[f[0], f[1], f[2]], [f[3], f[4], f[5]], [f[6], f[7], f[8]]])
                Tse_f_i[:, -1] = np.array([f[9], f[10], f[11], 1])

                #reconstructing Tse_fnext
                Tse_fnext_i[:3, :3] = np.array([[fnext[0], fnext[1], fnext[2]], [fnext[3], fnext[4], fnext[5]], [fnext[6], fnext[7], fnext[8]]])
                Tse_fnext_i[:, -1] = np.array([fnext[9], fnext[10], fnext[11], 1])



                velocity_controls_i, Xerr_i = FeedBackControl(current_config = current_robot_config, Tse_r=Tse_r, Tse_f=  Tse_f_i, Tse_fnext=  Tse_fnext_i, Kp = Kp, Ki = Ki )

                Xerrs = np.vstack((Xerrs, np.array([Xerr_i])))

                new_robot_configuration = NextState(current_robot_configuration = current_robot_config, wheels_joints_controls=velocity_controls_i)
                #  robot_configs.append(new_robot_configuration) #storing kth configuration for later animation
                current_robot_config = new_robot_configuration
                

                writer.writerow(current_robot_config + [ref_traj[i][-1]]) #corresponding i-th row in the csv file with reference trajectory points, from where'd get back the gripper state for point i...
        print("csv animation generated for " + names[o] +" Case!" )
    print("Writing error data and saving it as plotting for " + names[o] +" Case!" )
    with open(csvXerrs, 'w', newline='') as csvXerrscsv:
             writer = csv.writer(csvXerrscsv)

             for m in range(Xerrs.shape[0]):   writer.writerow(list(Xerrs[m, :])) 
    #Plotting errors...
    
    plot_Xerrs(Xerrs, Kp, Ki, names[o])

    print("All done for " + names[o] +" Case!")












