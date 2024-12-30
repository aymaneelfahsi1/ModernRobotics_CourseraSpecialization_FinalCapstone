
# ModernRobotics Coursera Specialization Mobile Manipulation Capstone

This repository contains the source code and documentation for the final capstone project of the Coursera "Modern Robotics" Specialization, "Mobile Manipulation Capstone." This project integrates concepts from previous courses of the specialization, involving trajectory planning and control for a mobile manipulator.

## Project Overview

This project features the KUKA youBot mobile manipulator, and the task is to control the manipulator to pick up a block from an initial position and move it to a target position. The task involves:

- Planning and generating trajectories for the robot arm and base.
- Simulating odometry and end-effector control.
- Feedback control for precise manipulation using CoppeliaSim physics.

The final goal is to autonomously generate a CSV output file that drives the robot to successfully pick up and place a block at designated positions.

## Project Structure

- **Original Data**: Scenes and data to be used in CoppeliaSim [download here](https://drive.google.com/drive/folders/1DAMKsbyvNFgIkUIRvVi6ls5nFo6hPmF2?usp=sharing).
- **Code**: Python scripts to generate the required trajectory, control the robot, and output CSV files for CoppeliaSim.
- **Results**: Generated CSV files, error data logs, and analysis of simulation runs.
- **README.md**: The current file explaining the project, its objectives, and instructions for usage.

## How to Run

To run this project and generate the desired outputs, follow these steps:

1. Clone the repository:
   ```bash
   git clone https://github.com/aymaneelfahsi1/ModernRobotics_CourseraSpecialization_FinalCapstone.git
   ```
2. Navigate to the project directory and install required dependencies:
   ```bash
   cd ModernRobotics_CourseraSpecialization_Mobile_Manipulation_Capstone
   pip install -r requirements.txt
   ```
3. Run the provided Python scripts to generate the trajectories and control commands.
4. Use the generated CSV files in CoppeliaSim for validation of the manipulator's performance.

## Milestones
- **Milestone 1**: Implement a kinematics simulator for youBot to generate CSV output based on odometry and arm movement.
- **Milestone 2**: Generate a reference trajectory for the youBot's end-effector, ensuring smooth movement.
- **Milestone 3**: Implement feedback control to ensure precision while picking up and placing the block.

## Features
- **Trajectory Planning**: Uses quintic polynomial planning for end-effector trajectories.
- **Feedback Control**: Implements feedforward plus PI control using Jacobian pseudoinverse techniques.
- **Simulation with CoppeliaSim**: Integrates a physics-based simulation for realistic interaction between the manipulator and its environment.

## Results
The results are validated using CoppeliaSim. The robot successfully picks up the block from the initial position and places it at the desired target, overcoming initial configuration errors through feedback control.

## Future Improvements
- Incorporate obstacle avoidance planning for the full robot.
- Add joint limit avoidance and singularity detection for enhanced trajectory planning.
- Experiment with different physics engines available in CoppeliaSim for improved block manipulation.

## Contributing
If you'd like to contribute to this project, feel free to fork the repository, make improvements, and create a pull request. Contributions to extend the functionality or enhance robustness are highly welcome.

## Contact
For any queries, reach out to us at [aymane.elfahsi@student-cs.fr].
