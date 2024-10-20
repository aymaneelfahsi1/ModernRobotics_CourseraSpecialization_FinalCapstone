Kp = .8, Ki = .05


Tsc_inital_newtask = np.array([[1, 0, 0, 2], [0, 1, 0, 0], [0, 0, 1, .025], [0, 0, 0, 1]])
Tsc_goal_newtask = np.array([[0, 1, 0, 1], [-1, 0, 0, -1.75], [0, 0, 1, .025], [0, 0, 0, 1]])

Robot intial configuration [chassis, joint angles, wheel angles]    == [3.14/5, .5, 0, 0, 0, .5, -2.2, 0, 0, 0, 0, 0]

Even though it induces 2 overshoots, the end effector however final configuration is converging to the desired one,


More calibration and angles' limiting would surely prove useful 