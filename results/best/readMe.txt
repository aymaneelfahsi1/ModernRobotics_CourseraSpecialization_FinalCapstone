Kp = .75 and Ki = 0.1 which I keep for "newTask"

Even though it induces 2 overshoots, the end effector however final configuration is converging to the desired one,
unlike "overshoot" case where overshoot is more important and also occuring in end stages of the manipulation
Tsc_goal_newtask = np.array([[0, 1, 0, 0], [-1, 0, 0, -2], [0, 0, 1, .025], [0, 0, 0, 1]])
