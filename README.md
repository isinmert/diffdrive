# diffdrive
Various ros2 nodes and launch files to drive a differential drive robot. Optimization based approaches are used to generate trajectories and cubic splines are used to store and sample points from trajectories in trajectory tracking.

## Nodes

### 1. Trajectory Trackers
All trajectory trackers are implemented using the TrajectoryTracker base class. By implementing the cmd_vel_pub_callback function in the derived classes. Currently, feedback linearization and lyapunov function based trajectory trackers are implemented.

### 2. Trajectory Generators
There is one trajectory_generator node. It uses services to generate trajectories. By making a service call, desired state for the robot can be given then the trajectory generator produces a minimum jerk trajectory by solving the associated optimization problem. 
Also, it can generate circular trajectories starting from robot state with a given length, radius and direction by making the appropriate service call.

### 3. State Publishers
Currently, there is only one robot_state_publisher node. It is assumed that the sensors are noiseless thus the robot_state_publisher node just reads sensor measurements and publishes robot state accordingly. 


## Future Work

1. Add obstacle avoidance functionality to trajectory trackers based on barrier cerificates defined in reference 2.

2. Add filtering features to robot state publisher to deal with noisy sensor measurements and partial observability. 

3. Add third trajectory tracker based on linear control principles which is explained in reference 1.

## References
1. Klancar G, Zdesar A, Blazic S, Skrjanc I. Wheeled mobile robotics: from fundamentals towards autonomous systems. Butterworth-Heinemann; 2017 Feb 2.

2. Thontepu P, Goswami BG, Singh N, PI S, Sundaram S, Katewa V, Kolathaya S. Control barrier functions in ugvs for kinematic obstacle avoidance: A collision cone approach. arXiv preprint arXiv:2209.11524. 2022 Sep 23.