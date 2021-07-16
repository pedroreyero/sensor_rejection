# Master Thesis: Sensor rejection for reliable state estimation of an autonomous last-mile delivery vehicle
## Author: Pedro Reyero Santiago
## Supervisors: Dr. Erik Wilhelm, Prof. Dr. Christopher Onder

This repository contains the code published with the thesis in runnable form with sample bags collected with the Autonomous Plus2.

### Instructions to prepare the workspace (only 1st time):
1. Clone the repository
```
git clone https://github.com/pedroreyero/state_estimation.git
```
2. Enter the catkin_ws folder
```
cd state_estimation/catkin_ws
```
3. Run the following command in the terminal
```
catkin_make
```
4. Run the following commands in the terminal
```
chmod +x src/state_estimation/src/node_data.py
chmod +x src/state_estimation/src/node_exnis.py
chmod +x src/state_estimation/src/node_tree.py
chmod +x src/state_estimation/src/node_knn.py
chmod +x src/state_estimation/src/node_ffnn.py
chmod +x src/state_estimation/src/node_rnn.py
chmod +x src/state_estimation/src/node_voting.py
```

### Instructions to run an example .bag file:
1. Inside the catkin_ws folder, run the following commands in a terminal:
```
source devel/setup.bash
roslaunch state_estimation state_estimation.launch
```
2. In the same folder, in a different terminal, run the following commands (you can change the desired .bag file):
```
source devel/setup.bash
rosbag play src/state_estimation/bag/example1.bag
```
3. (Optional) In the same folder, in a different terminal, run the following commands to visualize the output of the voting system (you can change the topic, for instance, to an individual algorithm like k-NN):
```
source devel/setup.bash
rostopic echo /state_estimation/voting_decision
```
