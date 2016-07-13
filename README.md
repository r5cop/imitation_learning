# Imitation Learning

ROS Packages related to imitation learning component R5COP

## ROS Nodes

![ROS Nodes](imitation_learning.png)

1. Application node (Actionlib Learn client) sends request to learn to Imitiation learning node (Actionlib Learn Server)
2. Imitiation learning node listens to JointState messages of the robot controller 
3. Application node cancels the goal. Imitiation learning node calculates the optimized trajectory and sends it as a succeeded response back to the application
4. Application sends trajectory to the robot controller
