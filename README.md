# kmeans
A ROS package that implements kmeans algorithm, in order to split an occupancy grid into sub-areas with the "same" size.
-------------------------------------------------
# How to install

> git clone https://github.com/gelardrc/kmeans.git

# How to use

First you need to run your map server 

> rosrun map_server map_server <your_map.yaml> 

Then use kmeans package

> rosrun kmeans kmeans.py <number_of_agents>

# Run a example

> roslaunch kmeans example.launch

If everythings goes right, you should see rviz like this : 

![rviz](https://github.com/gelardrc/kmeans/blob/main/img/example.png)

# To do list 

- Get from robot_pose the actual pose of agents
- Correct arg in example.launch 

