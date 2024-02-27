# servebot_project

<b>About</b> : Project of making serving robot in gazebo

<b>Schedule</b> : 2023.11 ~ 2024.01

<b>Project Explanation</b>
- 6 Waypoints are made
- Have 2 modes (Automatically follow waypoints & Control the Goal Waypoint by Keyboard)

<b>Specific</b>
- Robot Model : Turtlebot3 burger
- Gazebo
- Rviz
- ROS1 Noetic

<b>World View</b>

![world1](https://github.com/KimGyuheon/servebot_project/assets/97663910/0cd46741-9f0a-48b8-b1cd-a9fbe2e75228)

World Top view with waypoints explanation  
Red circle : Waypoints

![waypoint_exp](https://github.com/KimGyuheon/servebot_project/assets/97663910/dcd0c303-e6e0-4d3c-80f4-5872e0ba3d14)

<hr>

<b>Run</b>  
navigation_pr.launch
- to use functions in go_waypoint, this is need to be launched 
- for navigation

go_waypoint.launch
 -  select mode on current terminal
 -  type s to follow whole waypoints
 -  type Waypoint numbers(1~6) which user want -> will go to typed waypoint
