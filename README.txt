
/***********************************************/
Package name: collect_homework
Authors: Weiqi XU, wex064@eng.ucsd.edu
	 Yiding Qiu, yiqiu@eng.ucsd.edu
Date: 02/24/2019
Description: This is a ROS package for turtlebot that realizes homework collecting 		     function. CMvision, PointCloud library and sound_play package
	     are used to complete this task.

/***********************************************/

The executable file is collect_homework.cpp under src folder.
To get the program running, simply do:

> roscore
> roslaunch turtlebot_bringup minimal.launch
> roslaunch astra_launch astra_pro.launch
	
> roslaunch cmvision colorgui image:=/camera/rgb/image_raw
  <this is for color calibration, close when done>
> roslaunch cmvision cmvision.launch image:=/camera/rgb/image_raw
  <cnrl-c to kill process>
> rosparam set /cmvision/color_file ~turtlebot_ws/src/cmvision/colors.txt
> rosrun cmvision cmvision image:=/camera/rgb/image_raw 

> rosrun sound_play soundplay_node.py
> rosrun collect_homework collect_homework

Note that:
1. The source code is write up to only take bright pink color patch as the "goal". 
To change the color, re-calibration of colors is needed. The RGB values and the corresponding AB values need to be stored in colors.txt file.
2. The namelist is stored in namelist.txt under src folder.
