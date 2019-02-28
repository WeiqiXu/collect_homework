/**************************************** *************************
* Filename: collect_homework.cpp
* Authors: Weiqi XU, wex064@eng.ucsd.edu
*		   Yiding QIU, yiqiu@eng.ucsd.edu
* Date: 02/24/2019
* HW #11: Implementation and robot-focused testing
*
*Description: This is the main control file for HW 11, 
*			it demonstrates a robot that collects homework from students.
*			This program subscribes to the blob topic, point cloud topic 
*           and uses sound_play package.
			
*How to use:
 Usage:
*	roscore
*	roslaunch turtlebot_bringup minimal.launch
*	roslaunch astra_launch astra_pro.launch
*	
*	roslaunch cmvision colorgui image:=/camera/rgb/image_raw
*	<this is for color calibration, close when done>
*	roslaunch cmvision cmvision.launch image:=/camera/rgb/image_raw
*	<cnrl-c to kill process>
*	rosparam set /cmvision/color_file ~turtlebot_ws/src/cmvision/colors.txt
*	rosrun cmvision cmvision image:=/camera/rgb/image_raw
*	
*   rosrun sound_play soundplay_node.py
*	rosrun collect_homework collect_homework
*******************************************************************/

#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <sound_play/sound_play.h>
#include <fstream>
#include <string>
#include <unistd.h>

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

bool got_object = false;
bool got_goal_blobs = false;
/* use the centroid as the initialization */
double object_loc_x = 0;
double object_loc_y = 0;
int system(const char *command);

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 * Description: This is the callback function of the /blobs topic;
 *				and gets the area of the color blobs (homework indicator) if found.
 ***********************************************************/

void blobsCallBack (const cmvision::Blobs& blobsIn)
{
    double goal_area = 0; 

    if (blobsIn.blob_count > 0){
		for (int i = 0; i < blobsIn.blob_count; i++){
   			/* if bright pink color blob is detected, add up the blob area. */
		    if (blobsIn.blobs[i].red == 255 && blobsIn.blobs[i].green == 0 && blobsIn.blobs[i].blue == 0){
                goal_area += blobsIn.blobs[i].area;
		    }		
		}
        /* if the goal area is larger than a threshold,  
           it indicates that the homework is detected. */
        if (goal_area > 200){
	        ROS_INFO("goal_area: %f", goal_area);
            //ROS_INFO("the goal area is: %f", goal_area);
            got_goal_blobs = true;
            ROS_INFO("Got homework");

        }

	}
}

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 * Description: This is the callback function of the PointCloud
 * 				topic, flags when an object is found. 
 ***********************************************************/
void PointCloud_Callback (const PointCloud::ConstPtr& cloud){
  
  	double ZTHRESH = 0.9; // threshold closer than which a point is considered as an object.
	int point_count = 0; // number of points within the threshold
	/* add-up coordinates of the object points. */
	double pc_x = 0; 
	double pc_y = 0;
	object_loc_x = 0;
	object_loc_y = 0;
	got_object = false;

    /* Iterate through all the points in the window to detect object. */
  	for (int k = 0; k < 420; k++){
	    for (int i = 0; i < 640; i++){
			const pcl::PointXYZ & pt=cloud->points[640*(k)+(i)];
			if (pt.z < ZTHRESH){
				point_count += 1;
				pc_x += pt.x;
				pc_y += pt.y;
			}
	    }
  	}
	
	ROS_INFO("pc_x: %f, pc_y: %f", pc_x, pc_y);
  	object_loc_x = pc_x/point_count;
	object_loc_y = pc_y/point_count;
    ROS_INFO("point_count: %d", point_count);
	ROS_INFO("object_loc_x: %f, object_loc_y: %f", object_loc_x, object_loc_y);
	// when the number of points reach a threshold, an object is detected
    if (point_count > 50){ 
    	got_object = true;
    	ROS_INFO("Student Approching");
    	//ROS_INFO("point_count is: %d", point_count);
  	}

}

/************************************************************
 * Main function
 * Description: Subscribes to the blob topic, PointCloud topic, 
 *              and sound_play topic, publishes geometry message twist message.
 * Function:	Call up each stuednts on the list one by one, 
 *              then detect students approaching and whether he/she brings the 
 *              homework or not. 
 ***********************************************************/

int main (int argc, char** argv)
{
  // Initialize ROS
	ros::init (argc, argv, "blob");
	ros::NodeHandle nh;

    /* subscribe to /sound_play topic */
    sound_play::SoundClient sc;
	/* subscribe to /blobs topic */
	ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 1, blobsCallBack);
	/* subscribe to /PointCloud topic */
	ros::Subscriber PCSubscriber = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);
	/* publish the geometry message twist message */
	ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

	ros::Rate loop_rate(0.08);
	geometry_msgs::Twist T;
    system("amixer set Master 100%");
    system("rosrun sound_play say.py 'Hi, I will start collecting your homework'");
    sleep(2);
    
    std::ifstream file("/home/turtlebot/catkin_ws/src/collect_homework/src/namelist.txt");
    std::string str;
    std::string command;

	while (ros::ok()){

		T.linear.x = 0.0;T.linear.y = 0.0; T.linear.z = 0.0;
		T.angular.x = 0.0; T.angular.y = 0.0;T.angular.z = 0.0;
        
        //call students' name on the list
        while (std::getline(file, str))
        {

            bool found_person = false;
            std::cout << str << "\n";
            command = "rosrun sound_play say.py '" + str + "'";
	        system("amixer set Master 90%");
            system(command.c_str());
            sleep(7);
            ROS_INFO("%d",got_object);
            // call the student's name twice
            for (int j=0; j<2; ++j){
                for (int i=0; i<10; ++i){
		            ros::spinOnce();
                    /* If the student is approching, turn to that student, 
                        and ask him/she to turn in the homework */
                    if (got_object){
                        ROS_INFO("object_loc_x: %f, object_loc_y: %f", object_loc_x, object_loc_y);
                        while(object_loc_x < -0.1 || object_loc_x > 0.1){
                            if (object_loc_x < 0){
                                ROS_INFO("student at left");
                                T.angular.z = 0.3;
                            }
                            else{
                                ROS_INFO("student at right");
                                T.angular.z = -0.3;
                            }
            				velocityPublisher.publish(T);
            				ros::spinOnce();
                        }
			            T.angular.z = 0;
                        found_person = true;
                        command = "rosrun sound_play say.py 'hello, please turn in your homework'";
                        system(command.c_str());
                        sleep(3);
                        break;
                    }
                }

                if (found_person){
                    for (int i=0; i<10; ++i){
			            ros::spinOnce();
                        if (got_goal_blobs){
                            //if student's homework is detected
                            command = "rosrun sound_play say.py 'thank you'";
                            system(command.c_str());
			                sleep(3);
                            break;
                        }
                        // if no homework is detected, remind him/her to turn in homework with a louder voice
                        else if (i==5){
			                system("amixer set Master 100%");
                            command = "rosrun sound_play say.py 'where is your homework?'";
                            system(command.c_str());
                            sleep(1);
                        }
                    }
                    // if student still doesn't hand in homework, proceed to the next one
        		    if (got_goal_blobs == false){
        			    command = "rosrun sound_play say.py 'Ok, seems like you do not bring your homework'";
			            system("amixer set Master 90%");
                        system(command.c_str());
                        sleep(3);
        		    }
                    // reset the parameters
                    got_object = false;
                    got_goal_blobs = false;
		            object_loc_x = 0;
    		        object_loc_y = 0;
                    break;
                }
                else{
        		    if (j==1){
				        sleep(2);
        			    break;
		        }
                    //call student's name again if no one is approaching
                    command = "rosrun sound_play say.py '" + str + "'";
			        system("amixer set Master 100%");
                    system(command.c_str());
                    //wait for 10 seconds
                    sleep(10);
			        system("amixer set Master 90%");
                    
                }
            }
        //if no response, proceed to next student
		if (found_person == false){
			system("rosrun sound_play say.py 'ok, next.'");	
		}
		loop_rate.sleep();
		}
        // after going through the whole namelist, terminate
	    system("amixer set Master 100%");
	    system("rosrun sound_play say.py 'That is all, thank you. Have a nice day!'");
        break;
	}
}

