//irb120_reactive_task_commander.cpp
//this version is a variation on irb120_task_commander, but adds an action client of
//the magic_object_finder to make the robot move in response to perceived objects
#include <ros/ros.h>
#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std; // avoids having to say: std::string, std::cout, etc
#include <irb120_fk_ik/irb120_kinematics.h>  //access to forward and inverse kinematics
#include <fk_ik_virtual/fk_ik_virtual.h> //defines the base class with virtual fncs
// this is useful to keep the motion planner generic
#include "robot_specific_fk_ik_mappings.h" //these two files are needed to provide robot-specific info to generic planner
#include "robot_specific_names.h"
#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<sensor_msgs/JointState.h>
//add these to use the "magic" object finder action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>

//the following will be useful when need tool transforms
//#include <tf/transform_listener.h>
//#include <xform_utils/xform_utils.h>

//XformUtils xformUtils; //handy conversion utilities--but don't need these yet

//some magic numbers--place at the top of the program
std::vector<double> g_planner_joint_weights{3, 3, 2, 1, 1, 0.5}; //specify weights to use for planner optimization
double goalX = 0.3; //goal X position of the gear
double goalY = 0.0; //goal Y position of the gear
double offsetX = 0.07; //The offset in the X direction from the center of the gear
double offsetY = 0.07; //The offset in the Y direction from the center of the gear
double errorMargin = 0.05; //The margin of error the gear can have fromt the goal coordinates
//another magic value: hard-coded name of object of interest
string g_object_name("gear_part_ariac");  //hard-coded object name; edit this for different objects
int g_found_object_code; //global to communicate between callback and main: true if named object was found
geometry_msgs::PoseStamped g_perceived_object_pose; //global to communicate between callback and main: pose  of found object

ros::Publisher *g_pose_publisher; //make this global so callback can access it--for displaying object frames in rviz

CartTrajPlanner *pCartTrajPlanner; //does  not  have to be global, unless needed by other functions

//arm pose in joint space; the only reason this is global is that it will be useful, in the future, for it
//to be updated by a subscriber to joint_states
Eigen::VectorXd g_q_vec_arm_Xd;

//this callback function receives a result from the magic object finder action server
//it sets g_found_object_code to true or false, depending on whether the  object was found
//if the object was found, then components of g_perceived_object_pose are filled in
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const magic_object_finder::magicObjectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND) {
        ROS_WARN("object-finder responded: object not found");
    }
    else if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         g_pose_publisher->publish(g_perceived_object_pose);  //this is to enable display of pose of found object in rviz
    }
    else {
        ROS_WARN("object not found!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_commander"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    Eigen::Affine3d start_flange_affine, goal_flange_affine; //specify start and goal in Cartesian coords
    std::vector<Eigen::VectorXd> optimal_path; //a path in joint space is a sequence of 6-DOF joint-angle specifications
    trajectory_msgs::JointTrajectory new_trajectory; //will package trajectory messages here

    //set up an action client to query object poses using the magic object finder
    actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> object_finder_ac("object_finder_action_service", true);
    bool finished_before_timeout=false; 
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    magic_object_finder::magicObjectFinderGoal object_finder_goal; //instantiate goal message to communicate with magic_object_finder
    
    //the following is an std::vector of affines.  It describes a path in Cartesian coords, including orientations
    //not needed yet; is constructed inside the generic planner by interpolation
    //std::vector<Eigen::Affine3d> affine_path; 
    Eigen::Matrix3d R_down; //define an orientation corresponding to toolflange pointing down
    Eigen::Vector3d x_axis, y_axis, z_axis, flange_origin, offset_coord, goal_coord;
    z_axis << 0, 0, -1; //points flange down
    x_axis << -1, 0, 0; //arbitrary
    y_axis = z_axis.cross(x_axis); //construct y-axis consistent with right-hand coordinate frame
    R_down.col(0) = x_axis;
    R_down.col(1) = y_axis;
    R_down.col(2) = z_axis;
    flange_origin << 0.2, 0, 0.01;  //SHOULD GET FIXED: hard-coded pose can result in ugly/dangerous motion
    int nsteps = 5; //will need to specify how many interpolation points in Cartesian path; this is pretty coarse
    double arrival_time = 5.0; //will  need to specify arrival time for a Cartesian path

    //for this next line, I apparently did something wrong.  I should not have to  instantiate a cartesianInterpolator,
    //since the generic planner instantiates one.  But I get a compiler error.  Hmm...  Workaround.
    CartesianInterpolator cartesianInterpolator;

    g_q_vec_arm_Xd.resize(NJNTS); //generic vector resized to actual robot number of joints
    g_q_vec_arm_Xd << 0, 0, 0, 0, 0, 0; //assumes arm starts in this pose; better would be  to subscribe to joint_states to get actual angles

    //our irb120 control  interface uses this topic to receive trajectories
    ros::Publisher traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    //somewhat odd construction: a pointer to an object of type CartTrajPlanner, with arguments provided
    //that are pointers to forward and inverse kinematic functions.  This is to keep the planner generic,
    //and defer WHICH robot FK and IK to use until run time; Uses virtual functions for this.
    pCartTrajPlanner = new CartTrajPlanner(pIKSolver, pFwdSolver, njnts);
    //the planner needs to define penalty weights to optimize a path
    pCartTrajPlanner->set_jspace_planner_weights(g_planner_joint_weights);
    //to fill out a trajectory, need to provide the joint names; these are contained in a robot-specific header file
    pCartTrajPlanner->set_joint_names(g_jnt_names);

    optimal_path.clear(); //reset this std::vector before  each use, else  will have old values persisting
    optimal_path.push_back(g_q_vec_arm_Xd); //start from current pose
    optimal_path.push_back(g_q_vec_arm_Xd); // go from current pose to current pose--not very useful; but can "warm up" control 
    //publish/subscribe interface
    arrival_time = 1; //move should require zero time, but provide something small

    //function call from library (Class) CartTrajPlanner: converts a joint-space path to a joint-space trajectory
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);

    traj_publisher.publish(new_trajectory); //publish the trajectory; 
    ros::Duration(1).sleep();

    //example to show how to use forward kinematics from the class pointers provided 
    start_flange_affine = pFwdSolver->fwd_kin_solve(g_q_vec_arm_Xd);    
    
    //xxxxxxxxxxxxxx  the following makes an inquiry for the pose of the part of interest
    //specify the part name, send it in the goal message, wait for and interpret the result
    object_finder_goal.object_name = g_object_name.c_str(); //convert string object to old C-style string data
    object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server
        
    finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    //NOTE: could do something else here (if useful) while waiting for response from action server
    if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
    //check the result code to see if object was found or not
    if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND)   {
        ROS_INFO("found object!");
    }    
    else {
        ROS_WARN("object not found!  Quitting");
        return 1;
    }
    //Done with inquiry.  If here, then part pose is in g_perceived_object_pose.  Use it to compute robot motion    

    goal_flange_affine.linear() = R_down; //set the  goal orientation for flange to point down; will not need to change this for now
    //xxxx  use the x and y coordinates of the gear part, but specify a higher z value
    flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y, 0.5; //specify coordinates for the desired flange position (origin) with respect to the robot's base frame
    goal_flange_affine.translation() = flange_origin; //make this part of the flange  affine description

    //interpolate from start pose to goal pose with this many samples along Cartesian path
    nsteps = 5; //arbitrary; tune me

    //compute an optimal joint-space path:
    optimal_path.clear();
    //planner will return "false" if unsuccessful; should add error handling
    //successful result will be a joint-space path in optimal_path
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }
    //if here, have a viable joint-space path; convert it to a trajectory:
    //choose arrival time--to  be  tuned
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)
    ROS_INFO("done with first trajectory");

	//while the flange isn't within a margin of error of the goal
	while (ros::ok() && (g_perceived_object_pose.pose.position.x < goalX - errorMargin || 
			g_perceived_object_pose.pose.position.x > goalX + errorMargin ||
			g_perceived_object_pose.pose.position.y < goalY - errorMargin ||
			g_perceived_object_pose.pose.position.y > goalY + errorMargin)) {
		//Determine if the gear is off in the X position
		if (g_perceived_object_pose.pose.position.x < goalX - errorMargin || g_perceived_object_pose.pose.position.x > goalX + errorMargin) {
			//The gear is off in the x position
			if (g_perceived_object_pose.pose.position.x < goalX) {
				//offset in the negative direction to the gear, since it needs to be pushed in the positive direction
				offset_coord << g_perceived_object_pose.pose.position.x - offsetX, g_perceived_object_pose.pose.position.y, 0.0; //position next to gear
				goal_coord << goalX - offsetX, g_perceived_object_pose.pose.position.y, 0.0; //position next to the goal x
			}
			else {
				//offset in the positive direction of the gear, since it needs to be pushed in the negative direction
				offset_coord << g_perceived_object_pose.pose.position.x + offsetX, g_perceived_object_pose.pose.position.y, 0.0; //position next to gear
				goal_coord << goalX + offsetX, g_perceived_object_pose.pose.position.y, 0.0; //position next to the goal x
			}

			//MOVE ARM NEXT TO THE GEAR IN THE X DIRECTION
			flange_origin << offset_coord;
			goal_flange_affine.translation() = flange_origin;

			g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the  plan, so can use it for start of next plan
			// better would be to get resulting joint-space values from joint_states
			//compute an optimal Cartesian motion in joint space from current joint-space pose to desired Cartesian pose
			optimal_path.clear();
			nsteps = 100; //tune me
			//compute the plan, to be returned in optimal_path
			if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
				nsteps, optimal_path)) {
			    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
			    return 1;
			}
			//convert the path to a trajectory (adds joint-space names,  arrival times, etc)
			pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
			traj_publisher.publish(new_trajectory); //publish the trajectory
			ros::Duration(arrival_time).sleep(); //wait for the motion
			ROS_INFO("done with setting down in x direction");

			//MOVE ARM TO THE GOAL X DIRECTION
			flange_origin << goal_coord;
			goal_flange_affine.translation() = flange_origin;
			g_q_vec_arm_Xd = optimal_path.back(); //start from the joint-space pose that ended the prior plan
			//convert move to an optimal joint-space path:
			optimal_path.clear();
			nsteps = 100; //lots of points for smooth motion

			if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
				nsteps, optimal_path)) {
			    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
			    return 1;
			}

			pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
			traj_publisher.publish(new_trajectory); //publish the trajectory
			ros::Duration(arrival_time).sleep(); //wait for the motion
			ROS_INFO("done with sliding in x direction");

			//MOVE ABOVE THE GEAR 
			flange_origin << goalX + offsetX, goalY, 0.5;
			goal_flange_affine.translation() = flange_origin;

			g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the  plan, so can use it for start of next plan
			// better would be to get resulting joint-space values from joint_states
			//compute an optimal Cartesian motion in joint space from current joint-space pose to desired Cartesian pose
			optimal_path.clear();
			nsteps = 100; //tune me
			//compute the plan, to be returned in optimal_path
			if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
				nsteps, optimal_path)) {
			    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
			    return 1;
			}
			//convert the path to a trajectory (adds joint-space names,  arrival times, etc)
			pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
			traj_publisher.publish(new_trajectory); //publish the trajectory
			ros::Duration(arrival_time).sleep(); //wait for the motion
			ROS_INFO("done with moving up");
		}

		//Determine if the gear is off in the Y position
		if (g_perceived_object_pose.pose.position.y < goalY - errorMargin || g_perceived_object_pose.pose.position.y > goalY + errorMargin) {
			//The gear is off in the y position
			if (g_perceived_object_pose.pose.position.y < goalY) {
				//offset in the negative direction because the gear needs to be pushed in the positive direction
				offset_coord << goalX, g_perceived_object_pose.pose.position.y - offsetY, 0.0; //position next to the gear in Y direction
				goal_coord << goalX, goalY - offsetY, 0.0; //position next to the goal Y
			}
			else {
				//offset in the positive direction because the gear needs to be pushed in the negative direction
				offset_coord << goalX, g_perceived_object_pose.pose.position.y + offsetY, 0.0; //position next to the gear in the Y direction
				goal_coord << goalX, goalY + offsetY, 0.0; //position next to the goal Y
			}

			//MOVE ARM NEXT TO THE GEAR IN Y DIRECTION
			flange_origin << offset_coord;
			goal_flange_affine.translation() = flange_origin;

			g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the  plan, so can use it for start of next plan
			// better would be to get resulting joint-space values from joint_states
			//compute an optimal Cartesian motion in joint space from current joint-space pose to desired Cartesian pose
			optimal_path.clear();
			nsteps = 100; //tune me
			//compute the plan, to be returned in optimal_path
			if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
				nsteps, optimal_path)) {
			    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
			    return 1;
			}
			//convert the path to a trajectory (adds joint-space names,  arrival times, etc)
			pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
			traj_publisher.publish(new_trajectory); //publish the trajectory
			ros::Duration(arrival_time).sleep(); //wait for the motion
			ROS_INFO("done with setting down in y direction");


			//MOVE ARM NEXT TO THE GOAL Y POSITION
			flange_origin << goal_coord;
			goal_flange_affine.translation() = flange_origin;
			g_q_vec_arm_Xd = optimal_path.back(); //start from the joint-space pose that ended the prior plan
			//convert move to an optimal joint-space path:
			optimal_path.clear();
			nsteps = 100; //lots of points for smooth motion

			if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
				nsteps, optimal_path)) {
			    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
			    return 1;
			}

			pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
			traj_publisher.publish(new_trajectory); //publish the trajectory
			ros::Duration(arrival_time).sleep(); //wait for the motion
		 
			ROS_INFO("done with sliding in y direction");

			//MOVE ABOVE THE GEAR
			flange_origin << goalX + offsetX, goalY, 0.5;
			goal_flange_affine.translation() = flange_origin;

			g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the  plan, so can use it for start of next plan
			// better would be to get resulting joint-space values from joint_states
			//compute an optimal Cartesian motion in joint space from current joint-space pose to desired Cartesian pose
			optimal_path.clear();
			nsteps = 100; //tune me
			//compute the plan, to be returned in optimal_path
			if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
				nsteps, optimal_path)) {
			    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
			    return 1;
			}
			//convert the path to a trajectory (adds joint-space names,  arrival times, etc)
			pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
			traj_publisher.publish(new_trajectory); //publish the trajectory
			ros::Duration(arrival_time).sleep(); //wait for the motion
			ROS_INFO("done with moving up");
		}

		//find the gear's position again to see if it is at the goal and break out of the loop
		object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb);
 		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
		if (!finished_before_timeout) {
	        ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
	        return 1;
		}
		ros::spinOnce();
	}
}




