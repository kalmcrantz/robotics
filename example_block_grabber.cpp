// example_block_grabber: 
// wsn, Nov, 2018; 
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

//launch with roslaunch irb140_description irb140.launch, which places a block at x=0.5, y=0
// launch action server: rosrun irb140_planner irb140_cart_move_as
// then run this node

//for manual gripper control,  rosrun cwru_sticky_fingers finger_control_dummy_node /sticky_finger/link6 false

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <std_srvs/SetBool.h>
using namespace std;

//position of the block
double block_x, block_y;

//if the block is found
bool found_block = false;

void blockPoseCallback(const geometry_msgs::PoseStamped& message) {
	block_x = message.pose.position.x;
	block_y = message.pose.position.y;
	ROS_INFO("%f, %f", block_x, block_y);
	found_block = true;
} 

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm_cart_move_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/sticky_finger/link6");
    std_srvs::SetBool srv;
    srv.request.data = true;

	//create subscriber to block's pose
	ros::Subscriber my_subscriber_object= nh.subscribe("block_pose",1, blockPoseCallback);

    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;

    nsteps = 10;
    arrival_time = 2.0;



    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << -1, 0, 0;
    t_des = b_des.cross(n_des);

    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;

    //- Translation: [0.450, -0.000, 0.367]

    O_des << 0.5, 0.3, 0.3;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation() = O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    while (ros::ok()) {

        //move out of way for camera view:
        ROS_INFO("moving out of camera view");
		tool_pose.pose.position.x = 0;
        tool_pose.pose.position.y=0.3; 
        tool_pose.pose.position.z = 0.3; //0.01;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }  

	//need to find the block position to move forward
	while (!found_block && ros::ok()) {
		ROS_INFO("Listening for block position...");
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}      
        
        //move to approach pose:
        ROS_INFO("moving to approach pose");
		tool_pose.pose.position.x = block_x;
        tool_pose.pose.position.y= block_y; 
        tool_pose.pose.position.z = 0.05; //0.01;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }           
        
        
        ROS_INFO("moving to grasp pose");
        //lower tool to approach part to grasp
        //tool_pose.pose.position.y=0; 
        tool_pose.pose.position.z = 0.0343; //block is 0.035 high      

        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

		ros::Duration(2.0).sleep();

        ROS_INFO("enabling vacuum gripper");
        //enable the vacuum gripper:
        srv.request.data = true;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }

		//ros::Duration(1).sleep();

        ROS_INFO("requesting plan to depart with grasped object:");
        tool_pose.pose.position.z = 0.3;         

        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }


		//move to the goal pose:
        ROS_INFO("moving to goal pose");
		tool_pose.pose.position.x = 0.5;
        tool_pose.pose.position.y= 0.2; 
        tool_pose.pose.position.z = 0.05; //0.01;
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }  


        //disable the vacuum gripper:

        srv.request.data = false;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }

		ros::Duration(3.0).sleep();
		found_block = false;
    }

    return 0;
}

