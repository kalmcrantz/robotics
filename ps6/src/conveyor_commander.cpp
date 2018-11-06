#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>

using namespace std;

bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam1_data;

void cam2CB(const osrf_gear::LogicalCameraImage& message_holder) {
    if (g_take_new_snapshot) {
        ROS_INFO_STREAM("image from cam1: " << message_holder << endl);
        g_cam1_data = message_holder;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2", 1, cam2CB);

    startup_srv.response.success = false;
    while (!startup_srv.response.success) {
        ROS_WARN("not successful starting up yet...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from startup service");

    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from conveyor service");

	//CODE TO FIND BOX AND STOP CONVEYOR
	bool found_box = false;
	while (!found_box) {
		//need to take snapshot to see if there's a box
		g_take_new_snapshot = true;
		while (g_cam1_data.models.size() < 1) {
			//currently no box, keep running through loop
		    ros::spinOnce();
		    ros::Duration(0.5).sleep();
		}
		//found a box, but is it a shipping box?
		if (g_cam1_data.models[0].type == "shipping_box") {
			ROS_INFO("Found a shipping box");
			found_box = true;		
		}
	}

	bool box_under = false;
	while (!box_under) {
		//the box is not under the camera so keep taking snapshots
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		if (g_cam1_data.models[0].pose.position.z < 0.1 && g_cam1_data.models[0].pose.position.z > -0.1) {
			//the box is under the camera!! Well, more within (-0.1, 0.1)
			ROS_INFO("The box is under the camera");
			box_under = true;
			conveyor_srv.request.power = 0.0; //tell conveyor to stop
			conveyor_srv.response.success = false;
			while (!conveyor_srv.response.success) {
				ROS_WARN("not successful stopping conveyor yet...");
				conveyor_client.call(conveyor_srv);
				ros::Duration(0.5).sleep();
			}
			ros::Duration(5.0).sleep(); //stay under the camera for 5 seconds
			conveyor_srv.request.power = 100.0; //restart conveyor
			conveyor_srv.response.success = false;
			while (!conveyor_srv.response.success) {
				ROS_WARN("not successful starting conveyor yet...");
				conveyor_client.call(conveyor_srv);
				ros::Duration(0.5).sleep();
			}
		}
	}
	//no longer need to take snapshots because you found a box and got it under!
	g_take_new_snapshot = false;

    drone_srv.request.shipment_type = "order_0_shipment_0";
    drone_srv.response.success = false;
    while (!drone_srv.response.success) {
        ROS_WARN("not successful starting drone yet...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from drone service");
}

