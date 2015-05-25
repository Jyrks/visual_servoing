#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <phoebe_planning_msgs/PlanningRequest.h>
#include <phoebe_planning_msgs/MotionRequest.h>

class ClusterController {
public:
	ClusterController() {
		sub_ = n_.subscribe("/manipulator_target_pose", 1000, &ClusterController::callback, this);

		planning_request_client_ = n_.serviceClient<phoebe_planning_msgs::PlanningRequest>("phoebe_planning_request");
		motion_request_client_ = n_.serviceClient<phoebe_planning_msgs::MotionRequest>("phoebe_motion_request");
	}

	void callback(const geometry_msgs::PoseStamped msg) {

		std::cout << "Planned manipulator location: " << msg.pose.position.x << " " << msg.pose.position.y << " " << msg.pose.position.z << std::endl;
		phoebe_planning_msgs::PlanningRequest planningRequestSrv;
		planningRequestSrv.request.pose = msg;
		planningRequestSrv.request.id = 1; //manipulator ID (0=any, 1=left only, 2=right only)
		planningRequestSrv.request.duration = 16.0;
		std::vector<float> array;
		planningRequestSrv.request.joint_angles = array;

		phoebe_planning_msgs::MotionRequest motionRequestSrv;
		if (planning_request_client_.call(planningRequestSrv)) {
			if (planningRequestSrv.response.result) {
				ROS_INFO("Planning request succeeded");
				motionRequestSrv.request.trajectory = planningRequestSrv.response.trajectory;

				std::string move;
				std::cout << "To move the manipulator type: move" << std::endl;
				std::cin>>move;
				if (move == "move") {
					motionRequestSrv.request.tolerance = 0.1;
					if (motion_request_client_.call(motionRequestSrv)) {
						if(motionRequestSrv.response.result) {
							ROS_INFO("Motion request succeeded");
						}
						else {
							ROS_ERROR("Motion request failed");
						}
					}
				}
			}
			else {
				ROS_ERROR("Planning request failed");
			}
		}
	}

private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::ServiceClient planning_request_client_;
	ros::ServiceClient motion_request_client_;
};

int main (int argc, char **argv) {

	ros::init(argc, argv, "cluster_controller");

	ClusterController clusterController;

	ros::Rate r(10);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}