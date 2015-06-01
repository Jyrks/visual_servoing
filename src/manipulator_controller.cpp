#include "ros/ros.h"
#include <iostream>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <phoebe_planning_msgs/PlanningRequest.h>
#include <phoebe_planning_msgs/MotionRequest.h>
#include <phoebe_planning_msgs/GripperRequest.h>
#include <cluster_extraction/manipulatorAdjustment.h>
#include <cluster_extraction/objectDistance.h>

using namespace std;

class ManipulatorController {

public:
  ManipulatorController() {

    adjust_manipulator_sub_ = n_.subscribe("/adjust_manipulator_x", 1, &ManipulatorController::floatCallback, this);
    object_center_sub_ = n_.subscribe("/object_center_point", 1, &ManipulatorController::callback, this);

    pub_ = n_.advertise<geometry_msgs::PointStamped>("/manipulator_target_point", 1);
    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/manipulator_target_pose", 1);
    object_distance_pub_ = n_.advertise<cluster_extraction::objectDistance>("/manipulator_to_object_distance", 1);

    planning_request_client_ = n_.serviceClient<phoebe_planning_msgs::PlanningRequest>("phoebe_planning_request");
    motion_request_client_ = n_.serviceClient<phoebe_planning_msgs::MotionRequest>("phoebe_motion_request");
    gripper_request_client_ = n_.serviceClient<phoebe_planning_msgs::GripperRequest>("phoebe_gripper_request");

    initial_message_.x = 0;
    initial_message_.y = 0;
    initial_message_.z = 0;
    base_link_ = "base_link";
    manipulator_adjustment_x_ = 0;
    close_step_adder_ = 0;
    old_sync_counter_ = 0;
    new_sync_counter_ = 0;
    grab_object_ = false;
    manipulator_is_close_ = false;
    grabbed_object_ = false;
    add_final_step_delta_z_ = true;
    dont_ask_for_user_input_ = false;
    final_step_delta_z_ = -0.01;
    grab_next_step_ = false;

    n_.param("sleep_between_steps_", sleep_between_steps_, 1000000);
    n_.param("sleep_between_gripping_", sleep_between_gripping_, 3000000);
    n_.param("manipulator_first_step_x_", manipulator_first_step_x_, 0.0);
    n_.param("manipulator_first_step_y_", manipulator_first_step_y_, -0.15);
    n_.param("manipulator_first_step_z_", manipulator_first_step_z_, 0.15);
    n_.param("manipulator_camera_angle_", manipulator_camera_angle_, 0.61);
    n_.param("raise_object_when_grabbed_z_", raise_object_when_grabbed_z_, 0.05);
    n_.param("manipulator_distance_percentage_", manipulator_distance_percentage_, 0.7);
    n_.param("move_manipulator_by_on_final_steps_", move_manipulator_by_on_final_steps_, 0.02);
    n_.param("step_duration_", step_duration_, 4.0);
    n_.param("close_step_duration_", close_step_duration_, 4.0);
    n_.param("reset_grip_angle_", reset_grip_angle_, 0.40);
    n_.param("grip_angle_", grip_angle_, -0.36);
    n_.param("min_y_to_save_manipulator_pose_", min_y_to_save_manipulator_pose_, -0.03);
    n_.param("motion_request_tolerance_", motion_request_tolerance_, 0.1);
    n_.param("gripper_request_distace_", gripper_request_distace_, 0.0);
    n_.param("gripper_request_duration_", gripper_request_duration_, 1.0);
    n_.param("gripper_request_id_", gripper_request_id_, 2.0);
    n_.param("min_y_before_moving_manipulator_close_", min_y_before_moving_manipulator_close_, -0.06);
    n_.param("planning_request_id_", planning_request_id_, 1.0);

    resetGripper();
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber object_center_sub_;
  ros::Subscriber adjust_manipulator_sub_;
  ros::Publisher pub_;
  ros::Publisher pose_pub_;
  ros::Publisher object_distance_pub_;
  ros::ServiceClient planning_request_client_;
  ros::ServiceClient motion_request_client_;
  ros::ServiceClient gripper_request_client_;
  tf::TransformListener listener_;
  geometry_msgs::Point initial_message_;
  geometry_msgs::Quaternion last_pose_;
  geometry_msgs::Point initial_point_for_statistics_;
  vector<double> coordinate_history_;
  std::string base_link_;
  bool grab_object_;
  bool manipulator_is_close_;
  bool grabbed_object_;
  bool add_final_step_delta_z_;
  bool grab_next_step_;
  bool dont_ask_for_user_input_;
  int old_sync_counter_;
  int new_sync_counter_;
  int sleep_between_steps_;
  int sleep_between_gripping_;
  double manipulator_first_step_x_;
  double manipulator_first_step_y_;
  double manipulator_first_step_z_;
  double move_manipulator_by_on_final_steps_;
  double manipulator_camera_angle_;
  double manipulator_distance_percentage_;
  double raise_object_when_grabbed_z_;
  double manipulator_adjustment_x_;
  double close_step_adder_;
  double final_step_delta_z_;
  double grip_angle_;
  double reset_grip_angle_;
  double step_duration_;
  double close_step_duration_;
  double min_y_to_save_manipulator_pose_;
  double min_y_before_moving_manipulator_close_;
  double motion_request_tolerance_;
  double gripper_request_distace_;
  double gripper_request_duration_;
  double gripper_request_id_;
  double planning_request_id_;

  void callback(const geometry_msgs::Point &msg) {

    if (initial_message_.x == 0 && initial_message_.y == 0 && initial_message_.z == 0) {
      initial_message_ = msg;
      initial_point_for_statistics_ = msg;
    }

    double x_input = 0;
    double y_input = 0;
    double z_input = 0;
    double angle_input = 0;
    string command;

    if(!coordinate_history_.empty()) {
      if (!(old_sync_counter_ < new_sync_counter_)) {
        cluster_extraction::objectDistance distanceMessage;
        distanceMessage.distance = abs(coordinate_history_[coordinate_history_.size() - 3]);
        distanceMessage.counter = old_sync_counter_;
        distanceMessage.manipulatorIsClose = manipulator_is_close_;
        object_distance_pub_.publish(distanceMessage);
        return;
      }
    }

    cout << "Initial message: " << initial_message_.x << " " << initial_message_.y << " " << initial_message_.z << endl;
    cout << "Received point: " << msg.x << " " << msg.y << " " << msg.z << endl;
    cout << "Delta X: " << manipulator_adjustment_x_ << endl;
    cout << "Automate, type coordinates, fixed steps  or refresh object? (a/t/f/r)" << endl;
    if (!dont_ask_for_user_input_){
      cin >> command;
    }
    if (command == "a" || dont_ask_for_user_input_) {
      cout << "Grab Object: " << grab_object_ << endl;
      cout << "Grab Next Step: " << grab_next_step_ << endl;
      if (grab_next_step_) {
        gripObject();
        usleep(sleep_between_gripping_);
        manipulator_is_close_ = false;
        moveManipulator(0, coordinate_history_[coordinate_history_.size()-3], raise_object_when_grabbed_z_, 0);
        usleep(sleep_between_steps_);
        moveManipulator(0, coordinate_history_[coordinate_history_.size()-3], 0, 0);
        usleep(sleep_between_steps_);
        resetGripper();
        usleep(sleep_between_gripping_);
        grabbed_object_ = false;
        moveManipulator(manipulator_first_step_x_, manipulator_first_step_y_, manipulator_first_step_z_, manipulator_camera_angle_);
        dont_ask_for_user_input_ = false;
        coordinate_history_.clear();
        initial_message_.x = 0;
        initial_message_.y = 0;
        initial_message_.z = 0;
        grab_object_ = false;
        add_final_step_delta_z_ = true;
      }
      else {
        dont_ask_for_user_input_ = true;
        automateManipulatorStep();
        usleep(sleep_between_steps_);
      }
      if (grab_object_) {
        grab_next_step_ = true;
      }
    }
    else if (command == "t"){
      cout << "Enter x coordinate" << endl;
      cin >> x_input;
      cout << "Enter y coordinate" << endl;
      cin >> y_input;
      cout << "Enter z coordinate" << endl;
      cin >> z_input;
      cout << "Enter angle" << endl;
      cin >> angle_input;

      manipulator_is_close_ = false;
      manipulator_adjustment_x_ = 0;
      moveManipulator(x_input, y_input, z_input, angle_input);
    }
    else if (command == "f") {
      if (grab_next_step_) {
        gripObject();
      }
      if (grab_object_) {
        grab_next_step_ = true;
      }
      else {
        automateManipulatorStep();
        cout << "Type something to start new cycle" << endl;
        string s;
        cin >> s;
      }
    }
    else if (command == "r") {
      initial_message_ = msg;
      initial_point_for_statistics_ = msg;
    }
    else {
      return;
    }
  }



  void automateManipulatorStep() {
    if (coordinate_history_.empty()) {
      moveManipulator(manipulator_first_step_x_, manipulator_first_step_y_, manipulator_first_step_z_, manipulator_camera_angle_);
    }
    else if (coordinate_history_[coordinate_history_.size() - 3] < min_y_before_moving_manipulator_close_) {
      moveManipulator(
          coordinate_history_[coordinate_history_.size() - 4], coordinate_history_[coordinate_history_.size() - 3]*manipulator_distance_percentage_, coordinate_history_[coordinate_history_.size() - 2]*
          manipulator_distance_percentage_, coordinate_history_[coordinate_history_.size() - 1]);
    }
    else {
      if (add_final_step_delta_z_) {
        initial_message_.z = initial_message_.z + final_step_delta_z_;
        add_final_step_delta_z_ = false;
      }
      moveManipulator(coordinate_history_[coordinate_history_.size() - 4], 0 + close_step_adder_, 0, 0);
      close_step_adder_ = close_step_adder_ + move_manipulator_by_on_final_steps_;
      manipulator_is_close_ = true;
    }
  }

  void gripObject() {
    cout << "Grab it!" << endl;
    phoebe_planning_msgs::GripperRequest gripper_request;
    gripper_request.request.angle = grip_angle_;
    gripper_request.request.distance = gripper_request_distace_;
    gripper_request.request.duration = gripper_request_duration_;
    gripper_request.request.id = gripper_request_id_;

    if (gripper_request_client_.call(gripper_request)) {
      if (gripper_request.response.result) {
        ROS_INFO("Gripping request succeeded");
        grabbed_object_ = true;
      }
    }
    cout << "ESTIMATED LOCATION: X: " << initial_point_for_statistics_.x << " Y: " << initial_point_for_statistics_.y << " Z: " << initial_point_for_statistics_.z << endl;
    cout << "ACTUAL LOCATION: X: " << initial_message_.x << " Y: " << initial_message_.y + coordinate_history_[coordinate_history_.size()-3]<< " Z: " << initial_message_.z << endl;
  }

  void moveManipulator(double x_input, double y_input, double z_input, double angle_input) {
    old_sync_counter_ = new_sync_counter_;
    initial_message_.x = initial_message_.x + manipulator_adjustment_x_;
    double angle_max = angle_input + 0.14; // 8 kraadi
    double angle_min = angle_input - 0.14;
    double difference = (angle_max - angle_min)/2;
    int steps = 10;
    double delta_z = 0;
    cout << "Input X: " << x_input << " Y: " << y_input << " Z: " << z_input << endl;
    for (int i = 0; i < steps; i++) {
      double variation_step = difference/(steps/2);
      double j = i;
      double possible_angle;
      if ((int)j % 2 == 0) {
        j = j/2;
        possible_angle = (angle_min + difference) + variation_step * j;
        cout << "Angle: " << possible_angle << endl;
      }
      else {
        j = (j/2 + 0.5);
        possible_angle = (angle_min + difference) - variation_step * j;
        cout << "Angle: " << possible_angle << endl;
      }

      phoebe_planning_msgs::PlanningRequest planning_request;
      planning_request.request.pose = publishQuaternion(initial_message_, x_input, y_input, z_input + delta_z,
                                                          possible_angle);
      delta_z = -0.00001 * i; // 0.1 cm

      if (y_input > min_y_to_save_manipulator_pose_) {
        planning_request.request.pose.pose.orientation = last_pose_;
        planning_request.request.pose.pose.position.x = initial_message_.x;
        planning_request.request.pose.pose.position.y = initial_message_.y + y_input;
        planning_request.request.pose.pose.position.z = initial_message_.z + z_input;
      }
      else if (grabbed_object_) {
        planning_request.request.pose.pose.orientation = last_pose_;
        planning_request.request.pose.pose.position.x = x_input;
        planning_request.request.pose.pose.position.y = y_input;
        planning_request.request.pose.pose.position.z = z_input;
      }
      else {
        last_pose_ = planning_request.request.pose.pose.orientation;
      }

      cout << "PlanningRequest X: " << planning_request.request.pose.pose.position.x << " Y: " << planning_request.request.pose.pose.position.y << " Z: " << planning_request.request.pose.pose.position.z << endl;

      planning_request.request.id = planning_request_id_; //manipulator ID (0=any, 1=left only, 2=right only)
      if (manipulator_is_close_) {
        planning_request.request.duration = close_step_duration_;
      }
      else {
        planning_request.request.duration = step_duration_;
      }
      vector<float> array;
      planning_request.request.joint_angles = array;

      phoebe_planning_msgs::MotionRequest motion_request;
      if (planning_request_client_.call(planning_request)) {
        if (planning_request.response.result) {
          ROS_INFO("Planning request succeeded");
          motion_request.request.trajectory = planning_request.response.trajectory;

          string move;
          if (!dont_ask_for_user_input_) {
            cout << "To move the manipulator type: move" << endl;
            cin>>move;
          }
          else {
            move = "move";
          }
          if (move == "move") {
            motion_request.request.tolerance = motion_request_tolerance_;
            if (motion_request_client_.call(motion_request)) {
              if(motion_request.response.result) {
                ROS_INFO("Motion request succeeded");
                break;
              }
              else {
                ROS_ERROR("Motion request failed");
                break;
              }
            }
          }
          else {
            break;
          }
        }
        else {
          ROS_ERROR("Planning request failed");
        }
      }
    }
  }

  geometry_msgs::PoseStamped publishQuaternion(geometry_msgs::Point &msg, double x_input, double y_input, double z_input, double angle_input) {
    cluster_extraction::objectDistance distance_message;
    distance_message.distance = abs(y_input);
    distance_message.counter = old_sync_counter_;
    distance_message.manipulatorIsClose = manipulator_is_close_;

    object_distance_pub_.publish(distance_message);
    coordinate_history_.clear();
    coordinate_history_.push_back(x_input);
    coordinate_history_.push_back(y_input);
    coordinate_history_.push_back(z_input);
    coordinate_history_.push_back(angle_input);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped = calculateQuaternion(x_input, y_input, z_input, msg);

    geometry_msgs::Pose::_orientation_type rotation_quaternion;
    rotation_quaternion.w = cos(angle_input /2);
    rotation_quaternion.x = sin(angle_input /2);
    rotation_quaternion.y = 0;
    rotation_quaternion.z = 0;

    pose_stamped.pose.orientation = multiplyQuaternion(pose_stamped.pose.orientation, rotation_quaternion);

    geometry_msgs::PointStamped point_out;
    point_out.header.frame_id = base_link_;
    point_out.point.x = msg.x;
    point_out.point.y = msg.y;
    point_out.point.z = msg.z;

    pose_pub_.publish(pose_stamped);
    pub_.publish(point_out);

    return pose_stamped;
  }

  void floatCallback(const cluster_extraction::manipulatorAdjustment &msg) {
    manipulator_adjustment_x_ = msg.deltax;
    grab_object_ = msg.grab;
    new_sync_counter_ = msg.counter;
  }

  geometry_msgs::Pose::_orientation_type multiplyQuaternion(geometry_msgs::Pose::_orientation_type q2, geometry_msgs::Pose::_orientation_type q1) {

    geometry_msgs::Pose::_orientation_type quaternion;

    quaternion.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    quaternion.x = q1.w*q2.x + q1.x*q2.w - q1.y*q2.z + q1.z*q2.y;
    quaternion.y = q1.w*q2.y + q1.x*q2.z + q1.y*q2.w - q1.z*q2.x;
    quaternion.z = q1.w*q2.z - q1.x*q2.y + q1.y*q2.x + q1.z*q2.w;

//    t0=(r0q0−r1q1−r2q2−r3q3)
//    t1=(r0q1+r1q0−r2q3+r3q2)
//    t2=(r0q2+r1q3+r2q0−r3q1)
//    t3=(r0q3−r1q2+r2q1+r3q0)

    return quaternion;
  }

  double angleBetweenTwoVectors(geometry_msgs::Point p1, geometry_msgs::Point p2) {

    double angle = (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z) / (sqrt(p1.x*p1.x + p1.y*p1.y + p1.z*p1.z) * sqrt(p2.x*p2.x + p2.y*p2.y + p2.z*p2.z));
    if (angle < -1) {
      angle = -1;
    }
    else if (angle > 1) {
      angle = 1;
    }

    return acos(angle);

  }

  geometry_msgs::PointStamped::_point_type vectorCrossProduct(geometry_msgs::Point p1, geometry_msgs::Point p2) {

    geometry_msgs::PointStamped::_point_type vector;

    vector.x = p1.y*p2.z - p2.y*p1.z;
    vector.y = p1.z*p2.x - p1.x*p2.z;
    vector.z = p1.x*p2.y - p2.x*p1.y;

//    x2y3 - x3y2, x3y1 - x1y3, x1y2 - x2y1

    return vector;

  }

  geometry_msgs::Pose::_orientation_type rotationVectorToQuaternion(geometry_msgs::PointStamped::_point_type rotation_vector, double angle) {

    double magnitude = sqrt(
        rotation_vector.x* rotation_vector.x + rotation_vector.y* rotation_vector.y + rotation_vector.z*
                                                                                      rotation_vector.z);

    double beta_x = -rotation_vector.x / magnitude;
    double beta_y = -rotation_vector.y / magnitude;
    double beta_z = -rotation_vector.z / magnitude;

    geometry_msgs::Pose::_orientation_type quaternion;

    quaternion.w = cos(angle/2);
    quaternion.x = sin(angle/2) * beta_x;
    quaternion.y = sin(angle/2) * beta_y;
    quaternion.z = sin(angle/2) * beta_z;

    return quaternion;

  }

  geometry_msgs::PoseStamped calculateQuaternion (double x, double y, double z, geometry_msgs::Point msg) {

    geometry_msgs::Point manipulator;
    manipulator.x = msg.x + x;
    manipulator.y = msg.y + y;
    manipulator.z = msg.z + z;

    geometry_msgs::Point zero_zero_one_vector;
    zero_zero_one_vector.x = 0;
    zero_zero_one_vector.y = 0;
    zero_zero_one_vector.z = 1;

    geometry_msgs::Point manipulator_to_object_vector; // ( 0; -0.2; -0.15 )
    manipulator_to_object_vector.x = msg.x - manipulator.x;
    manipulator_to_object_vector.y = msg.y - manipulator.y;
    manipulator_to_object_vector.z = msg.z - manipulator.z;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = base_link_;

    pose_stamped.pose.position = manipulator;

    geometry_msgs::Point rotationVector = vectorCrossProduct(manipulator_to_object_vector, zero_zero_one_vector); // ( -0.2; 0; 0 )
    pose_stamped.pose.orientation = rotationVectorToQuaternion(rotationVector, angleBetweenTwoVectors(manipulator_to_object_vector, zero_zero_one_vector));

    return pose_stamped;
  }

  void resetGripper() {
    phoebe_planning_msgs::GripperRequest gripper_request;
    gripper_request.request.angle = reset_grip_angle_;
    gripper_request.request.distance = gripper_request_distace_;
    gripper_request.request.duration = gripper_request_duration_;
    gripper_request.request.id = gripper_request_id_;

    if (gripper_request_client_.call(gripper_request)) {
      if (gripper_request.response.result) {
        cout << "Resetting gripper" << endl;
        ROS_INFO("Gripping request succeeded");
      }
    }
  }
};


int main(int argc, char **argv) {

  ros::init(argc, argv, "hand_transform");

  ManipulatorController hand_transform;

  ros::Duration d(0.1);
  while (ros::ok()) {
    ros::spinOnce();
    d.sleep();
  }

  return 0;
}