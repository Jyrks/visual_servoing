#include "ros/ros.h"
#include <iostream>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <phoebe_planning_msgs/PlanningRequest.h>
#include <phoebe_planning_msgs/MotionRequest.h>
#include <phoebe_planning_msgs/GripperRequest.h>
#include <cluster_extraction/manipulatorAdjustment.h>
#include <cluster_extraction/objectDistance.h>

using namespace std;

class HandTransform {
public:
  HandTransform() {

    adjust_manipulator_sub_ = n_.subscribe("/adjust_manipulator_x", 1, &HandTransform::floatCallback, this);
    object_center_sub_ = n_.subscribe("/object_center_point", 1, &HandTransform::callback, this);

    pub_ = n_.advertise<geometry_msgs::PointStamped>("/manipulator_target_point", 1);
    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/manipulator_target_pose", 1);
    object_distance_pub_ = n_.advertise<cluster_extraction::objectDistance>("/manipulator_to_object_distance", 1);

    planning_request_client_ = n_.serviceClient<phoebe_planning_msgs::PlanningRequest>("phoebe_planning_request");
    motion_request_client_ = n_.serviceClient<phoebe_planning_msgs::MotionRequest>("phoebe_motion_request");
    gripper_request_client_ = n_.serviceClient<phoebe_planning_msgs::GripperRequest>("phoebe_gripper_request");

    initialMessage.x = 0;
    initialMessage.y = 0;
    initialMessage.z = 0;
    base_link_ = "base_link";
    manipulatorAdjustmentX = 0;
    manipulatorIsCloseStepAdder = 0;
    oldSyncCounter = 0;
    newSyncCounter = 0;
    grabObject = false;
    manipulatorIsClose = false;
    grabbedObject = false;
    addFinalStepDeltaZ = true;
    dontAskForUserInput = false;
    finalStepDeltaZ = -0.01;
    grabNextStep = false;

    resetGripper();

    n_.param("sleepDurationMicroSeconds", sleepDurationMicroSeconds, 1000000);
    n_.param("gripAngle", gripAngle, -0.36);
  }

  void callback(const geometry_msgs::Point &msg) {

    if (initialMessage.x == 0 && initialMessage.y == 0 && initialMessage.z == 0) {
      initialMessage = msg;
      initialPointForStatistics = msg;
    }

    double xInput = 0;
    double yInput = 0;
    double zInput = 0;
    double angleInput = 0;
    string command;

    if(!coordinateHistory.empty()) {
      if (!(oldSyncCounter < newSyncCounter)) {
        cluster_extraction::objectDistance distanceMessage;
        distanceMessage.distance = abs(coordinateHistory[coordinateHistory.size() - 3]);
        distanceMessage.counter = oldSyncCounter;
        distanceMessage.manipulatorIsClose = manipulatorIsClose;
        object_distance_pub_.publish(distanceMessage);
        return;
      }
    }

    cout << "Initial message: " << initialMessage.x << " " << initialMessage.y << " " << initialMessage.z << endl;
    cout << "Received point: " << msg.x << " " << msg.y << " " << msg.z << endl;
    cout << "Delta X: " << manipulatorAdjustmentX << endl;
    cout << "Automate, type coordinates, fixed steps  or refresh object? (a/t/f/r)" << endl;
    if (!dontAskForUserInput){
      cin >> command;
    }
    if (command == "a" || dontAskForUserInput) {
      cout << "Grab Object: " << grabObject << endl;
      cout << "Grab Next Step: " << grabNextStep << endl;
      if (grabNextStep) {
        gripObject();
        usleep(3000000);
        manipulatorIsClose = false;
        moveManipulator(0, coordinateHistory[coordinateHistory.size()-3], 0.05, 0);
        usleep(1000000);
        moveManipulator(0.01, coordinateHistory[coordinateHistory.size()-3], 0, 0);
        usleep(1000000);
        resetGripper();
        usleep(3000000);
        grabbedObject = false;
        moveManipulator(0, -0.15, 0.15, 0.61);
        dontAskForUserInput = false;
        coordinateHistory.clear();
        initialMessage.x = 0;
        initialMessage.y = 0;
        initialMessage.z = 0;
        grabObject = false;
        addFinalStepDeltaZ = true;
      }
      else {
        dontAskForUserInput = true;
        automateManipulatorStep();
        usleep(sleepDurationMicroSeconds);
      }
      if (grabObject) {
        grabNextStep = true;
      }
    }
    else if (command == "t"){
      cout << "Enter x coordinate" << endl;
      cin >> xInput;
      cout << "Enter y coordinate" << endl;
      cin >> yInput;
      cout << "Enter z coordinate" << endl;
      cin >> zInput;
      cout << "Enter angle" << endl;
      cin >> angleInput;

      manipulatorIsClose = false;
      manipulatorAdjustmentX = 0;
      moveManipulator(xInput, yInput, zInput, angleInput);
    }
    else if (command == "f") {
      if (grabNextStep) {
        gripObject();
      }
      if (grabObject) {
        grabNextStep = true;
      }
      else {
        automateManipulatorStep();
        cout << "Type something to start new cycle" << endl;
        string s;
        cin >> s;
      }
    }
    else if (command == "r") {
      initialMessage = msg;
      initialPointForStatistics = msg;
    }
    else {
      return;
    }
  }



  void automateManipulatorStep() {
    if (coordinateHistory.empty()) {
      moveManipulator(0, -0.15, 0.15, 0.61);
    }
    else if (coordinateHistory[coordinateHistory.size() - 3] < -0.06) {
      moveManipulator(coordinateHistory[coordinateHistory.size() - 4], coordinateHistory[coordinateHistory.size() - 3] * 0.7, coordinateHistory[coordinateHistory.size() - 2] * 0.7, coordinateHistory[coordinateHistory.size() - 1]);
    }
    else {
      if (addFinalStepDeltaZ) {
        initialMessage.z = initialMessage.z + finalStepDeltaZ;
        addFinalStepDeltaZ = false;
      }
      moveManipulator(coordinateHistory[coordinateHistory.size() - 4], 0 + manipulatorIsCloseStepAdder, 0, 0);
      manipulatorIsCloseStepAdder = manipulatorIsCloseStepAdder + 0.02;
      manipulatorIsClose = true;
    }
  }

  void gripObject() {

    cout << "Grab it!" << endl;
    phoebe_planning_msgs::GripperRequest gripperRequest;
    gripperRequest.request.angle = gripAngle;
    gripperRequest.request.distance = 0;
    gripperRequest.request.duration = 1;
    gripperRequest.request.id = 2;

    if (gripper_request_client_.call(gripperRequest)) {
      if (gripperRequest.response.result) {
        ROS_INFO("Gripping request succeeded");
        grabbedObject = true;
      }
    }
    cout << "ESTIMATED LOCATION: X: " << initialPointForStatistics.x << " Y: " << initialPointForStatistics.y << " Z: " << initialPointForStatistics.z << endl;
    cout << "ACTUAL LOCATION: X: " << initialMessage.x << " Y: " << initialMessage.y + coordinateHistory[coordinateHistory.size()-3]<< " Z: " << initialMessage.z << endl;
  }

  void moveManipulator(double xInput, double yInput, double zInput, double angleInput) {
    oldSyncCounter = newSyncCounter;
    initialMessage.x = initialMessage.x + manipulatorAdjustmentX;
    double angleMax = angleInput + 0.14; // 8 kraadi
    double angleMin = angleInput - 0.14;
    double difference = (angleMax - angleMin)/2;
    int steps = 10;
    double deltaZ = 0;
    cout << "Input X: " << xInput << " Y: " << yInput << " Z: " << zInput << endl;
    for (int i = 0; i < steps; i++) {
      double differenceStep = difference/(steps/2);
      double j = i;
      double  possibleAngle;
      if ((int)j % 2 == 0) {
        j = j/2;
        possibleAngle = (angleMin + difference) + differenceStep * j;
        cout << "Angle: " << possibleAngle << endl;
      }
      else {
        j = (j/2 + 0.5);
        possibleAngle = (angleMin + difference) - differenceStep * j;
        cout << "Angle: " << possibleAngle << endl;
      }

      phoebe_planning_msgs::PlanningRequest planningRequestSrv;
      planningRequestSrv.request.pose = publishQuaternion(initialMessage, xInput, yInput, zInput + deltaZ, possibleAngle);
      deltaZ = -0.00001 * i; // 0.1 cm

      if (yInput > -0.03) {
        planningRequestSrv.request.pose.pose.orientation = lastPose;
        planningRequestSrv.request.pose.pose.position.x = initialMessage.x;
        planningRequestSrv.request.pose.pose.position.y = initialMessage.y + yInput;
        planningRequestSrv.request.pose.pose.position.z = initialMessage.z + zInput;
      }
      else if (grabbedObject) {
        planningRequestSrv.request.pose.pose.orientation = lastPose;
        planningRequestSrv.request.pose.pose.position.x = xInput;
        planningRequestSrv.request.pose.pose.position.y = yInput;
        planningRequestSrv.request.pose.pose.position.z = zInput;
      }
      else {
        lastPose = planningRequestSrv.request.pose.pose.orientation;
      }

      cout << "PlanningRequest X: " << planningRequestSrv.request.pose.pose.position.x << " Y: " << planningRequestSrv.request.pose.pose.position.y << " Z: " << planningRequestSrv.request.pose.pose.position.z << endl;

      planningRequestSrv.request.id = 1; //manipulator ID (0=any, 1=left only, 2=right only)
      if (manipulatorIsClose) {
        planningRequestSrv.request.duration = 4.0;
      }
      else {
        planningRequestSrv.request.duration = 4.0;
      }
      vector<float> array;
      planningRequestSrv.request.joint_angles = array;

      phoebe_planning_msgs::MotionRequest motionRequestSrv;
      if (planning_request_client_.call(planningRequestSrv)) {
        if (planningRequestSrv.response.result) {
          ROS_INFO("Planning request succeeded");
          motionRequestSrv.request.trajectory = planningRequestSrv.response.trajectory;

          string move;
          if (!dontAskForUserInput) {
            cout << "To move the manipulator type: move" << endl;
            cin>>move;
          }
          else {
            move = "move";
          }
          if (move == "move") {
            motionRequestSrv.request.tolerance = 0.1;
            if (motion_request_client_.call(motionRequestSrv)) {
              if(motionRequestSrv.response.result) {
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

  geometry_msgs::PoseStamped publishQuaternion(geometry_msgs::Point &msg, double xInput, double yInput, double zInput, double angleInput) {
    cluster_extraction::objectDistance distanceMessage;
    distanceMessage.distance = abs(yInput);
    distanceMessage.counter = oldSyncCounter;
    distanceMessage.manipulatorIsClose = manipulatorIsClose;

    object_distance_pub_.publish(distanceMessage);
    coordinateHistory.clear();
    coordinateHistory.push_back(xInput);
    coordinateHistory.push_back(yInput);
    coordinateHistory.push_back(zInput);
    coordinateHistory.push_back(angleInput);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped = calculateQuaternion(xInput, yInput, zInput, msg);

    geometry_msgs::Pose::_orientation_type rotationQuaternion;
    rotationQuaternion.w = cos(angleInput/2);
    rotationQuaternion.x = sin(angleInput/2);
    rotationQuaternion.y = 0;
    rotationQuaternion.z = 0;

    poseStamped.pose.orientation = multiplyQuaternion(poseStamped.pose.orientation, rotationQuaternion);

    geometry_msgs::PointStamped pointOut;
    pointOut.header.frame_id = base_link_;
    pointOut.point.x = msg.x;
    pointOut.point.y = msg.y;
    pointOut.point.z = msg.z;

    pose_pub_.publish(poseStamped);
    pub_.publish(pointOut);

    return poseStamped;
  }

  void floatCallback(const cluster_extraction::manipulatorAdjustment &msg) {
    manipulatorAdjustmentX = msg.deltax;
    grabObject = msg.grab;
    newSyncCounter = msg.counter;
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

  geometry_msgs::Pose::_orientation_type rotationVectorToQuaternion(geometry_msgs::PointStamped::_point_type rotVec, double angle) {

    double magnitude = sqrt(rotVec.x*rotVec.x + rotVec.y*rotVec.y + rotVec.z*rotVec.z);

    double beta_x = -rotVec.x / magnitude;
    double beta_y = -rotVec.y / magnitude;
    double beta_z = -rotVec.z / magnitude;

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

    geometry_msgs::Point zeroZeroOneVector;
    zeroZeroOneVector.x = 0;
    zeroZeroOneVector.y = 0;
    zeroZeroOneVector.z = 1;

    geometry_msgs::Point manipulatorToObjectVector; // ( 0; -0.2; -0.15 )
    manipulatorToObjectVector.x = msg.x - manipulator.x;
    manipulatorToObjectVector.y = msg.y - manipulator.y;
    manipulatorToObjectVector.z = msg.z - manipulator.z;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = base_link_;

    poseStamped.pose.position = manipulator;

    geometry_msgs::Point rotationVector = vectorCrossProduct(manipulatorToObjectVector, zeroZeroOneVector); // ( -0.2; 0; 0 )
    poseStamped.pose.orientation = rotationVectorToQuaternion(rotationVector, angleBetweenTwoVectors(manipulatorToObjectVector, zeroZeroOneVector));

    return poseStamped;
  }

  void resetGripper() {
    phoebe_planning_msgs::GripperRequest gripperRequest;
    gripperRequest.request.angle = 0.40;
    gripperRequest.request.distance = 0;
    gripperRequest.request.duration = 1;
    gripperRequest.request.id = 2;

    if (gripper_request_client_.call(gripperRequest)) {
      if (gripperRequest.response.result) {
        cout << "Resetting gripper" << endl;
        ROS_INFO("Gripping request succeeded");
      }
    }
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
  std::string base_link_;
  double manipulatorAdjustmentX;
  vector<double> coordinateHistory;
  double manipulatorIsCloseStepAdder;
  geometry_msgs::Point initialMessage;
  int oldSyncCounter;
  int newSyncCounter;
  bool grabObject;
  bool manipulatorIsClose;
  geometry_msgs::Quaternion lastPose;
  double finalStepDeltaZ;
  bool grabbedObject;
  bool addFinalStepDeltaZ;
  int sleepDurationMicroSeconds;
  bool dontAskForUserInput;
  double gripAngle;
  geometry_msgs::Point initialPointForStatistics;
  bool grabNextStep;
};


int main(int argc, char **argv) {

  ros::init(argc, argv, "hand_transform");

  HandTransform handTransform;

  ros::Duration d(0.1);
  while (ros::ok()) {
    ros::spinOnce();
    d.sleep();
  }

  return 0;
}