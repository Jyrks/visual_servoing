#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <shape_msgs/SolidPrimitive.h>
#include <phoebe_planning_msgs/CollisionObject.h>


class SubscribeAndPublish {
public:
  SubscribeAndPublish() {
    sub_ = n_.subscribe("/camera/depth/points", 1, &SubscribeAndPublish::callback, this);

    pub_ = n_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
    point_pub_ = n_.advertise<geometry_msgs::Point>("/object_center_point", 1);
    clusters_pub_ = n_.advertise<sensor_msgs::PointCloud2>("cloud_before_extraction", 1);
    collision_object_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("collision_object_polygon", 1);
    collision_object_client_ = n_.serviceClient<phoebe_planning_msgs::CollisionObject>("collision_object");

    base_link_ = "base_link";
    camera_depth_ = "camera_depth_optical_frame";
    waitOnTransform_ = true;
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    tf::StampedTransform transform;
    if (waitOnTransform_ == true) {
      listener_.waitForTransform(camera_depth_, base_link_, ros::Time(0), ros::Duration(10.0));
      waitOnTransform_ = false;
    }
    listener_.lookupTransform(camera_depth_, base_link_, ros::Time(0), transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr inputCloud1 = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);

    //Transform PointCloud to camera_depth_optical_frame
    pcl_conversions::moveToPCL(const_cast<sensor_msgs::PointCloud2 &>(*msg), *inputCloud1);
    pcl::fromPCLPointCloud2(*inputCloud1, *cloud1);
    *cloud = *cloud1;
    pcl_ros::transformPointCloud(*cloud1, *cloud, transform.inverse());

    //Downsampling a PointCloud using a VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points) {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Get the points associated with the planar surface
      extract.filter(*cloud_plane);

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_f);
      *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    //Publish PointCloud before extraction
    sensor_msgs::PointCloud2 cloudClusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBeforeExtraction(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*cloud_filtered, *cloudBeforeExtraction, transform);
    pcl::toROSMsg(*cloudBeforeExtraction, cloudClusters);
    clusters_pub_.publish(cloudClusters);

    geometry_msgs::Point central_point;
    geometry_msgs::PolygonStamped polygonStamped;
    shape_msgs::SolidPrimitive solidPrimitive;
    geometry_msgs::PoseStamped solidPrimitiveCenter;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      double totalZ = 0.0;
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
        cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        totalZ += cloud_filtered->points[*pit].z;
      }

      double averageZ = totalZ / cloud_cluster->points.size();
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

//      cout << "Cloud" << j << " Average z: " << averageZ << endl;
      j++;

      if (cloud_cluster->points.size() > 1000 && averageZ > 0.6) {

        double maxX = cloud_cluster->points[i].x;
        double minX = cloud_cluster->points[i].x;
        double maxY = cloud_cluster->points[i].y;
        double minY = cloud_cluster->points[i].y;
        double maxZ = averageZ + 0.03;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloud_cluster->points.size(); i++) {
          if (cloud_cluster->points[i].z > maxZ) {
            filtered_cloud_cluster->points.push_back(cloud_cluster->points[i]);
          }
          else{
            if (cloud_cluster->points[i].x > maxX) {
              maxX = cloud_cluster->points[i].x;
            }
            if (cloud_cluster->points[i].x < minX) {
              minX = cloud_cluster->points[i].x;
            }
            if (cloud_cluster->points[i].y > maxY) {
              maxY = cloud_cluster->points[i].y;
            }
            if (cloud_cluster->points[i].y < minY) {
              minY = cloud_cluster->points[i].y;
            }

          }
        }

        solidPrimitive.type = 1; //BOX
        solidPrimitive.dimensions.resize(3);
        solidPrimitive.dimensions[0] = std::abs(maxX - minX);
        solidPrimitive.dimensions[1] = std::abs(maxY - minY);
        solidPrimitive.dimensions[2] = maxZ;

        solidPrimitiveCenter.header.frame_id = "base_link";
        solidPrimitiveCenter.pose.position.x = (maxX + minX)/2;
        solidPrimitiveCenter.pose.position.y = (maxY + minY)/2;
        solidPrimitiveCenter.pose.position.z = maxZ/2;
        solidPrimitiveCenter.pose.orientation.x = 0;
        solidPrimitiveCenter.pose.orientation.y = 0;
        solidPrimitiveCenter.pose.orientation.z = 0;
        solidPrimitiveCenter.pose.orientation.w = 1;

        geometry_msgs::Point32 point1;
        geometry_msgs::Point32 point2;
        geometry_msgs::Point32 point3;
        geometry_msgs::Point32 point4;
        geometry_msgs::Point32 point5;

        point1.x = minX;
        point1.y = minY;
        point1.z = maxZ;

        point2.x = maxX;
        point2.y = minY;
        point2.z = maxZ;

        point3.x = maxX;
        point3.y = maxY;
        point3.z = maxZ;

        point4.x = minX;
        point4.y = maxY;
        point4.z = maxZ;

        point5.x = (maxX + minX)/2;
        point5.y = (maxY + minY)/2;
        point5.z = maxZ/2;


        polygonStamped.header.frame_id = "base_link";
        polygonStamped.polygon.points.push_back(point1);
        polygonStamped.polygon.points.push_back(point2);
        polygonStamped.polygon.points.push_back(point3);
        polygonStamped.polygon.points.push_back(point4);
        polygonStamped.polygon.points.push_back(point5);

        cloud_filtered->swap(*filtered_cloud_cluster);

        //Compute centroid
        double totalPointsX = 0.0;
        double totalPointsY = 0.0;
        double totalPointsZ = 0.0;
        for (int i = 0; i < cloud_filtered->points.size(); i++) {
          totalPointsX += cloud_filtered->points[i].x;
          totalPointsY += cloud_filtered->points[i].y;
          totalPointsZ += cloud_filtered->points[i].z;
        }

        central_point.x = totalPointsX/cloud_filtered->points.size();
        central_point.y = totalPointsY/cloud_filtered->points.size();
        central_point.z = totalPointsZ/cloud_filtered->points.size();

        break;
      }
    }

    //Transform CloudCluster to base_link
    sensor_msgs::PointCloud2 cloudCluster;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformBack(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*cloud_filtered, *cloudTransformBack, transform);
    pcl::toROSMsg(*cloudTransformBack, cloudCluster);

    phoebe_planning_msgs::CollisionObject collisionObject;
    collisionObject.request.shape = solidPrimitive;
    collisionObject.request.pose = solidPrimitiveCenter;
    collisionObject.request.action = 0; //Add object

    //if (collision_object_client_.call(collisionObject)) {
    //  ROS_INFO("Collision object succeeded");
    //}
    //else {
    //  ROS_ERROR("Collision object failed");
    //}

    pub_.publish(cloudCluster);
    point_pub_.publish(central_point);
    collision_object_pub_.publish(polygonStamped);
  };

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher point_pub_;
  ros::Publisher clusters_pub_;
  ros::Publisher collision_object_pub_;
  ros::ServiceClient collision_object_client_;
  ros::Subscriber sub_;
  tf::TransformListener listener_;
  bool waitOnTransform_;
  std::string base_link_;
  std::string camera_depth_;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "subscribe_and_publish");

  SubscribeAndPublish SAPObject;

  ros::Duration d(1);
  while (ros::ok()) {

    ros::spinOnce();
    d.sleep();
  }

  return 0;
}

