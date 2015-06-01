#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/PolygonStamped.h>
#include <shape_msgs/SolidPrimitive.h>
#include <phoebe_planning_msgs/CollisionObject.h>


class ClusterExtraction {

public:
  ClusterExtraction() {
    point_cloud_sub_ = n_.subscribe("/camera/depth/points", 1, &ClusterExtraction::callback, this);

    filtered_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
    object_center_point_pub_ = n_.advertise<geometry_msgs::Point>("/object_center_point", 1);
    cloud_before_extraction_pub_ = n_.advertise<sensor_msgs::PointCloud2>("cloud_before_extraction", 1);
    collision_object_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("collision_object_polygon", 1);
    collision_object_client_ = n_.serviceClient<phoebe_planning_msgs::CollisionObject>("collision_object");

    base_link_ = "base_link";
    camera_depth_ = "camera_depth_optical_frame";
    wait_on_transform_ = true;

    n_.param("cluster_minimum_size_", cluster_minimum_size_, 1000);
    n_.param("cluster_average_min_z_", cluster_average_min_z_, 0.6);
    n_.param("table_filtering_delta_z_", table_filtering_delta_z_, 0.03);
    n_.param("percentage_of_points_to_filter", percentage_of_points_to_filter, 0.3); // 100% = 1.0
  }

private:
  ros::NodeHandle n_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher object_center_point_pub_;
  ros::Publisher cloud_before_extraction_pub_;
  ros::Publisher collision_object_pub_;
  ros::ServiceClient collision_object_client_;
  ros::Subscriber point_cloud_sub_;
  tf::TransformListener listener_;
  std::string base_link_;
  std::string camera_depth_;
  bool wait_on_transform_;
  int cluster_minimum_size_;
  double cluster_average_min_z_;
  double table_filtering_delta_z_;
  double percentage_of_points_to_filter;
  double cluster_max_x_;
  double cluster_min_x_;
  double cluster_max_y_;
  double cluster_min_y_;
  double cluster_max_z_;

  void callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    tf::StampedTransform transform;
    if (wait_on_transform_) {
      listener_.waitForTransform(camera_depth_, base_link_, ros::Time(0), ros::Duration(10.0));
      wait_on_transform_ = false;
    }
    listener_.lookupTransform(camera_depth_, base_link_, ros::Time(0), transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    *cloud = transformPointCloud(*msg, transform.inverse());

    //Downsampling a PointCloud using a VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filterPointCloud(cloud, cloud_filtered);

    segmentPointCloud(cloud_filtered);

    //Publish PointCloud before extraction
    transformAndPublishPointCloud(*cloud_filtered, transform, cloud_before_extraction_pub_);

    extractCluster(cloud_filtered);

//    publishCollisionObject();

    object_center_point_pub_.publish(calculateClusterCenter(*cloud_filtered));

    transformAndPublishPointCloud(*cloud_filtered, transform, filtered_cloud_pub_);
  }

  void extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) {

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

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
      double totalZ = 0.0;
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
        cloud_cluster.points.push_back(cloud_filtered->points[*pit]);
        totalZ += cloud_filtered->points[*pit].z;
      }

      double averageZ = totalZ / cloud_cluster.points.size();
      cloud_cluster.width = cloud_cluster.points.size();
      cloud_cluster.height = 1;
      cloud_cluster.is_dense = true;

      if (cloud_cluster.points.size() > cluster_minimum_size_ && averageZ > cluster_average_min_z_) {

        cluster_max_x_ = cloud_cluster.points[0].x;
        cluster_min_x_ = cloud_cluster.points[0].x;
        cluster_max_y_ = cloud_cluster.points[0].y;
        cluster_min_y_ = cloud_cluster.points[0].y;
        cluster_max_z_ = averageZ + table_filtering_delta_z_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0; i < cloud_cluster.points.size(); i++) {
          if (cloud_cluster.points[i].z > cluster_max_z_) {
            filtered_cloud_cluster->points.push_back(cloud_cluster.points[i]);
          }
          else{
            if (cloud_cluster.points[i].x > cluster_max_x_) {
              cluster_max_x_ = cloud_cluster.points[i].x;
            }
            if (cloud_cluster.points[i].x < cluster_min_x_) {
              cluster_min_x_ = cloud_cluster.points[i].x;
            }
            if (cloud_cluster.points[i].y > cluster_max_y_) {
              cluster_max_y_ = cloud_cluster.points[i].y;
            }
            if (cloud_cluster.points[i].y < cluster_min_y_) {
              cluster_min_y_ = cloud_cluster.points[i].y;
            }
          }
        }

        cloud_filtered->swap(*filtered_cloud_cluster);

        break;
      }
    }
  }

  void publishCollisionObject() {
    //Collision object for rviz visualisation
    collision_object_pub_.publish(getPolygonStamped());

    //Collision object for the robot
    phoebe_planning_msgs::CollisionObject collisionObject;
    collisionObject.request.shape = getPrimitive(1);
    collisionObject.request.pose = getStamped();
    collisionObject.request.action = 0;
    if (collision_object_client_.call(collisionObject)) {
      ROS_INFO("Collision object succeeded");
    }
    else {
      ROS_ERROR("Collision object failed");
    }
  }

  void segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) {

    pcl::PointCloud<pcl::PointXYZ> cloud_f;
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

    int nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > percentage_of_points_to_filter * nr_points) {
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
      extract.filter(cloud_f);
      *cloud_filtered = cloud_f;
    }
  }

  void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) {

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
  }

  pcl::PointCloud<pcl::PointXYZ> transformPointCloud(const sensor_msgs::PointCloud2 &msg, tf::Transform transform) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> input_PointCloud1;
    pcl::PCLPointCloud2 input_PointCloud2;
    pcl_conversions::moveToPCL(const_cast<sensor_msgs::PointCloud2 &>(msg), input_PointCloud2);
    pcl::fromPCLPointCloud2(input_PointCloud2, input_PointCloud1);
    pcl_ros::transformPointCloud(input_PointCloud1, cloud, transform);
    return cloud;
  }

  geometry_msgs::Point calculateClusterCenter(const pcl::PointCloud<pcl::PointXYZ> &cloud_filtered) {

    geometry_msgs::Point central_point;
    double totalPointsX = 0.0;
    double totalPointsY = 0.0;
    double totalPointsZ = 0.0;
    for (int i = 0; i < cloud_filtered.points.size(); i++) {
      totalPointsX += cloud_filtered.points[i].x;
      totalPointsY += cloud_filtered.points[i].y;
      totalPointsZ += cloud_filtered.points[i].z;
    }

    central_point.x = totalPointsX/cloud_filtered.points.size();
    central_point.y = totalPointsY/cloud_filtered.points.size();
    central_point.z = totalPointsZ/cloud_filtered.points.size();

    return central_point;
  }

  geometry_msgs::PolygonStamped getPolygonStamped() {

    geometry_msgs::PolygonStamped polygon_stamped;
    geometry_msgs::Point32 point1;
    geometry_msgs::Point32 point2;
    geometry_msgs::Point32 point3;
    geometry_msgs::Point32 point4;
    geometry_msgs::Point32 point5;

    point1.x = cluster_min_x_;
    point1.y = cluster_min_y_;
    point1.z = cluster_max_z_;

    point2.x = cluster_max_x_;
    point2.y = cluster_min_y_;
    point2.z = cluster_max_z_;

    point3.x = cluster_max_x_;
    point3.y = cluster_max_y_;
    point3.z = cluster_max_z_;

    point4.x = cluster_min_x_;
    point4.y = cluster_max_y_;
    point4.z = cluster_max_z_;

    point5.x = (cluster_max_x_ + cluster_min_x_)/2;
    point5.y = (cluster_max_y_ + cluster_min_y_)/2;
    point5.z = cluster_max_z_ /2;


    polygon_stamped.header.frame_id = base_link_;
    polygon_stamped.polygon.points.push_back(point1);
    polygon_stamped.polygon.points.push_back(point2);
    polygon_stamped.polygon.points.push_back(point3);
    polygon_stamped.polygon.points.push_back(point4);
    polygon_stamped.polygon.points.push_back(point5);

    return polygon_stamped;
  }

  geometry_msgs::PoseStamped getStamped() {

    geometry_msgs::PoseStamped solid_primitive_center;
    solid_primitive_center.header.frame_id = base_link_;
    solid_primitive_center.pose.position.x = (cluster_max_x_ + cluster_min_x_)/2;
    solid_primitive_center.pose.position.y = (cluster_max_y_ + cluster_min_y_)/2;
    solid_primitive_center.pose.position.z = cluster_max_z_/2;
    solid_primitive_center.pose.orientation.x = 0;
    solid_primitive_center.pose.orientation.y = 0;
    solid_primitive_center.pose.orientation.z = 0;
    solid_primitive_center.pose.orientation.w = 1;

    return solid_primitive_center;
  }

  shape_msgs::SolidPrimitive getPrimitive(int type) {

    shape_msgs::SolidPrimitive solid_primitive;
    solid_primitive.type = type; //BOX = 1
    solid_primitive.dimensions.resize(3);
    solid_primitive.dimensions[0] = std::abs(cluster_max_x_ - cluster_min_x_);
    solid_primitive.dimensions[1] = std::abs(cluster_max_y_ - cluster_min_y_);
    solid_primitive.dimensions[2] = cluster_max_z_;

    return solid_primitive;
  }

  void transformAndPublishPointCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud, tf::StampedTransform &transform, ros::Publisher pub) {
    sensor_msgs::PointCloud2 pointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(input_cloud, *pointCloud1, transform);
    pcl::toROSMsg(*pointCloud1, pointCloud2);
    pub.publish(pointCloud2);
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "cluster_extraction");

  ClusterExtraction cluster_extraction;

  ros::Duration d(1);
  while (ros::ok()) {

    ros::spinOnce();
    d.sleep();
  }

  return 0;
}

