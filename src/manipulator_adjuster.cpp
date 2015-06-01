#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "Corner.h"
#include <cluster_extraction/manipulatorAdjustment.h>
#include <cluster_extraction/objectDistance.h>


using namespace cv;
using namespace std;

class ManipulatorAdjuster {
public:
  ManipulatorAdjuster()
      : it_(nh_)
  {
    image_sub_ = it_.subscribe("/cam2/image_raw", 1, &ManipulatorAdjuster::imageCallback, this);
    delta_x_pub_ = nh_.advertise<cluster_extraction::manipulatorAdjustment>("/adjust_manipulator_x", 1);
    object_distance_sub_ = nh_.subscribe("/manipulator_to_object_distance", 1, &ManipulatorAdjuster::objectDistanceCallback, this);
    use_contour_detection_to_find_corners_ = true;
    manipulator_to_object_distance_ = 0;
    sync_counter_ = 0;
    is_manipulator_close_ = false;
    grab_object_ = false;

    nh_.param("object_grayscale_for_centering_", object_grayscale_for_centering_, 220);
    nh_.param("object_grayscale_for_grabbing_close_", object_grayscale_for_grabbing_close_, 160);
    nh_.param("look_for_grayscale_min_x_close_", look_for_grayscale_min_x_close_, 520);
    nh_.param("look_for_grayscale_max_x_close_", look_for_grayscale_max_x_close_, 630);
    nh_.param("look_for_grayscale_min_y_close_", look_for_grayscale_min_y_close_, 830);
    nh_.param("look_for_grayscale_max_y_close_", look_for_grayscale_max_y_close_, 860);
    nh_.param("look_for_grayscale_max_y_close_", horizontal_angle_, 25.9);
    nh_.param("cut_image_y_", cut_image_y_, 700);
    nh_.param("distance_to_object_close_manipulator_", distance_to_object_close_manipulator_, 0.05);
    nh_.param("grab_below_grayscale_point_count_", grab_below_grayscale_point_count_, 1000);
    nh_.param("detect_contours_with_size_min_", detect_contours_with_size_min_, 150);
    nh_.param("detect_contours_with_size_max_", detect_contours_with_size_max_, 400);
    nh_.param("detect_close_contours_with_size_min_", detect_close_contours_with_size_min_, 250);
    nh_.param("detect_contours_between_min_y_", detect_contours_between_min_y_, 384); //image_.rows*0.4
    nh_.param("detect_contours_between_max_y_", detect_contours_between_max_y_, 768);//image_.rows*0.8
    nh_.param("min_array_size_to_use_grayscale_detection_", min_array_size_to_use_grayscale_detection_, 500);
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher delta_x_pub_;
  ros::Subscriber object_distance_sub_;
  Mat src_gray_;
  Mat image_;
  Mat cut_image_;
  Mat cut_image_gray_;
  Mat drawing_;
  Mat cut_drawing_;
  Corner left_corner_;
  Corner right_corner_;
  vector<Point> object_grayscale_points_;
  bool use_contour_detection_to_find_corners_;
  bool is_manipulator_close_;
  bool grab_object_;
  bool grab_object_grayscale_close_;
  int corner_distance_;
  int sync_counter_;
  int image_middle_x_;
  int look_for_grayscale_max_x_close_;
  int look_for_grayscale_min_x_close_;
  int look_for_grayscale_max_y_close_;
  int look_for_grayscale_min_y_close_;
  int object_grayscale_for_centering_;
  int object_grayscale_for_grabbing_close_;
  int cut_image_y_;
  int grab_below_grayscale_point_count_;
  int detect_contours_with_size_min_;
  int detect_contours_with_size_max_;
  int detect_close_contours_with_size_min_;
  int detect_contours_between_min_y_;
  int detect_contours_between_max_y_;
  int min_array_size_to_use_grayscale_detection_;
  double manipulator_to_object_distance_;
  double object_location_pixels_x_;
  double horizontal_angle_;
  double distance_to_object_close_manipulator_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::Mat imageIn(cv_bridge::toCvCopy(msg, "bgr8")->image);
      image_ = imageIn;
      cut_image_ = imageIn;
      cut_image_.resize(cut_image_y_);
      cvtColor(cut_image_, cut_image_gray_, CV_BGR2GRAY );
      blur(cut_image_gray_, cut_image_gray_, Size(3,3) );
      cvtColor(image_, src_gray_, CV_BGR2GRAY );
      blur(src_gray_, src_gray_, Size(3,3) );
//      createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
      thresh_callback( 0, 0 );
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }


/** @function thresh_callback */
  void thresh_callback(int, void* )
  {
    Mat threshold_output;
    Mat threshold_cutOutput;
    vector<vector<Point> > contours;
    vector<vector<Point> > cutContours;
    vector<Vec4i> hierarchy;
    vector<Vec4i> cut_hierarchy;
    int image_middle_delta = 0;
    int thresh = 100;
    int max_thresh = 255;
    int contour_min_y = image_.rows;
    image_middle_x_ = image_.cols/2 - image_middle_delta;

    /// Detect edges using Threshold
    threshold(src_gray_, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    threshold(cut_image_gray_, threshold_cutOutput, thresh, 255, THRESH_BINARY );
    findContours( threshold_cutOutput, cutContours, cut_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    detectContoursAndDraw(contours, contour_min_y, threshold_output.size());

    detectCloseContours(cutContours, threshold_cutOutput.size());

    vector<Point> corner_points;
    goodFeaturesToTrack(src_gray_, corner_points, 4, 0.1, 50);

    if (left_corner_.counter_ > 20 && right_corner_.counter_ > 20) {
      use_contour_detection_to_find_corners_ = true;
    }

    if (use_contour_detection_to_find_corners_) {
      findCornersUsingContourDetection(contour_min_y, corner_points);
    }

    if (!use_contour_detection_to_find_corners_) {
      keepTrackingCornersAndFindObjectLocationPixels(corner_points);
    }

    cout << "Manipulator is close: " << is_manipulator_close_ << endl;
    if (is_manipulator_close_) {
      manipulator_to_object_distance_ = distance_to_object_close_manipulator_;
      calculateCloseManipulatorCorners(cutContours);
      detectGrayscaleSides();
      detectIfHaveToGrab();
    }
    else {
      grab_object_grayscale_close_ = false;
      grab_object_ = false;
      detectGrayscaleSides();
    }

    cout << "Corners distance " << corner_distance_ << endl;
    double delta_x = calculateDeltaX();

    cluster_extraction::manipulatorAdjustment adjustMessage;
    adjustMessage.counter = sync_counter_;
    adjustMessage.deltax = delta_x;
    adjustMessage.grab = grab_object_;
    delta_x_pub_.publish(adjustMessage);

    Mat tmp, dst;
    tmp = drawing_;
    dst = tmp;

    Mat tmp3, dst3;
    tmp3 = src_gray_;

    circle(tmp3, left_corner_.point_, 20, Scalar(0, 200, 0), 4, 16, 0);
    circle(tmp3, right_corner_.point_, 20, Scalar(0, 200, 0), 4, 16, 0);

    rectangle(tmp3, Point(look_for_grayscale_max_x_close_, look_for_grayscale_max_y_close_), Point(
        look_for_grayscale_min_x_close_,
        look_for_grayscale_min_y_close_), Scalar(200, 0, 0), 2, 8, 0);


    for (int i = 0; i < corner_points.size(); i++) {
      circle(tmp3, corner_points[i], 10, Scalar(200, 0, 0), 4, 10, 0);
    }

    pyrDown( tmp, dst, Size( tmp.cols/2, tmp.rows/2));
    pyrDown( tmp3, dst3, Size( tmp.cols/2, tmp.rows/2));

//    imshow("Contours", dst);
//    imshow("Cut image", cutDrawing_);
    imshow("Corners", dst3);

    waitKey(3);
  }

  void calculateCloseManipulatorCorners(vector<vector<Point> > &cut_contours) {
    double left_point_x = 0;
    double right_point_x = 0;
    double max_y = cut_image_.rows - 150;
    double min_y = cut_image_.rows - 250;
    double max_x = image_middle_x_ + 300;
    double minx_x = image_middle_x_ - 350;
    vector<Point> object_points;

    for (int i = 0; i < cut_contours.size(); i++) {
      if (cut_contours[i].size() > detect_close_contours_with_size_min_) {
        for (int j = 0; j < cut_contours[i].size(); j++) {
          if (cut_contours[i][j].y < max_y && cut_contours[i][j].y > min_y && cut_contours[i][j].x < max_x && cut_contours[i][j].x > minx_x) {
            object_points.push_back(cut_contours[i][j]);
          }
        }

        cout << "Cut contour chosen points: " << object_points.size() << endl;
        double left_point_total_x = 0;
        double right_point_total_x = 0;
        double right_counter = 0;
        double left_counter = 0;
        double totalMiddleX = 0;
        for (int i = 0; i < object_points.size(); i++) {
          totalMiddleX += object_points[i].x;
        }
        double leftAndRightMiddleX = totalMiddleX / object_points.size();
        for (int i = 0; i < object_points.size(); i++) {
          if (object_points[i].x > leftAndRightMiddleX) {
            right_point_total_x += object_points[i].x;
            right_counter++;
          }
          else {
            left_point_total_x += object_points[i].x;
            left_counter++;
          }
        }
        left_point_x = left_point_total_x / left_counter;
        right_point_x = right_point_total_x / right_counter;
      }
    }

    if (right_point_x - left_point_x > 50) {
      left_corner_.point_.x = (int) left_point_x;
      right_corner_.point_.x = (int) right_point_x;
    }
  }

  void detectGrayscaleSides() {
    object_grayscale_points_.clear();
    for(int j=0;j< src_gray_.rows;j++) {
      for (int i=0;i< src_gray_.cols;i++) {
        if(src_gray_.at<uchar>(j,i) > object_grayscale_for_centering_) {
//          src_gray.at<uchar>(j,i) = 0;
          object_grayscale_points_.push_back(Point(i,j));
        }
      }
    }

    int grayscale_min_x = image_.cols;
    int grayscale_max_x = 0;
    int grayscale_y_at_max_x = 0;
    int grayscale_y_at_min_x = 0;

    if (object_grayscale_points_.size() > min_array_size_to_use_grayscale_detection_) {
      cout << "Using color detection" << endl;

      for (int i = 0; i < object_grayscale_points_.size(); i++) {
        if (object_grayscale_points_[i].x > grayscale_max_x) {
          grayscale_max_x = object_grayscale_points_[i].x;
          grayscale_y_at_max_x = object_grayscale_points_[i].y;
        }
        if (object_grayscale_points_[i].x < grayscale_min_x) {
          grayscale_min_x = object_grayscale_points_[i].x;
          grayscale_y_at_min_x = object_grayscale_points_[i].y;
        }
      }

      if (grayscale_y_at_max_x > grayscale_y_at_min_x) {
        int XatMaxLevel = grayscale_max_x;
        for (int i = 0; i < object_grayscale_points_.size(); i++) {
          if (object_grayscale_points_[i].y == grayscale_y_at_max_x) {
            if (object_grayscale_points_[i].x < XatMaxLevel) {
              XatMaxLevel = object_grayscale_points_[i].x;
            }
          }
        }

        right_corner_.point_.x = grayscale_max_x;
        left_corner_.point_.x = XatMaxLevel;
        right_corner_.point_.y = grayscale_y_at_max_x;
        left_corner_.point_.y = grayscale_y_at_max_x;
      }
      else {
        int XatMinLevel = grayscale_min_x;
        for (int i = 0; i < object_grayscale_points_.size(); i++) {
          if (object_grayscale_points_[i].y == grayscale_y_at_min_x) {
            if (object_grayscale_points_[i].x > XatMinLevel) {
              XatMinLevel = object_grayscale_points_[i].x;
            }
          }
        }

        right_corner_.point_.x = XatMinLevel;
        left_corner_.point_.x = grayscale_min_x;
        right_corner_.point_.y = grayscale_y_at_min_x;
        left_corner_.point_.y = grayscale_y_at_min_x;
      }

      cout << "Grayscale min X: " << grayscale_min_x << endl;
      cout << "Grayscale max X: " << grayscale_max_x << endl;

      corner_distance_ = right_corner_.point_.x - left_corner_.point_.x;
      object_location_pixels_x_ = left_corner_.point_.x + corner_distance_ /2 - image_middle_x_;
    }
    else {
      cout << "Using contour detection" << endl;
    }
  }

  void detectIfHaveToGrab() {
    double white = 0;
    double non_white = 0;
    double total_grayscale = 0;
    for(int j= look_for_grayscale_min_x_close_;j< look_for_grayscale_max_x_close_;j++) {
      for (int i= look_for_grayscale_min_y_close_;i< look_for_grayscale_max_y_close_;i++) {
        if(src_gray_.at<uchar>(i,j) > object_grayscale_for_grabbing_close_) {
          white++;
        }
        else {
          non_white++;
        }
        total_grayscale += src_gray_.at<uchar>(i,j);
      }
    }

    if (white > 400) {
      grab_object_grayscale_close_ = true;
    }

    double averageGrayscale = total_grayscale / (white + non_white);
    cout << "CLOSE GRAB: " << grab_object_grayscale_close_ << endl;
    cout << "White: " << white << endl;
    cout << "Average grayscale: " << averageGrayscale << endl;

    if (grab_object_grayscale_close_ || object_grayscale_points_.size() < grab_below_grayscale_point_count_) {
      grab_object_ = true;
    }
  }

  void detectContoursAndDraw(vector<vector<Point> > &contours, int &contour_min_y,
                             Size treshold_output_size) {
    drawing_ = Mat::zeros( treshold_output_size, CV_8UC3 );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> bound_rect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) {
      if (contours[i].size() > detect_contours_with_size_min_ && contours[i].size() < detect_contours_with_size_max_) {
        double total_y = 0;
        for (int j = 0; j < contours[i].size(); j++) {
          total_y += contours[i][j].y;
        }

        double average_y = total_y / contours[i].size();

        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true );
        bound_rect[i] = boundingRect(Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );

        if (detect_contours_between_min_y_ < average_y && average_y < detect_contours_between_max_y_) {
          for (int j = 0; j < contours[i].size(); j++) {
            if (contours[i][j].y < contour_min_y) {
              contour_min_y = contours[i][j].y;
            }
          }
          RNG rng(12345);
          Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          drawContours(drawing_, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          circle(drawing_, center[i], (int)radius[i], color, 2, 8, 0 );
        }
      }
    }
  }

  void detectCloseContours(vector<vector<Point> > &contours, Size treshold_output_size) {
    cut_drawing_ = Mat::zeros( treshold_output_size, CV_8UC3 );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) {
      if (contours[i].size() > detect_close_contours_with_size_min_) {

        double totalY = 0;
        for (int j = 0; j < contours[i].size(); j++) {
          totalY += contours[i][j].y;
        }

        double averageY = totalY / contours[i].size();

        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect(Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );

        RNG rng(12345);
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(cut_drawing_, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        circle(cut_drawing_, center[i], (int)radius[i], color, 2, 8, 0 );

      }
    }
  }

  double calculateDeltaX() {
    double angle_to_object = abs(object_location_pixels_x_ / image_middle_x_) * (horizontal_angle_ /2);
    double delta_x = manipulator_to_object_distance_ * tan(angle_to_object * (atan(1)*4) / 180);
    if (object_location_pixels_x_ < 0) {
      delta_x *= -1;
    }

    return delta_x;
  }

  void findCornersUsingContourDetection(int contourMinY, vector<Point> corners) {
    vector<Point> left_and_right_corner;

    //using lowest y value corner to detect right and left corner
    int max_y = 0;
    int x_at_max_y = 0;
    for (int i = 0; i < corners.size(); i++) {
      cout << "Corner: X: " << corners[i].x << " Y: " << corners[i].y << endl;
      if (corners[i].y > max_y) {
        x_at_max_y = corners[i].x;
        max_y = corners[i].y;
      }
    }
    cout << "MaxY: " << max_y << endl;
    cout << "X at max Y: " << x_at_max_y << endl;
    for (int i = 0; i < corners.size(); i++) {
      if (corners[i].y == max_y && corners[i].x == x_at_max_y) {
        left_and_right_corner.push_back(corners[i]);
        corners.erase(corners.begin()+i);
      }
    }


    for (int i = 0; i < corners.size(); i++) {
      if (corners[i].x <= x_at_max_y + 100 && corners[i].x >= x_at_max_y - 100 && corners[i].y <= max_y + 50 && corners[i].y >= max_y - 50) {
        left_and_right_corner.push_back(corners[i]);
      }
    }
//  CUsing contours to detect correct right and left corner
//    for (int i = 0; i < corners.size(); i++) {
//      if (corners[i].y < contourMinY + 10 && corners[i].y > contourMinY - 20) {
//        left_and_right_corner.push_back(corners[i]);
//      }
//    }
    if (left_and_right_corner.size() == 2) {
      if (left_and_right_corner[0].x < left_and_right_corner[1].x) {
        left_corner_.point_ = left_and_right_corner[0];
        right_corner_.point_ = left_and_right_corner[1];
      }
      else {
        left_corner_.point_ = left_and_right_corner[1];
        right_corner_.point_ = left_and_right_corner[0];
      }
      use_contour_detection_to_find_corners_ = false;
    }
    else {
//      cout << "Cannot find two corners using contour detection." << endl;
    }
  }

  void keepTrackingCornersAndFindObjectLocationPixels(vector<Point> &corners) {
    left_corner_.trackCorner(corners, right_corner_);
    right_corner_.trackCorner(corners, left_corner_);

    object_location_pixels_x_ = left_corner_.point_.x + corner_distance_ /2 - image_middle_x_;
    cout << "Delta X by pixels " << object_location_pixels_x_ << endl;
  }

  void objectDistanceCallback(const cluster_extraction::objectDistance msg) {
    manipulator_to_object_distance_ = msg.distance;
    sync_counter_ = msg.counter + 1;
    is_manipulator_close_ = msg.manipulatorIsClose;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulator_adjuster");

  ManipulatorAdjuster manipulator_adjuster;

  ros::Duration d(0.1);
  while (ros::ok()) {

    ros::spinOnce();
    d.sleep();
  }

  return 0;
}

