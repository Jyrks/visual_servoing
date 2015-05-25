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

class ContourDetector {
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher delta_x_pub_;
  ros::Subscriber object_distance_sub_;
  Mat src_gray;
  Mat image;
  Mat cutImage;
  Mat cutImageGray;
  bool useContourDetectionToFindCorners;
  Corner leftCorner;
  Corner rightCorner;
  int cornersDistance;
  double manipulatorToObjectDistance;
  static const double horizontalAngle = 25.9;
  int syncCounter;
  bool grabObject;
  bool addObjectLastFrame;
  vector<Corner> recoveryArray;
  vector<Corner> closeArray;
  int imageMiddleX;
  Mat drawing;
  Mat cutDrawing;
  double objectLocationPixelsX;
  bool manipulatorIsClose;
  vector<Point> objectTopArray;
  int lookForGrayscaleMaxXClose;
  int lookForGrayscaleMinXClose;
  int lookForGrayscaleMaxYClose;
  int lookForGrayscaleMinYClose;
  int objectGrayscaleForCentering;
  int objectGrayscaleForGrabbingClose;
  int objectGrayscaleForGrabbingFar;
  bool grayscaleClose;
  bool grayscaleFar;
  bool grayscaleFarAway;
  int lookForGrayscaleMinXFar;
  int lookForGrayscaleMaxXFar;
  int lookForGrayscaleMinYFar;
  int lookForGrayscaleMaxYFar;
  int lookForGrayscaleMinXFarAway;
  int lookForGrayscaleMaxXFarAway;
  int lookForGrayscaleMinYFarAway;
  int lookForGrayscaleMaxYFarAway;

public:
  ContourDetector()
      : it_(nh_)
  {
    image_sub_ = it_.subscribe("/cam2/image_raw", 1, &ContourDetector::imageCallback, this);
    delta_x_pub_ = nh_.advertise<cluster_extraction::manipulatorAdjustment>("/adjust_manipulator_x", 1);
    object_distance_sub_ = nh_.subscribe("/manipulator_to_object_distance", 1, &ContourDetector::objectDistanceCallback, this);
    useContourDetectionToFindCorners = true;
    manipulatorToObjectDistance = 0;
    syncCounter = 0;
    manipulatorIsClose = false;
    grabObject = false;
    lookForGrayscaleMinXClose = 520;
    lookForGrayscaleMaxXClose = 630;
    lookForGrayscaleMinYClose = 830;
    lookForGrayscaleMaxYClose = 860;

    lookForGrayscaleMinXFar = 520;
    lookForGrayscaleMaxXFar = 630;
    lookForGrayscaleMinYFar = 580;
    lookForGrayscaleMaxYFar = 610;

    lookForGrayscaleMinXFarAway = 320;
    lookForGrayscaleMaxXFarAway = 830;
    lookForGrayscaleMinYFarAway = 80;
    lookForGrayscaleMaxYFarAway = 110;

    nh_.param("objectGrayscaleForCentering", objectGrayscaleForCentering, 215);
    nh_.param("objectGrayscaleForGrabbingClose", objectGrayscaleForGrabbingClose, 160); //140 töötaks ka
    nh_.param("objectGrayscaleForGrabbingClose", objectGrayscaleForGrabbingFar, 170);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::Mat imageIn(cv_bridge::toCvCopy(msg, "bgr8")->image);
      image = imageIn;
      cutImage = imageIn;
      cutImage.resize(700);
      cvtColor( cutImage, cutImageGray, CV_BGR2GRAY );
      blur( cutImageGray, cutImageGray, Size(3,3) );
      cvtColor( image, src_gray, CV_BGR2GRAY );
      blur( src_gray, src_gray, Size(3,3) );
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
    vector<Vec4i> cutHierarchy;
    int imageMiddleDeltaX = 0;
    int thresh = 100;
    int max_thresh = 255;
    int contourMinY = 960;
    imageMiddleX = image.cols/2 - imageMiddleDeltaX;

    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    threshold( cutImageGray, threshold_cutOutput, thresh, 255, THRESH_BINARY );
    findContours( threshold_cutOutput, cutContours, cutHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    detectContoursAndDraw(contours, contourMinY, threshold_output.size());

    detectCutContoursAndDraw(cutContours, threshold_cutOutput.size());

    vector<Point> corners;
    goodFeaturesToTrack(src_gray, corners, 6, 0.1, 50);

    if (leftCorner.counter > 20 && rightCorner.counter > 20) {
      useContourDetectionToFindCorners = true;
    }

    if (useContourDetectionToFindCorners) {
      findCornersUsingContourDetection(contourMinY, corners);
    }

    if (!useContourDetectionToFindCorners) {
      keepTrackingCornersAndFindObjectLocationPixels(corners);
    }

//    recoverCornersFromPreviousFrame(corners);

//    addCornersFromPreviousFrame();

    cout << "Manipulator is close: " << manipulatorIsClose << endl;
    if (manipulatorIsClose) {
      manipulatorToObjectDistance = 0.05;
      calculateCloseManipulatorCorners(cutContours);
      detectGrayscaleSides();
      detectIfHaveToGrab();
    }
    else {
      grayscaleFarAway = false;
      grayscaleClose = false;
      grayscaleFar = false;
      grabObject = false;
      detectGrayscaleSides();
    }

    cout << "Corners distance " << cornersDistance << endl;
    double deltaX = calculateDeltaX();

    cluster_extraction::manipulatorAdjustment adjustMessage;
    adjustMessage.counter = syncCounter;
    adjustMessage.deltax = deltaX;
    adjustMessage.grab = grabObject;
    delta_x_pub_.publish(adjustMessage);

    Mat tmp, dst;
    tmp = drawing;
    dst = tmp;

    Mat tmp2, dst2;
    tmp2 = cutDrawing;
    dst2 = tmp2;

    Mat tmp3, dst3;
    tmp3 = src_gray;

    circle(tmp3, leftCorner.point, 20, Scalar(200, 0, 0), 4, 16, 0);
    circle(tmp3, rightCorner.point, 20, Scalar(0, 200, 0), 4, 16, 0);

    rectangle(tmp3, Point(lookForGrayscaleMaxXClose, lookForGrayscaleMaxYClose), Point(lookForGrayscaleMinXClose,
                                                                                       lookForGrayscaleMinYClose), Scalar(200, 0, 0), 2, 8, 0);

//    rectangle(tmp3, Point(lookForGrayscaleMaxXFar, lookForGrayscaleMaxYFar), Point(lookForGrayscaleMinXFar,
//                                                                                   lookForGrayscaleMinYFar), Scalar(200, 0, 0), 2, 8, 0);

//    rectangle(tmp3, Point(lookForGrayscaleMaxXFarAway, lookForGrayscaleMaxYFarAway), Point(lookForGrayscaleMinXFarAway,
//                                                                                           lookForGrayscaleMinYFarAway), Scalar(200, 0, 0), 2, 8, 0);

    for (int i = 0; i < corners.size(); i++) {
//      circle(tmp3, corners[i], 10, Scalar(200,200,200), 2, 8, 0);
    }

    pyrDown( tmp, dst, Size( tmp.cols/2, tmp.rows/2));
//    pyrDown( tmp2, dst2, Size( tmp.cols/2, tmp.rows/2));
    pyrDown( tmp3, dst3, Size( tmp.cols/2, tmp.rows/2));

//    imshow("Contours", dst);
//    imshow("Cut image", tmp2);
    imshow("Corners", dst3);

    waitKey(3);
  }

  void calculateCloseManipulatorCorners(vector<vector<Point> > &cutContours) {
    double leftPointValueX = 0;
    double rightPointValueX = 0;
    double maxY = cutImage.rows - 150;
    double minY = cutImage.rows - 250;
    double maxX = imageMiddleX + 300;
    double minX = imageMiddleX - 350;
    vector<Point> objectPoints;

    for (int i = 0; i < cutContours.size(); i++) {
      if (cutContours[i].size() > 250) {
        for (int j = 0; j < cutContours[i].size(); j++) {
          if (cutContours[i][j].y < maxY && cutContours[i][j].y > minY && cutContours[i][j].x < maxX && cutContours[i][j].x > minX) {
            objectPoints.push_back(cutContours[i][j]);
          }
        }

        cout << "Cut contour chosen points: " << objectPoints.size() << endl;
        double leftPointTotalX = 0;
        double rightPointTotalX = 0;
        double rightPointCounter = 0;
        double leftPointCounter = 0;
        double totalMiddleX = 0;
        for (int i = 0; i < objectPoints.size(); i++) {
          totalMiddleX += objectPoints[i].x;
        }
        double leftAndRightMiddleX = totalMiddleX / objectPoints.size();
        for (int i = 0; i < objectPoints.size(); i++) {
          if (objectPoints[i].x > leftAndRightMiddleX) {
            rightPointTotalX += objectPoints[i].x;
            rightPointCounter++;
//                cout << "Right point: " << objectPoints[i].x << endl;
          }
          else {
            leftPointTotalX += objectPoints[i].x;
            leftPointCounter++;
//                cout << "Left point: " << objectPoints[i].x << endl;
          }
        }
        leftPointValueX = leftPointTotalX / leftPointCounter;
        rightPointValueX = rightPointTotalX / rightPointCounter;
      }
    }

    if (rightPointValueX - leftPointValueX > 50) {
      closeArray.clear();
      leftCorner.point.x = (int) leftPointValueX;
      rightCorner.point.x = (int) rightPointValueX;
      closeArray.push_back(leftCorner);
      closeArray.push_back(rightCorner);
    }
    else if (!closeArray.empty()){
      leftCorner = closeArray[0];
      rightCorner = closeArray[1];
    }
  }

  void detectGrayscaleSides() {
    objectTopArray.clear();
    for(int j=0;j< src_gray.rows;j++) {
      for (int i=0;i< src_gray.cols;i++) {
        if(src_gray.at<uchar>(j,i) > objectGrayscaleForCentering) {
//          src_gray.at<uchar>(j,i) = 0;
          objectTopArray.push_back(Point(i,j));
        }
      }
    }

    int grayScaleMinX = 1280;
    int grayScaleMaxX = 0;
    int grayScaleYatMaxX = 0;
    int grayScaleYatMinX = 0;

    if (objectTopArray.size() > 500) {
      cout << "Using color detection" << endl;

      for (int i = 0; i < objectTopArray.size(); i++) {
        if (objectTopArray[i].x > grayScaleMaxX) {
          grayScaleMaxX = objectTopArray[i].x;
          grayScaleYatMaxX = objectTopArray[i].y;
        }
        if (objectTopArray[i].x < grayScaleMinX) {
          grayScaleMinX = objectTopArray[i].x;
          grayScaleYatMinX = objectTopArray[i].y;
        }
      }

//    leftCorner.point.x = grayScaleMinX;
//    rightCorner.point.x = grayScaleMaxX;

      if (grayScaleYatMaxX > grayScaleYatMinX) {
        int XatMaxLevel = grayScaleMaxX;
        for (int i = 0; i < objectTopArray.size(); i++) {
          if (objectTopArray[i].y == grayScaleYatMaxX) {
            if (objectTopArray[i].x < XatMaxLevel) {
              XatMaxLevel = objectTopArray[i].x;
            }
          }
        }

        rightCorner.point.x = grayScaleMaxX;
        leftCorner.point.x = XatMaxLevel;
        rightCorner.point.y = grayScaleYatMaxX;
        leftCorner.point.y = grayScaleYatMaxX;
      }
      else {
        int XatMinLevel = grayScaleMinX;
        for (int i = 0; i < objectTopArray.size(); i++) {
          if (objectTopArray[i].y == grayScaleYatMinX) {
            if (objectTopArray[i].x > XatMinLevel) {
              XatMinLevel = objectTopArray[i].x;
            }
          }
        }

        rightCorner.point.x = XatMinLevel;
        leftCorner.point.x = grayScaleMinX;
        rightCorner.point.y = grayScaleYatMinX;
        leftCorner.point.y = grayScaleYatMinX;
      }

      cout << "Grayscale min X: " << grayScaleMinX << endl;
      cout << "Grayscale max X: " << grayScaleMaxX << endl;

      cornersDistance = rightCorner.point.x - leftCorner.point.x;
      objectLocationPixelsX = leftCorner.point.x + cornersDistance /2 - imageMiddleX;
    }
    else {
      cout << "Using contour detection" << endl;
    }
  }

  void detectIfHaveToGrab() {
    double white = 0;
    double nonWhite = 0;
    double totalGrayscale = 0;
    for(int j= lookForGrayscaleMinXClose;j< lookForGrayscaleMaxXClose;j++) {
      for (int i= lookForGrayscaleMinYClose;i< lookForGrayscaleMaxYClose;i++) {
        if(src_gray.at<uchar>(i,j) > objectGrayscaleForGrabbingClose) {
          white++;
        }
        else {
          nonWhite++;
        }
        totalGrayscale += src_gray.at<uchar>(i,j);
      }
    }

    if (white > 400) {
      grayscaleClose = true;
    }

    double averageGrayscale = totalGrayscale / (white + nonWhite);
    cout << "CLOSE GRAB: " << grayscaleClose << endl;
    cout << "White: " << white << endl;
    cout << "Average grayscale: " << averageGrayscale << endl;

    white = 0;
    nonWhite = 0;
    totalGrayscale = 0;
    for(int j= lookForGrayscaleMinXFar;j< lookForGrayscaleMaxXFar;j++) {
      for (int i= lookForGrayscaleMinYFar;i< lookForGrayscaleMaxYFar;i++) {
        if(src_gray.at<uchar>(i,j) > objectGrayscaleForGrabbingFar) {
          white++;
        }
        else {
          nonWhite++;
        }
        totalGrayscale += src_gray.at<uchar>(i,j);
      }
    }

    if (white > 400) {
      grayscaleFar = true;
    }

    averageGrayscale = totalGrayscale / (white + nonWhite);
    cout << "FAR GRAB: " << grayscaleFar << endl;
    cout << "White: " << white << endl;
    cout << "Average grayscale: " << averageGrayscale << endl;

    white = 0;
    nonWhite = 0;
    totalGrayscale = 0;
    for(int j= lookForGrayscaleMinXFarAway;j< lookForGrayscaleMaxXFarAway;j++) {
      for (int i= lookForGrayscaleMinYFarAway;i< lookForGrayscaleMaxYFarAway;i++) {
        if(src_gray.at<uchar>(i,j) > objectGrayscaleForGrabbingFar) {
          white++;
        }
        else {
          nonWhite++;
        }
        totalGrayscale += src_gray.at<uchar>(i,j);
      }
    }

    if (white < 3000) {
      grayscaleFarAway = true;
    }

//    averageGrayscale = totalGrayscale / (white + nonWhite);
//    cout << "White: " << white << endl;
//    cout << "Far Away Average grayscale: " << averageGrayscale << endl;
//    cout << "FAR AWAY GRAB: " << grayscaleFar << endl;

    if (grayscaleClose || cornersDistance < 50) {
      grabObject = true;
    }
  }

  void detectContoursAndDraw(vector<vector<Point> > &contours, int &contourMinY,
                             Size treshold_output_size) {
    drawing = Mat::zeros( treshold_output_size, CV_8UC3 );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) {
      if (contours[i].size() > 150 && contours[i].size() < 400) {
        double totalY = 0;
        for (int j = 0; j < contours[i].size(); j++) {
          totalY += contours[i][j].y;
        }

        double averageY = totalY / contours[i].size();

        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect(Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );

        if (image.rows*0.4 < averageY && averageY < image.rows*0.8) {
          for (int j = 0; j < contours[i].size(); j++) {
            if (contours[i][j].y < contourMinY) {
              contourMinY = contours[i][j].y;
            }
          }
        }
      }
      RNG rng(12345);
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
    }
  }

  void detectCutContoursAndDraw(vector<vector<Point> > &contours, Size treshold_output_size) {
    cutDrawing = Mat::zeros( treshold_output_size, CV_8UC3 );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) {
      //      if (contours[i].size() > 150 && contours[i].size() < 1000) {
      if (contours[i].size() > 250) {

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
        drawContours( cutDrawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        circle( cutDrawing, center[i], (int)radius[i], color, 2, 8, 0 );

      }
    }
  }

  double calculateDeltaX() {
    double angleToObject = abs(objectLocationPixelsX / imageMiddleX) * (horizontalAngle/2);
    double deltaX = manipulatorToObjectDistance * tan(angleToObject * (atan(1)*4) / 180);
    if (objectLocationPixelsX < 0) {
      deltaX *= -1;
    }

    return deltaX;
  }

  void findCornersUsingContourDetection(int contourMinY, vector<Point> corners) {
    vector<Point> leftAndRightCorner;

//    int maxY = 0;
//    int XatMaxY = 0;
//    for (int i = 0; i < corners.size(); i++) {
//      cout << "Corner: X: " << corners[i].x << " Y: " << corners[i].y << endl;
//      if (corners[i].y > maxY) {
//        XatMaxY = corners[i].x;
//        maxY = corners[i].y;
//      }
//    }
//    cout << "MaxY: " << maxY << endl;
//    cout << "XatMaxY " << XatMaxY << endl;
//    for (int i = 0; i < corners.size(); i++) {
//      if (corners[i].y == maxY && corners[i].x == XatMaxY) {
//        leftAndRightCorner.push_back(corners[i]);
//        corners.erase(corners.begin()+i);
//      }
//    }
//
//
//    for (int i = 0; i < corners.size(); i++) {
//      if (corners[i].x <= XatMaxY + 100 && corners[i].x >= XatMaxY - 100 && corners[i].y <= maxY + 50 && corners[i].y >= maxY - 50) {
//        leftAndRightCorner.push_back(corners[i]);
//      }
//    }

    for (int i = 0; i < corners.size(); i++) {
      if (corners[i].y < contourMinY + 10 && corners[i].y > contourMinY - 20) {
        leftAndRightCorner.push_back(corners[i]);
      }
    }
    if (leftAndRightCorner.size() == 2) {
      if (leftAndRightCorner[0].x < leftAndRightCorner[1].x) {
        leftCorner.point = leftAndRightCorner[0];
        rightCorner.point = leftAndRightCorner[1];
      }
      else {
        leftCorner.point = leftAndRightCorner[1];
        rightCorner.point = leftAndRightCorner[0];
      }
      useContourDetectionToFindCorners = false;
    }
    else {
//      cout << "Cannot find two corners using contour detection." << endl;
    }
  }

  void keepTrackingCornersAndFindObjectLocationPixels(vector<Point> &corners) {
    leftCorner.trackCorner(corners);
    rightCorner.trackCorner(corners);
//    cout << "Left corner counter: " << leftCorner.counter << endl;
//    cout << "Right corner counter: " << rightCorner.counter << endl;
    int delta = 30;
    if (rightCorner.counter < 5 && leftCorner.counter < 5) {
      cornersDistance = rightCorner.point.x - leftCorner.point.x;
    }
    else if (leftCorner.counter > rightCorner.counter) {
      for (int i = 0; i < corners.size(); i++) {
        if (corners[i].x < rightCorner.point.x - cornersDistance + delta && corners[i].x > rightCorner.point.x -
                                                                                           cornersDistance - delta &&
            corners[i].y < rightCorner.point.y + delta  && corners[i].y > rightCorner.point.y - delta) {

          leftCorner.point = corners[i];
          leftCorner.counter = 0;
        }
      }
    }
    else {
      for (int i = 0; i < corners.size(); i++) {
        if (corners[i].x < leftCorner.point.x + cornersDistance + delta && corners[i].x > leftCorner.point.x +
                                                                                          cornersDistance - delta &&
            corners[i].y < leftCorner.point.y + delta  && corners[i].y > leftCorner.point.y - delta) {

          rightCorner.point = corners[i];
          rightCorner.counter = 0;
        }
      }
    }
    objectLocationPixelsX = leftCorner.point.x + cornersDistance /2 - imageMiddleX;
    cout << "Delta X by pixels " << objectLocationPixelsX << endl;
  }


  void addCornersFromPreviousFrame() {
    if (addObjectLastFrame && leftCorner.counter == 0 && rightCorner.counter == 0) {
      recoveryArray.clear();
      recoveryArray.push_back(leftCorner);
      recoveryArray.push_back(rightCorner);
      addObjectLastFrame = false;
    }
  }

  void recoverCornersFromPreviousFrame(vector<Point> &corners) {
    if (leftCorner.counter > 20 && rightCorner.counter > 20) {
      if (!recoveryArray.empty()) {
        recoveryArray[recoveryArray.size()-2].trackCorner(corners);
        recoveryArray[recoveryArray.size()-1].trackCorner(corners);
      }
    }
  }

  void objectDistanceCallback(const cluster_extraction::objectDistance msg) {
    manipulatorToObjectDistance = msg.distance;
    syncCounter = msg.counter + 1;
    addObjectLastFrame = true;
    manipulatorIsClose = msg.manipulatorIsClose;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "contour_detector");

  ContourDetector contourDetector;

  ros::Duration d(0.1);
  while (ros::ok()) {

    ros::spinOnce();
    d.sleep();
  }

  return 0;
}

