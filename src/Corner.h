//
// Created by jyrx on 6.05.15.
//

#ifndef CONTOUR_DETECTION_CORNER_H
#define CONTOUR_DETECTION_CORNER_H

#include <opencv2/core/core.hpp>

class Corner {
public:
  cv::Point point_;
  int counter_;

  Corner() {
    counter_ = 0;
  }

  void trackCorner(std::vector<cv::Point> &corners, Corner otherCorner) {
    int delta = 30;
    bool foundCorner = false;
    for (int i = 0; i < corners.size(); i++) {
      if (corners[i].x < point_.x + delta && corners[i].x > point_.x - delta && corners[i].y < point_.y + delta && corners[i].y > point_.y - delta) {
        if (corners[i].x != otherCorner.point_.x && corners[i].y != otherCorner.point_.y) {
          point_ = corners[i];
          foundCorner = true;
          break;
        }
      }
    }
    if (foundCorner == true) {
      counter_ = 0;
    }
    else {
      counter_++;
    }
  }
};

#endif //CONTOUR_DETECTION_CORNER_H
