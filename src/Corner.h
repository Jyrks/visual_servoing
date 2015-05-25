//
// Created by jyrx on 6.05.15.
//

#ifndef CONTOUR_DETECTION_CORNER_H
#define CONTOUR_DETECTION_CORNER_H

#include <opencv2/core/core.hpp>

class Corner {
public:
  cv::Point point;
  int counter;
  int side;

  Corner() {
    counter = 0;
  }

  void trackCorner(std::vector<cv::Point> &corners) {
    int delta = 30;
    bool foundCorner = false;
    for (int i = 0; i < corners.size(); i++) {
      if (corners[i].x < point.x + delta && corners[i].x > point.x - delta && corners[i].y < point.y + delta && corners[i].y > point.y - delta) {
        point = corners[i];
        foundCorner = true;
        break;
      }
    }
    if (foundCorner == true) {
      counter = 0;
    }
    else {
      counter++;
    }
  }
};

enum {
  LEFT = 0,
  RIGHT = 1
};


#endif //CONTOUR_DETECTION_CORNER_H
