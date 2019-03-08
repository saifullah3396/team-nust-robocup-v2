#include <iostream>
#include <Eigen/Dense>
#include "../include/PotentialField2D.h"
#include "../include/VisionUtils.h"

using namespace Utils;

int main()
{
  PotentialField2D<float> pf;
  RobotPose2D<float> r, g;
  r.x() = 0.0;
  r.y() = 0.0;
  r.theta() = 0.0;
  
  g.x() = 2.0;
  g.y() = 0.0;
  g.theta() = M_PI / 2;
  int i = 0;
  Mat img = Mat(Size(1000, 700), CV_8UC1, cv::Scalar(0));
  double prevCost = 1e9;
  while (true) {
    auto update = pf.update(r, g, vector<Obstacle>());
    //cout << "update: " << update.mat.transpose() << endl;
    r.mat += update.mat * 0.05;
    //cout << "r: " << r.mat.transpose() << endl;
    double cost = (r.mat-g.mat).segment(0, 2).norm();
    if (cost < 1e-2)
      break;
    if (cost < prevCost) { 
      cout << "decreasing..." << endl;
    } else {
      cout << "increasing..." << endl;
    }
    cout << "cost: " << cost << endl;
    VisionUtils::drawPoint(Point2f(r.mat[0]* 100 + 500, 350 - 100 * r.mat[1]), img);
    VisionUtils::displayImage(img, "result");
    waitKey(0);
    prevCost = cost;
    cout << "i: " << i++ << endl;
  }
  VisionUtils::displayImage(img, "result");
  waitKey(0);
}
