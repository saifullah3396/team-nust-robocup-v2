/**
 * @file Utils/src/VisionUtils.cpp
 *
 * This file defines the vision utility functions
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Eigen/Dense"
#include "Utils/include/Constants.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/VisionUtils.h"

using namespace Eigen;

namespace VisionUtils
{
  template <typename... Args>
  auto createWindow(Args&&... args) ->
    decltype(cv::namedWindow(std::forward<Args>(args)...))
  {
    return cv::namedWindow(std::forward<Args>(args)...);
  }

  template <typename... Args>
  auto addTrackbar(Args&&... args) ->
    decltype(cv::createTrackbar(std::forward<Args>(args)...))
  {
    return cv::createTrackbar(std::forward<Args>(args)...);
  }

  template <typename... Args>
  auto applyThreshold(Args&&... args) ->
    decltype(cv::inRange(std::forward<Args>(args)...))
  {
    return cv::inRange(std::forward<Args>(args)...);
  }

  void displayImage(
    const std::string& name, const cv::Mat& image, const float& size_ratio)
  {
    //cv::namedWindow(name, CV_WINDOW_NORMAL);
    //cv::resizeWindow(name, image.rows * size_ratio, image.cols * size_ratio);
    cv::imshow(name, image);
    cv::waitKey(1);
  }

  void drawRotatedRect(
    cv::Mat& image, const cv::RotatedRect& rect, const cv::Scalar& color)
  {
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (size_t j = 0; j < 4; j++)
      cv::line(image, vertices[j], vertices[(j + 1) % 4], color);
  }

  template<typename T>
  void drawPoint(
    const cv::Point_<T>& point, cv::Mat& image, const cv::Scalar& color)
  {
    cv::circle(image, cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)), 2, color, -1);
  }
  template void drawPoint<int>(
    const cv::Point_<int>&, cv::Mat&, const cv::Scalar&);
  template void drawPoint<float>(
    const cv::Point_<float>&, cv::Mat&, const cv::Scalar&);

  template<typename T>
  void drawPoints(
    const std::vector<cv::Point_<T> >& points, cv::Mat& image, const cv::Scalar& color)
  {
    for (auto& p : points) {
      drawPoint(p, image, color);
    }
  }
  template void drawPoints<int>(
    const std::vector<cv::Point_<int> >&, cv::Mat&, const cv::Scalar&);
  template void drawPoints<float>(
    const std::vector<cv::Point_<float> >&, cv::Mat&, const cv::Scalar&);
  template void drawPoints<double>(
    const std::vector<cv::Point_<double> >&, cv::Mat&, const cv::Scalar&);

  template<typename T>
  auto minContourDist(
    const std::vector<cv::Point_<T> >& c1, const std::vector<cv::Point_<T> >& c2) -> T
  {
    auto min_dist = std::numeric_limits<T>::infinity();
    for (const auto& c1p : c1) {
      for (const auto& c2p : c2) {
        min_dist = std::min(cv::norm(c1p - c2p), min_dist);
      }
    }
    return min_dist;
  }

  auto cvMatToString(const cv::Mat& image) -> std::string
  {
    vector<uchar> bytes_data;
    if (imencode(".jpg", image, bytes_data)) {
      unsigned char* ptr = &bytes_data[0];
      return DataUtils::bytesToHexString(ptr, bytes_data.size());
    } else {
      return std::string("");
    }
  }

  template<typename T>
  auto computeSlope(const cv::Vec<T, 4>& l) -> T
  {
    auto den = (l[2] - l[0]);
    return den != 0 ? (l[3] - l[1]) / den : Constants::infinity;
  }

  template<typename T>
  auto computeAngle(const cv::Vec<T, 4>& l) -> T
  {
    return atan2(l[3] - l[1], l[2] - l[0]);
  }

  template<typename T>
  auto checkParallel(
    const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2, const T& threshold) -> bool
  {
    return fabsf(computeSlope(l1) - computeSlope(l2)) < threshold ? true : false;
  }

  template<typename T>
  auto checkCollinearity(
    const cv::Vec<T, 4>& l1,
    const cv::Vec<T, 4>& l2,
    const T& threshold_theta,
    const T& threshold_dist) -> bool
  {
    auto angle1 = computeAngle(l1);
    auto angle2 = computeAngle(l2);
    if (fabsf(angle1 - angle2) > threshold_theta)
      return false;

    auto angle = atan2(l2[3] - l1[3], l2[2] - l1[2]) - (angle1 + angle2) / 2;
    auto mag = cv::norm(cv::Point_<T>(l2[2], l2[3]) - cv::Point_<T>(l1[2], l1[3]));
    return fabsf(mag * sin(angle)) < threshold_dist ? true : false;
  }

  template<typename T>
  auto checkCollinearity(
    const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2) -> bool
  {
    Matrix<T, 3, 3> area_mat;
    area_mat << l1[0], l1[2], l2[0], l1[1], l1[3], l2[1], 1, 1, 1;
    return 0.5 * fabsf(area_mat.determinant()) < 200 ? true : false;
  }

  template<typename T>
  auto onSegment(
    const cv::Point_<T>& p, const cv::Point_<T>& q, const cv::Point_<T>& r) -> bool
  {
    return q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
           q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y);
  }

  template <typename T>
  auto clip(const T& n, const T& lower, const T& upper) -> T
  {
    return std::max(lower, std::min(n, upper));
  }
  template auto clip<int>(const int& n, const int& lower, const int& upper) -> int;
  template auto clip<float>(const float& n, const float& lower, const float& upper) -> float;
  template auto clip<double>(const double& n, const double& lower, const double& upper) -> double;

  template<typename T>
  auto orientation(const cv::Point_<T>& p, const cv::Point_<T>& q, const cv::Point_<T>& r) -> int
  {
    return clip(int((q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)), -1, 1);
  }

  template<typename T>
  auto doIntersect(const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2) -> bool
    {
      ///< Find the four orientations needed for general and special cases
      auto p1 = cv::Point_<T> (l1[0], l1[1]);
      auto q1 = cv::Point_<T> (l1[2], l1[3]);
      auto p2 = cv::Point_<T> (l2[0], l2[1]);
      auto q2 = cv::Point_<T> (l2[2], l2[3]);
      auto o1 = orientation(p1, q1, p2);
      auto o2 = orientation(p1, q1, q2);
      auto o3 = orientation(p2, q2, p1);
      auto o4 = orientation(p2, q2, q1);
      ///< General case
      if (o1 != o2 && o3 != o4) return true;
      ///< Special Cases
      ///< p1, q1 and p2 are colinear and p2 lies on segment p1q1
      else if (o1 == 0 && onSegment(p1, p2, q1)) return true;
      ///< p1, q1 and p2 are colinear and q2 lies on segment p1q1
      else if (o2 == 0 && onSegment(p1, q2, q1)) return true;
      ///< p2, q2 and p1 are colinear and p1 lies on segment p2q2
      else if (o3 == 0 && onSegment(p2, p1, q2)) return true;
      ///< p2, q2 and q1 are colinear and q1 lies on segment p2q2
      else if (o4 == 0 && onSegment(p2, q1, q2)) return true;
      return false; ///< Doesn't fall in any of the above cases
    }

  template<typename T>
  bool findIntersection(const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2, cv::Point_<T>& p)
  {
    auto p1 = cv::Point_<T> (l1[0], l1[1]);
    auto p2 = cv::Point_<T> (l1[2], l1[3]);
    auto p3 = cv::Point_<T> (l2[0], l2[1]);
    auto p4 = cv::Point_<T> (l2[2], l2[3]);
    auto p1p2 = p1 - p2;
    auto p3p4 = p3 - p4;
    auto det = p1p2.x * p3p4.y - p1p2.y * p3p4.x;
    if (det != 0.0) {
      ///< det == 0 -> Lines are parallel
      auto c1 = p1.x * p2.y - p1.y * p2.x;
      auto c2 = p3.x * p4.y - p3.y * p4.x;
      p.x = (c1 * p3p4.x - p1p2.x * c2) / det;
      p.y = (c1 * p3p4.y - p1p2.y * c2) / det;
      return true;
    }
    return false;
  }
  template bool findIntersection<float>(const cv::Vec<float, 4>& l1, const cv::Vec<float, 4>& l2, cv::Point_<float>& p);

  /*template<typename T>
   bool isInside(const vector<Point_<T> >& polygon, const Point_<T>& p)
   {
   ///< There must be at least 3 vertices in polygon
   if (polygon.size() < 3)  return false;
   ///< Create a point for line segment from p to infinite
   auto extreme = Point_<T>(INF, p.y);
   ///< Count intersections of the above line with sides of polygon
   auto count = 0, i = 0;
   do
   {
   auto next = (i+1)%n;
   ///< Check if the line segment from 'p' to 'extreme' intersects
   ///< with the line segment from 'polygon[i]' to 'polygon[next]'
   if (doIntersect(polygon[i], polygon[next], p, extreme))
   {
   ///< If the point 'p' is colinear with line segment 'i-next',
   ///< then check if it lies on segment. If it lies, return true,
   ///< otherwise false
   if (orientation(polygon[i], p, polygon[next]) == 0)
   return onSegment(polygon[i], p, polygon[next]);
   count++;
   }
   i = next;
   } while (i != 0);
   ///< Return true if count is odd, false otherwise
   return count&1;
   }*/

  /*unsigned connectedComponents(const Mat1i& image, Mat1i& labels)
  {
    vector < vector<Point> > components;
    return connectedComponents(image, labels, components);
  }*/

  /*unsigned
  connectedComponents(const Mat1i& image, Mat1i& labels,
    vector<vector<Point> >& components)
  {
    ASSERT(!image.empty());
    Mat1i src = image;
    labels = Mat1i(image.rows, image.cols, 0);
    int label = 0;
    int w = src.cols;
    int h = src.rows;
    int i;
    Point point;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if ((src(y, x)) > 0) // Seed found
        {
          stack<int, vector<int> > stack2;
          i = x + y * w;
          stack2.push(i);
          vector < Point > comp;
          while (!stack2.empty()) {
            i = stack2.top();
            stack2.pop();
            int x2 = i % w;
            int y2 = i / w;
            src(y2, x2) = 0;
            point.x = x2;
            point.y = y2;
            comp.push_back(point);
            ///< 4 connected
            if (x2 > 0 && (src(y2, x2 - 1) != 0)) {
              stack2.push(i - 1);
              src(y2, x2 - 1) = 0;
            }
            if (y2 > 0 && (src(y2 - 1, x2) != 0)) {
              stack2.push(i - w);
              src(y2 - 1, x2) = 0;
            }
            if (y2 < h - 1 && (src(y2 + 1, x2) != 0)) {
              stack2.push(i + w);
              src(y2 + 1, x2) = 0;
            }
            if (x2 < w - 1 && (src(y2, x2 + 1) != 0)) {
              stack2.push(i + 1);
              src(y2, x2 + 1) = 0;
            }

            ///< 8 connected
            if (x2 > 0 && y2 > 0 && (src(y2 - 1, x2 - 1) != 0)) {
              stack2.push(i - w - 1);
              src(y2 - 1, x2 - 1) = 0;
            }
            if (x2 > 0 && y2 < h - 1 && (src(y2 + 1, x2 - 1) != 0)) {
              stack2.push(i + w - 1);
              src(y2 + 1, x2 - 1) = 0;
            }
            if (x2 < w - 1 && y2 > 0 && (src(y2 - 1, x2 + 1) != 0)) {
              stack2.push(i - w + 1);
              src(y2 - 1, x2 + 1) = 0;
            }
            if (x2 < w - 1 && y2 < h - 1 && (src(y2 + 1, x2 + 1) != 0)) {
              stack2.push(i + w + 1);
              src(y2 + 1, x2 + 1) = 0;
            }
          }
          ++label;
          components.push_back(comp);
          for (int k = 0; k < comp.size(); ++k) {
            labels(comp[k]) = label;
          }
        }
      }
    }
    return label;
  }*/

  /*    static void getEllipseParam(double a,double b,double c,double d,double f,double g, Ellipse& ellipse){
   ellipse.x = (c * d - b * f)/(b * b - a * c);
   ellipse.y = (a * f - b * d)/(b * b - a * c);

   ellipse.majorAxis = sqrt( (2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g))/((b*b-a*c)*(sqrt((a-c)*(a-c)+4*b*b)-(a+c))));
   ellipse.minorAxis = sqrt( (2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g))/((b*b-a*c)*(sqrt((a-c)*(a-c)+4*b*b)+(a+c))));

   ellipse.angle=0;
   if(b == 0 && a < c){
   ellipse.angle = 0;
   }
   else if(b == 0 && a > c){
   ellipse.angle = 90;
   }
   else if(b != 0 && a < c){
   ellipse.angle = 0.5 * MathsUtils::aCotan( (a-c)/(2*b) ) * 180 / M_PI;
   }
   else if(b != 0 && a > c){
   ellipse.angle = 90 + 0.5 * MathsUtils::aCotan( (a-c)/(2*b) ) * 180 / M_PI;
   }
   if(ellipse.minorAxis > ellipse.majorAxis){
   double temp = ellipse.majorAxis;
   ellipse.majorAxis = ellipse.minorAxis;
   ellipse.minorAxis = temp;
   ellipse.angle += 90;
   }

   double temp_c;
   if(ellipse.majorAxis > ellipse.minorAxis)
   temp_c = sqrt(ellipse.majorAxis * ellipse.majorAxis - ellipse.minorAxis * ellipse.minorAxis);
   else
   temp_c = sqrt(ellipse.minorAxis * ellipse.minorAxis - ellipse.majorAxis * ellipse.majorAxis);
   ellipse.f1_x = ellipse.x - temp_c * cos(ellipse.angle*M_PI/180);
   ellipse.f1_y = ellipse.y - temp_c * sin(ellipse.angle*M_PI/180);
   ellipse.f2_x = ellipse.x + temp_c * cos(ellipse.angle*M_PI/180);
   ellipse.f2_y = ellipse.y + temp_c * sin(ellipse.angle*M_PI/180);
   }

   static bool pointInEllipse(Point point,Ellipse ellipse){
   double dist1 = sqrt((point.x - ellipse.f1_x) * (point.x - ellipse.f1_x) +
   (point.y - ellipse.f1_y) * (point.y - ellipse.f1_y));
   double dist2 = sqrt((point.x - ellipse.f2_x) * (point.x - ellipse.f2_x) +
   (point.y - ellipse.f2_y) * (point.y - ellipse.f2_y));
   double max;
   if(ellipse.majorAxis > ellipse.minorAxis)
   max = ellipse.majorAxis;
   else
   max = ellipse.minorAxis;
   if(dist1+dist2 <= 2*max)
   return true;
   else
   return false;
   }

   static Ellipse fitEllipseRANSAC(vector<Point> points,int &count){
   Ellipse ellipse;
   count=0;
   int index[5];
   bool match=false;
   for(int i=0;i<5;i++){
   do {
   match = false;
   index[i]=rand()%points.size();
   for(int j=0;j<i;j++){
   if(index[i] == index[j]){
   match=true;
   }
   }
   }
   while(match);
   }
   double aData[] = {
   points[index[0]].x * points[index[0]].x, 2 * points[index[0]].x * points[index[0]].y, points[index[0]].
   y * points[index[0]].y, 2 * points[index[0]].x, 2 * points[index[0]].y,

   points[index[1]].x * points[index[1]].x, 2 * points[index[1]].x * points[index[1]].y, points[index[1]].
   y * points[index[1]].y, 2 * points[index[1]].x, 2 * points[index[1]].y,

   points[index[2]].x * points[index[2]].x, 2 * points[index[2]].x * points[index[2]].y, points[index[2]].
   y * points[index[2]].y, 2 * points[index[2]].x, 2 * points[index[2]].y,

   points[index[3]].x * points[index[3]].x, 2 * points[index[3]].x * points[index[3]].y, points[index[3]].
   y * points[index[3]].y, 2 * points[index[3]].x, 2 * points[index[3]].y,

   points[index[4]].x * points[index[4]].x, 2 * points[index[4]].x * points[index[4]].y, points[index[4]].
   y * points[index[4]].y, 2 * points[index[4]].x, 2 * points[index[4]].y };
   Mat matA=Mat(5,5,CV_64F,aData);
   Mat *D,*U,*V;
   D=new Mat(5,5,CV_64F);
   U=new Mat(5,5,CV_64F);
   V=new Mat(5,5,CV_64F);

   SVD svd(matA);

   Mat matV(5, 5, CV_64F);
   matV = svd.vt;

   double a,b,c,d,f,g;
   a=matV.at<double>(0,4);
   b=matV.at<double>(1,4);
   c=matV.at<double>(2,4);
   d=matV.at<double>(3,4);
   f=matV.at<double>(4,4);
   g=1;

   getEllipseParam(a,b,c,d,f,g,ellipse);

   vector<Point>::iterator point_iter;
   if(ellipse.majorAxis > 0 && ellipse.minorAxis > 0){
   for(point_iter=points.begin();point_iter!=points.end();point_iter++){
   Point point = *point_iter;
   if(pointInEllipse(point,ellipse)){
   count++;
   }
   }
   }

   return ellipse;
   }*/
} ///< VisionUtils

