/**
 * @file Utils/include/VisionUtils.h
 *
 * This file declares the vision utility functions
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace VisionUtils
{
  /**
   * @brief VisionUtils::namedWindow A wrapper function for cv::namedWindow()
   */
  template <typename... Args>
  auto createWindow(Args&&... args) -> decltype(cv::namedWindow(std::forward<Args>(args)...));

  /**
   * @brief addTrackBar A wrapper function for cv:::createTrackbar()
   */
  template <typename... Args>
  auto addTrackbar(Args&&... args) -> decltype(cv::createTrackbar(std::forward<Args>(args)...));

  /**
   * @brief inRange A wrapper function for cv:::inRange()
   */
  template <typename... Args>
  auto applyThreshold(Args&&... args) -> decltype(cv::inRange(std::forward<Args>(args)...));

  /**
   * @brief displayImage A wrapper function for cv::imshow()
   * @param name Name of display window
   * @param image Image to display
   * @param size_ratio Display size ratio
   */
  void displayImage(
    const std::string& name, const cv::Mat& image, const float& size_ratio = 0.75f);

  /**
   * @brief drawRotatedRect Draws a rotated rect on the given input image matrix
   * @param image Input image matrix
   * @param rect Rotated rectangle
   * @param color Color of the rectangle
   * @return void
   */
  void drawRotatedRect(
    cv::Mat& image, const cv::RotatedRect& rect, const cv::Scalar& color);

  /**
   * @brief Draws a point with given color on the given image matrix
   * @param point Point to be drawn
   * @param image Input image matrix
   * @param color Color
   * @return void
   */
  template<typename T>
  void drawPoint(
    const cv::Point_<T>& point,
    cv::Mat& image,
    const cv::Scalar& color = cv::Scalar(0, 0, 0));

  /**
   * @brief Draws points on the image matrix with three channels
   * @param points Vector of points
   * @param image: Input image matrix
   * @return void
   */
  template<typename T>
  void drawPoints(
    const std::vector<cv::Point_<T> >& points,
    cv::Mat& image,
    const cv::Scalar& color = cv::Scalar(0, 0, 0));

  /**
   * @brief minContourDist Returns the minimum euclidean distance between
   *   two contours
   * @param c1 First input contour
   * @param c2 Second input contour
   * @return Minimum distance
   */
  template<typename T>
  auto minContourDist(
    const std::vector<cv::Point_<T> >& c1, const std::vector<cv::Point_<T> >& c2) -> T;

  /**
   * @brief cvMatToString Encodes the opencv matrix in hex string format
   * @param image Input image
   * @return Encoded string
   */
  auto cvMatToString(const cv::Mat& image) -> std::string;

  /**
   * @brief computeSlope Returns the lines slope
   * @param l Line
   * @return Slope
   */
  template<typename T>
  auto computeSlope(const cv::Vec<T, 4>& l) -> T;

  /**
   * @brief computeSlope Returns the lines angle
   * @param l Line
   * @return Angle
   */
  template<typename T>
  auto computeAngle(const cv::Vec<T, 4>& l) -> T;

  /**
   * @brief Checks if the two lines are approximately parallel
   *   by comparing their slopes with a given threshold
   * @param l1 First line
   * @param l2 Second line
   * @return True if parallel
   */
  template<typename T>
  auto checkParallel(
    const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2, const T& threshold) -> bool;

  /**
   * @brief Checks if the two lines are collinear using triangle area property
   *   or by using slope with y-intercept.
   * @param l1 First line
   * @param l2 Second line
   * @param threshold Threshold for slopes and yItercept or area
   * @return bool
   */
  template<typename T>
  auto checkCollinearity(
    const cv::Vec<T, 4>& l1,
    const cv::Vec<T, 4>& l2,
    const T& threshold_theta,
    const T& threshold_dist) -> bool;

  template<typename T>
  auto checkCollinearity(
    const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2) -> bool;

  /**
   * @brief Given three collinear points p, q, r, the function checks if
   * point q lies on line segment 'pr'
   * @param p First point
   * @param q Second point
   * @param r Third point
   * @return bool
   */
  template<typename T>
  auto onSegment(
    const cv::Point_<T>& p, const cv::Point_<T>& q, const cv::Point_<T>& r) -> bool;

  /**
   * @brief clip Clips a number between minimum and maximum value limit
   * @param n Number
   * @param lower Lower limit
   * @param upper Upper limit
   * @return Clipped number
   */
  template <typename T>
  auto clip(const T& n, const T& lower, const T& upper) -> T;

  /**
   * @brief To find orientation of ordered triplet (p, q, r).
   * The function returns following values:
   * 0 --> p, q and r are colinear
   * -1 --> Clockwise
   * 1 --> Counterclockwise
   * @param p First point
   * @param q Second point
   * @param r Third point
   * @return unsigned
   */
  template<typename T>
  auto orientation(const cv::Point_<T>& p, const cv::Point_<T>& q, const cv::Point_<T>& r) -> int;

  /**
   * @brief Return true if two lines intersect
   * @param l1 First line
   * @param l2 Second line
   * @return bool
   */
  template<typename T>
  auto doIntersect(const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2) -> bool;

  /**
   * @brief Finds the point of intersection between two lines
   * @param l1 First line
   * @param l2 Second line
   * @return Point of intersection
   */
  template<typename T>
  bool findIntersection(const cv::Vec<T, 4>& l1, const cv::Vec<T, 4>& l2, cv::Point_<T>& p);

  /**
   * @brief This function checks whether a point lies within a polygon.
   * @param polygon The polygon defined by several points.
   * @param p Point with x and y to be checked.
   * @return bool
   */
  /*template<typename T>
   bool isInside(const vector<Point_<T> >& polygon, const Point_<T>& p)
   {
   //! There must be at least 3 vertices in polygon
   if (polygon.size() < 3)  return false;
   //! Create a point for line segment from p to infinite
   auto extreme = Point_<T>(INF, p.y);
   //! Count intersections of the above line with sides of polygon
   auto count = 0, i = 0;
   do
   {
   auto next = (i+1)%n;
   //! Check if the line segment from 'p' to 'extreme' intersects
   //! with the line segment from 'polygon[i]' to 'polygon[next]'
   if (doIntersect(polygon[i], polygon[next], p, extreme))
   {
   //! If the point 'p' is colinear with line segment 'i-next',
   //! then check if it lies on segment. If it lies, return true,
   //! otherwise false
   if (orientation(polygon[i], p, polygon[next]) == 0)
   return onSegment(polygon[i], p, polygon[next]);
   count++;
   }
   i = next;
   } while (i != 0);
   //! Return true if count is odd, false otherwise
   return count&1;
   }*/

  /**
   * @brief This function finds all the connected components in an image
   *   and updates the labelled image.
   * @param image: Input image matrix.
   * @param labels: Labelled image matrix.
   * @return Number of labels
   */
  /*unsigned connectedComponents(const Mat1i& image, Mat1i& labels)
  {
    vector < vector<Point> > components;
    return connectedComponents(image, labels, components);
  }*/

  /**
   * @brief This function finds all the connected components in an image
   *   and updates the labelled image.
   * @param image: Input image matrix.
   * @param labels: Labelled image matrix.
   * @param components: Separate component contours.
   * @return Number of labels
   */
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
            //! 4 connected
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

            //! 8 connected
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
} //! VisionUtils
