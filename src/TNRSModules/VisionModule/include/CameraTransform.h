/**
 * @file VisionModule/CameraTransform.h
 *
 * This file declares the class CameraTransform.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Oct 2017
 */

#pragma once

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "TNRSBase/include/MemoryBase.h"
//#include "VisionModule/include/VisionModule.h"
//#include "VisionModule/include/CameraModule/CameraModule.h"
//#include "Utils/include/DataHolders/Camera.h"
//#include "Utils/include/HardwareIds.h"
//#include "Utils/include/MathsUtils.h"
//#include "Utils/include/VisionUtils.h"

enum class CameraId : unsigned int;
class VisionModule;
class CameraModule;
template <typename T>
class Camera;
typedef boost::shared_ptr<Camera<float> > CameraPtr;
typedef boost::shared_ptr<CameraModule> CameraModulePtr;

/**
 * @class CameraTransform
 * @brief A class for performing image to world transformation
 */
class CameraTransform : public MemoryBase
{
public:

  /**
   * Constructor
   *
   * @param visionModule: Pointer to base vision module
   * @param camIndex: Index of the camera
   */
  CameraTransform(
    VisionModule* visionModule,
    const CameraId& camIndex);

  /**
   * Destructor
   */
  ~CameraTransform()
  {
  }

  void computeCamMatrix();


  /*Point3f compute3DOnPlaneFrom2D(const Point2f& imagePt) 
   {
   Point2f normalizedImagePt;
   normalizedImagePt.x = (imagePt.x - cams->centerOffX) / cams->focalX;
   normalizedImagePt.y = (imagePt.y - cams->centerOffY) / cams->focalY;

   float s = -plane[3] / (plane[0]*normalizedImagePt.x + plane[1]*normalizedImagePt.y + plane[2]);

   Point3f pt;
   pt.x = s*normalizedImagePt.x;
   pt.y = s*normalizedImagePt.y;
   pt.z = s;

   return pt;
   }
   
   Vector4f computePlaneEquation(const vector<Vector4f>& ps) 
   {
   //Vector p0_p1
   Vector3f p0_p1 = (ps[0] - ps[1]).block(0, 0, 3, 1);
   Vector3f p0_p2 = (ps[0] - ps[2]).block(0, 0, 3, 1);

   //Normal vector
   Vector3f n = p0_p1.cross(p0_p2);
   
   Vector4f coeffs;
   coeffs[0] = n[0];
   coeffs[1] = n[1];
   coeffs[2] = n[2];
   coeffs[3] = -(coeffs[0]*ps[0][0] + coeffs[1]*ps[0][1] + coeffs[2]*ps[0][2]);

   float norm =  sqrt(coeffs[0]*coeffs[0] + coeffs[1]*coeffs[1] + coeffs[2]*coeffs[2]);
   coeffs /= norm;
   return coeffs;
   }*/

  void update();

  /**
   * @brief Gets the transformation matrix from foot to camera
   * @return Matrix4f
   */
  const Matrix4f& getCamInFoot();

  /**
   * Converts the posize_t in image coordinates on height Z to the
   * world coordinates. The static frame for the world coordinates
   * lies between the robot feet on the ground and so the world
   * coordinate Z = Constant value. The simplified equations to solve 
   * for world coordinates X, Y and Camera coordinate Z are found and used.
   *
   * @param worldPoint: Extracted X, Y in world frame.
   * @param imagePoint: Image coordinates x and y in image frame.
   * @param worldZ: Known Z coordinate in world frame.
   */
  void imageToWorld(
    Point2f& worldPoint, Point2f imagePoint, float worldZ = 0.f);

  /**
   * Converts a vector of points in image coordinates on height Z to the
   * world coordinates. The static frame for the world coordinates
   * lies between the robot feet on the ground and so the world
   * coordinate Z = Constant value. The simplified equations to solve 
   * for world coordinates X, Y and Camera coordinate Z are found and used.
   *
   * @param worldPoints: Extracted X, Y in world frame.
   * @param imagePoints: Image coordinates x and y in image frame.
   * @param worldZ: Known Z coordinate in world frame.
   */
  void imageToWorld(
    vector<Point2f>& worldPoints,
    vector<Point2f> imagePoints,
    float worldZ = 0.f);

  /**
   * Converts the posize_t in world coordinates to the image coordinates.
   * 
   * @param worldPoint: Extracted X, Y, Z in world frame.
   * @param imagePoint: Image coordinates x and y in image frame.
   */
  void worldToImage(
    const Point3f& worldPoint,
    Point2f& imagePoint);

  /**
   * Converts the points in world coordinates to the image coordinates.
   * 
   * @param worldPoints: Extracted points in world frame.
   * @param imagePoints: Image coordinates x and y in image frame.
   */
  void worldToImage(
    const vector<Point3f>& worldPoints,
    vector<Point2f>& imagePoints);

  Mat& getCamMatrixCv() { return camMatrixCv; }

  Matrix<float, 3, 4> getExtMatrix() { return extMatrix; }

  Matrix4f getExtInvMatrix() { return invExtMatrix; }

  Mat getPTransform() { return pTransform; }

  Matrix<float, 3, 4> getProjMatrix() { return projMatrix; }

private:
  //! The matrix A, for finding the solution to the system of equations
  //! defined by camera, and perspective transformation relations. 
  //! Used as inv(A) * b
  Matrix3f A;

  //! The matrix b, for finding the solution to the system of equations
  //! defined by camera, and perspective transformation relations. 
  //! Used as inv(A) * b
  Vector3f b;

  //! The camera matrices vector.
  Matrix3f camMatrix;

  //! The camera matrices vector in opencv mat.
  Mat camMatrixCv;

  //! The extrinsic matrices vector
  Matrix<float, 3, 4> extMatrix;

  //! The inverse extrinsic matrices vector
  Matrix4f invExtMatrix;

  //! The inverse projection transformation matrices
  //vector<Matrix<float, 4, 3> > invProjMatrix;

  //! The forward projection transformation matrices
  Matrix<float, 3, 4> projMatrix;

  //! Vector of cams
  CameraPtr cam;

  //! CameraModule ptr
  CameraModulePtr camModule;

  //! Perspective transform matrix
  Mat pTransform;

  //! Known points in image
  vector<Point2f> imageHomographyPoints;

  //! Known points in world
  vector<Point2f> worldHomographyPoints;

  //! Perspective transformation matrix is found
  bool persFound;

  //! Index of the camera
  CameraId camIndex;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<CameraTransform> CameraTransformPtr;

