/**
 * @file VisionModule/src/CameraTransform.cpp
 *
 * This file implements the class CameraTransform.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Oct 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/HardwareIds.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/CameraModule.h"
#include "Utils/include/MathsUtils.h"

CameraTransform::CameraTransform(
  VisionModule* visionModule,
  const CameraId& camIndex) :
  MemoryBase(visionModule),
  camModule(visionModule->getCameraModule()),
  camIndex(camIndex)
{
  persFound = false;
  cam = camModule->getCameraPtr(camIndex);
  computeCamMatrix();
  update();
}

void CameraTransform::computeCamMatrix()
{
  camMatrix(0, 0) = cam->focalX;
  camMatrix(0, 1) = 0.f;
  camMatrix(0, 2) = cam->centerOffX;
  camMatrix(1, 0) = 0.f;
  camMatrix(1, 1) = cam->focalY;
  camMatrix(1, 2) = cam->centerOffY;
  camMatrix(2, 0) = 0.f;
  camMatrix(2, 1) = 0.f;
  camMatrix(2, 2) = 1;
  eigen2cv(camMatrix, camMatrixCv);
}


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

void CameraTransform::update()
{
  Matrix4f T = getCamInFoot();
  invExtMatrix = MathsUtils::getTInverse(T);
  extMatrix = T.block(0, 0, 3, 4);
  b[0] = -T(0, 3);
  b[1] = -T(1, 3);
  b[2] = -T(2, 3);
  A(0, 0) = T(0, 0);
  A(0, 1) = T(0, 1);
  A(0, 2) = 1;
  A(1, 0) = T(1, 0);
  A(1, 1) = T(1, 1);
  A(1, 2) = 1;
  A(2, 0) = T(2, 0);
  A(2, 1) = T(2, 1);
  A(2, 2) = -1;
  projMatrix = camMatrix * extMatrix;
  //MatrixXf proj = projMatrix;
  //invProjMatrix = MathsUtils::pseudoInverse(proj);
  /*vector<Vector4f> points;
   points.push_back(Vector4f(3.0, 0.0, 0.0, 1.0));
   points.push_back(Vector4f(2.0, 1.0, 0.0, 1.0));
   points.push_back(Vector4f(2.0, -1.0, 0.0, 1.0));
   for (size_t i = 0; i < points.size(); ++i) {
   points = T * points;
   }
   plane = computePlaneEquation(points);*/
  //cout << "upper cam: " << endl;
  //Vector4f wP1(1.0f, 0.f, 0.f, 1.f);
  //Vector4f wP2(0.f, -4.5f, 0.f, 1.f);
  //cout << "twp1: "<< extMatrix[0] * wP1 << endl;
  //cout << "twp2: "<< extMatrix[0] * wP2 << endl;
  worldHomographyPoints.clear();
  imageHomographyPoints.clear();
  persFound = false;
}

const Matrix4f& CameraTransform::getCamInFoot() {
  if (camIndex == CameraId::headTop)
    return UPPER_CAM_TRANS_IN(VisionModule);
  else
    return LOWER_CAM_TRANS_IN(VisionModule);
}

void CameraTransform::imageToWorld(
  Point2f& worldPoint, Point2f imagePoint, float worldZ)
{
  if (worldZ != 0.f) {
    A(0, 2) = (cam->centerOffX - imagePoint.x) / cam->focalX;
    A(1, 2) = (cam->centerOffY - imagePoint.y) / cam->focalY;
    Vector3f tempB;
    tempB[0] = b[0] - extMatrix(0, 2) * worldZ;
    tempB[1] = b[1] - extMatrix(1, 2) * worldZ;
    tempB[2] = b[2] - extMatrix(2, 2) * worldZ;
    Vector3f sol = A.inverse() * tempB;
    worldPoint.x = sol[0];
    worldPoint.y = sol[1];
  } else {
    if (persFound) {
      vector<Point2f> src, dst;
      src.push_back(imagePoint);
      perspectiveTransform(src, dst, pTransform);
      worldPoint = dst[0];
    } else {
      A(0, 2) =
        (cam->centerOffX - imagePoint.x) / cam->focalX;
      A(1, 2) =
        (cam->centerOffY - imagePoint.y) / cam->focalY;
      Vector3f tempB;
      tempB[0] = b[0] - extMatrix(0, 2) * worldZ;
      tempB[1] = b[1] - extMatrix(1, 2) * worldZ;
      tempB[2] = b[2] - extMatrix(2, 2) * worldZ;
      Vector3f sol = A.inverse() * tempB;
      worldPoint.x = sol[0];
      worldPoint.y = sol[1];
      imageHomographyPoints.push_back(imagePoint);
      worldHomographyPoints.push_back(worldPoint);
      if (imageHomographyPoints.size() == 4 && !persFound) {
        //cout << "worldPoints: " << worldPoints << endl;
        /*float minX = worldPoints[0].x;
         float minY = worldPoints[0].y;
         for (size_t i = 1; i < worldPoints.size(); ++i)
         {
         minX = min(worldPoints[i].x, minX);
         minY = min(worldPoints[i].y, minY);
         }
         //Point2f worldOffset = Point2f(minX, minY);
         for (size_t i = 0; i < worldPoints.size(); ++i) {
         worldPoints[i] = worldPoints[i] * 50;
         //float x = worldPoints[i].x;
         worldPoints[i].x += 125;
         worldPoints[i].y = 350 / 2 - worldPoints[i].y;
         }*/
        try {
          pTransform = findHomography(imageHomographyPoints, worldHomographyPoints);
          persFound = true;
        } catch (cv::Exception &e){
          LOG_EXCEPTION(e.what());
          persFound = false;
          worldHomographyPoints.clear();
          imageHomographyPoints.clear();
        }
        //cout << "p: " << pTransform << endl;
        //warpPerspective(inImage,  persFound, H,  persFound.size());
        //pTransform = getPerspectiveTransform(imageHomographyPoints, worldPoints);
      }
    }
  }
}

void CameraTransform::imageToWorld(
  vector<Point2f>& worldPoints,
  vector<Point2f> imagePoints,
  float worldZ)
{
  worldPoints.resize(imagePoints.size());
  undistortPoints(
    imagePoints,
    imagePoints,
    camMatrixCv,
    cam->distCoeffs,
    noArray(),
    camMatrixCv);
  if (worldZ != 0.f) {
    for (size_t i = 0; i < imagePoints.size(); ++i) {
      imageToWorld(worldPoints[i], imagePoints[i], worldZ);
    }
  } else {
    if (persFound) {
      perspectiveTransform(imagePoints, worldPoints, pTransform);
    } else {
      for (size_t i = 0; i < imagePoints.size(); ++i) {
        imageToWorld(worldPoints[i], imagePoints[i], worldZ);
        if (persFound) {
          perspectiveTransform(imagePoints, worldPoints, pTransform);
          break;
        }
      }
    }
  }
}

void CameraTransform::worldToImage(
  const Point3f& worldPoint,
  Point2f& imagePoint)
{
  Vector4f wP(worldPoint.x, worldPoint.y, worldPoint.z, 1.f);
  Vector3f iP = projMatrix * wP;
  //cout << "camMatrix: " << endl << camMatrix << endl;
  //cout << "cam->width : " << endl << cam->width << endl;
  //cout << "cam->height : " << endl << cam->height << endl;
  imagePoint.x = iP[0] / iP[2];
  imagePoint.y = iP[1] / iP[2];
  //imagePoint.x = imagePoint.x + cam->width / 2;
  //imagePoint.y = imagePoint.y + cam->height / 2;
  //cout << "worldToImageXY: " << imagePoint << endl;
  //cout << "worldToImageScale: " << iP[2] << endl;
}

void CameraTransform::worldToImage(
  const vector<Point3f>& worldPoints,
  vector<Point2f>& imagePoints)
{
  imagePoints.resize(worldPoints.size());
  for (size_t i = 0; i < worldPoints.size(); ++i) {
    Vector4f wP(worldPoints[i].x, worldPoints[i].y, worldPoints[i].z, 1.f);
    Vector3f iP = projMatrix * wP;
    //cout << "camMatrix: " << endl << camMatrix << endl;
    //cout << "cam->width : " << endl << cam->width << endl;
    //cout << "cam->height : " << endl << cam->height << endl;
    if (iP[2] > 0) {
      imagePoints[i].x = iP[0] / iP[2];
      imagePoints[i].y = iP[1] / iP[2];
    } else {
      imagePoints[i].x = 1000;
      imagePoints[i].y = 1000;
    }
    //cout << "scale: " << iP[2] << endl;
    //imagePoint.x = imagePoint.x + cam->width / 2;
    //imagePoint.y = imagePoint.y + cam->height / 2;
    //cout << "worldToImageXY: " << imagePoint << endl;
    //cout << "worldToImageScale: " << iP[2] << endl;
  }
  //cout << "imagepoints: " << imagePoints << endl;
  /*Mat R = Mat(3, 3, CV_64F);
   Mat t = Mat(3, 1, CV_64F);
   R.at<double>(0, 0) = extMatrix(0, 0);
   R.at<double>(0, 1) = extMatrix(0, 1);
   R.at<double>(0, 2) = extMatrix(0, 2);
   R.at<double>(1, 0) = extMatrix(1, 0);
   R.at<double>(1, 1) = extMatrix(1, 1);
   R.at<double>(1, 2) = extMatrix(1, 2);
   R.at<double>(2, 0) = extMatrix(2, 0);
   R.at<double>(2, 1) = extMatrix(2, 1);
   R.at<double>(2, 2) = extMatrix(2, 2);

   t.at<double>(0, 0) = extMatrix(0, 3);
   t.at<double>(1, 0) = extMatrix(1, 3);
   t.at<double>(2, 0) = extMatrix(2, 3);*/

  //cout << "eigen: " << extMatrix << endl;
  //cout << "R: " << R << endl;
  //cout << "t: " << t << endl;
  //projectPoints(worldPoints, R, t, camMatrixCv, cam->distCoeffs, imagePoints);
}
