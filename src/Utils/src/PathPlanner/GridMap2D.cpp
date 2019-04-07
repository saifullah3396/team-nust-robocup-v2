/**
 * A simple 2D gridmap structure
 *
 * Copyright 2011 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "Utils/include/PathPlanner/GridMap2D.h"

namespace PathPlannerSpace
{

  GridMap2D::GridMap2D(OccupancyMap<float>* occMap, bool unknownAsObstacle) :
    occMap(occMap)
  {
    distMap = Mat(occMap->data.size(), CV_32FC1);
    distMapR = Mat(occMap->data.size() * (1 / 8), CV_32FC1);
  }

  GridMap2D::~GridMap2D() {}

  void GridMap2D::updateDistanceMap()
  {
    Mat binaryMapR;
    resize(
      occMap->data,
      binaryMapR,
      Size(
        occMap->data.size().width / 4,
        occMap->data.size().height / 4
      ));
    distanceTransform(binaryMapR, distMapR, CV_DIST_L2, 3);

    ///< distance map now contains distance in meters:
    resize(distMapR, distMap, occMap->data.size());
    distMap = distMap * occMap->resolution * 4;
    //high_resolution_clock::time_point t1 =
    //high_resolution_clock::now();
    //duration<double> time_span = t1 - tStart;
    //cout << "distTransform: " << time_span.count() << "secs." << endl;
  }

  void GridMap2D::inflateMap(double inflationRadius)
  {
    occMap->data = (distMap > inflationRadius);
    ///< recompute distance map with new binary map:
    distanceTransform(occMap->data, distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    distMap = distMap * occMap->resolution;
  }

  ///< See Costmap2D for mapToWorld / worldToMap implementations:
  void GridMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx,
    double& wy) const
  {
    //wx = (mx + mapOriginPose.x + 0.5) * resolution;
    //wy = (mapOriginPose.y - my - 0.5) * resolution;
    wx = (mx + 0.5) * occMap->resolution;
    wy = (-my - 0.5) * occMap->resolution;

  }

  void
  GridMap2D::worldToMapNoBounds(double wx, double wy, unsigned int& mx,
    unsigned int& my) const
  {
    mx = (int) (wx / occMap->resolution + occMap->originPose.x);
    my = (int) (occMap->originPose.y - wy / occMap->resolution);
  }

  bool
  GridMap2D::worldToMap(double wx, double wy, unsigned int& mx,
    unsigned int& my) const
  {
    //if(wx  mapOriginPose.x || wy < mapOriginPose.y)
    //  return false;
    mx = (int) (wx / occMap->resolution + occMap->originPose.x);
    my = (int) (occMap->originPose.y - wy / occMap->resolution);
    if (mx < occMap->data.size().width && my < occMap->data.size().height) return true;
    return false;
  }

  bool GridMap2D::inMapBounds(double wx, double wy) const
  {
    unsigned mx, my;
    return worldToMap(wx, wy, mx, my);
  }

  float GridMap2D::distanceMapAt(double wx, double wy) const
  {
    unsigned mx, my;
    if (worldToMap(wx, wy, mx, my)) return distMap.at<float>(my, mx);
    else return -1.0f;
  }

  uchar GridMap2D::binaryMapAt(double wx, double wy) const
  {
    unsigned mx, my;
    if (worldToMap(wx, wy, mx, my)) return occMap->data.at < uchar > (my, mx);
    else return 0;
  }

  float GridMap2D::distanceMapAtCell(unsigned int mx, unsigned int my) const
  {
    return distMap.at<float>(my, mx);
  }

  uchar GridMap2D::binaryMapAtCell(unsigned int mx, unsigned int my) const
  {
    return occMap->data.at < uchar > (my, mx);
  }

  uchar& GridMap2D::binaryMapAtCell(unsigned int mx, unsigned int my)
  {
    return occMap->data.at < uchar > (my, mx);
  }

  bool
  GridMap2D::isOccupiedAtCell(unsigned int mx, unsigned int my) const
  {
    return (occMap->data.at < uchar > (my, mx) < 255);
  }

  bool
  GridMap2D::isOccupiedAt(double wx, double wy) const
  {
    unsigned mx, my;
    if (worldToMap(wx, wy, mx, my)) return isOccupiedAtCell(mx, my);
    else return true;
  }

}
