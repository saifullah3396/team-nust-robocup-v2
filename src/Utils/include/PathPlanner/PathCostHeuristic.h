/**
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstepPlanner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "Utils/include/PathPlanner/MyHeuristic.h"
#include "Utils/include/PathPlanner/DiscreteState.h"
#include "Utils/include/PathPlanner/GridMap2D.h"
#include "SbplLib/include/headers.h"

namespace PathPlannerSpace
{

  /**
   * @brief Determining the heuristic value by calculating a 2D path from each
   * grid cell of the map to the goal and using the path length as expected
   * distance.
   *
   * The heuristic value consists of the following factors:
   *
   *  + The expected distance retreived from the 2D path.
   *
   *  + The expected path costs.
   *
   *  + The difference between the orientation of the two states multiplied
   *    by some cost factor.
   */
  class PathCostHeuristic : public MyHeuristic
  {
  public:
    PathCostHeuristic(double cellSize, int numAngleBins, double stepCost,
      double diffAngleCost, double maxStepWidth, double inflationRadius);
    virtual
    ~PathCostHeuristic();

    /**
     * @return The estimated costs needed to reach the state 'to' from within the
     * current state.
     */
    virtual double
    getHValue(const DiscreteState& current, const DiscreteState& to) const;

    /**
     * @brief Calculates for each grid cell of the map a 2D path to the
     * cell (to.x, to.y).
     * For forward planning 'to' is supposed to be the goal state, for backward
     * planning 'to' is supposed to be the start state.
     */
    bool
    calculateDistances(const DiscreteState& from, const DiscreteState& to);

    void
    updateMap(GridMap2DPtr map);

  private:
    static const int cvObstacleThreshold = 200;

    unsigned char** grid;

    double stepCost;
    double diffAngleCost;
    double maxStepWidth;
    double inflationRadius;

    int goalX;
    int goalY;

    GridMap2DPtr mapPtr;
    boost::shared_ptr<SBPL2DGridSearch> gridSearchPtr;

    void
    resetGrid();
  };

}
;
