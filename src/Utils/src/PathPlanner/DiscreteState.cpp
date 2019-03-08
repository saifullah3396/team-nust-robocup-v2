/**
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 * D* Lite (Koenig et al. 2002) partly based on the implementation
 * by J. Neufeld (http://code.google.com/p/dstarlite/)
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

#include "Utils/include/PathPlanner/DiscreteState.h"

namespace PathPlannerSpace
{

  DiscreteState::DiscreteState(double x, double y, double theta, Leg leg,
    double cellSize, int numAngleBins, int maxHashSize) :
    x(state2Cell(x, cellSize)), y(state2Cell(y, cellSize)),
      theta(angleState2Cell(theta, numAngleBins)), leg(leg), id(-1),
      hashTag(calcHashTag(x, y, theta, leg, maxHashSize))
  {
  }

  DiscreteState::DiscreteState(int x, int y, int theta, Leg leg,
    int maxHashSize) :
    x(x), y(y), theta(theta), leg(leg), id(-1),
      hashTag(calcHashTag(x, y, theta, leg, maxHashSize))
  {
  }

  DiscreteState::DiscreteState(const State& s, double cellSize,
    int numAngleBins, int maxHashSize) :
    x(state2Cell(s.getX(), cellSize)), y(state2Cell(s.getY(), cellSize)),
      theta(angleState2Cell(s.getTheta(), numAngleBins)), leg(s.getLeg()),
      id(-1), hashTag(calcHashTag(x, y, theta, leg, maxHashSize))
  {
  }

  DiscreteState::DiscreteState(const DiscreteState& s) :
    x(s.getX()), y(s.getY()), theta(s.getTheta()), leg(s.getLeg()),
      id(s.getId()), hashTag(s.getHashTag())
  {
  }

  DiscreteState::~DiscreteState()
  {
  }

  bool
  DiscreteState::operator ==(const DiscreteState& s2) const
  {
    // First test the hash tag. If they differ, the states are definitely
    // different.
    if (hashTag != s2.getHashTag()) return false;

    return (x == s2.getX() && y == s2.getY() && theta == s2.getTheta() && leg == s2.getLeg());
  }

  bool
  DiscreteState::operator !=(const DiscreteState& s2) const
  {
    return hashTag != s2.getHashTag();
  }

  State
  DiscreteState::getState(double cellSize, int numAngleBins) const
  {
    return State(
      cell2State(x, cellSize),
      cell2State(y, cellSize),
      angles::normalizeAngle(angleCell2State(theta, numAngleBins)),
      leg);
  }

}
;
