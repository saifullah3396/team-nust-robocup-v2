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

#include "Utils/include/PathPlanner/DiscreteStateChangeQuery.h"

namespace PathPlannerSpace
{

  DiscreteStateChangeQuery::DiscreteStateChangeQuery(
    const vector<int>& neighbors) :
    neighbors(neighbors)
  {
  }

  DiscreteStateChangeQuery::~DiscreteStateChangeQuery()
  {
  }

  const vector<int>*
  DiscreteStateChangeQuery::getPredecessors() const
  {
    return &neighbors;
  }

  const vector<int>*
  DiscreteStateChangeQuery::getSuccessors() const
  {
    return &neighbors;
  }

}
