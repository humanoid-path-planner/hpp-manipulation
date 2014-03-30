// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/manipulation/object.hh>
#include <hpp/manipulation/problem-solver.hh>
#include <hpp/manipulation/robot.hh>

namespace hpp {
  namespace manipulation {
    void ProblemSolver::buildCompositeRobot (const std::string& robotName,
					     const Names_t& robotNames)
    {
      Devices_t robots;
      Objects_t objects;
      Names_t::const_iterator itName = robotNames.begin ();
      for (;
	   itName != robotNames.end (); ++itName) {
	const DevicePtr_t& rob (robotsAndObjects_ [*itName]);
	ObjectPtr_t object = HPP_DYNAMIC_PTR_CAST (Object, rob);
	if (object) {
	  objects.push_back (object);
	} else {
	  robots.push_back (rob);
	}
      }
      RobotPtr_t composite (Robot::create (robotName, robots, objects));
      robot (composite);
    }

  } // namespace manipulation
} // namespace hpp
