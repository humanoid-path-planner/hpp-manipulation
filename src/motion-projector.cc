//
// Copyright (c) 2014 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-manipulation
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation  If not, see
// <http://www.gnu.org/licenses/>.

#include <limits>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/differentiable-function.hh>
#include <hpp/core/locked-dof.hh>

#include "hpp/manipulation/motion-projector.hh"

namespace hpp {
  namespace manipulation {
    MMotionProjectorPtr_t MotionProjector::create (const DevicePtr_t& robot,
						  const std::string& name,
						  value_type errorThreshold,
						  size_type maxIterations)
    {
      MotionProjector* ptr = new MotionProjector (robot, name, errorThreshold,
						  maxIterations);
      MotionProjectorPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    MotionProjector::MotionProjector (const DevicePtr_t& robot,
				      const std::string& name,
				      value_type errorThreshold,
				      size_type maxIterations) :
      ConfigProjector (robot, name, errorThreshold, maxIterations), weak_ ()
    {
    }

    void MotionProjector::addConstraint
    (const DifferentiableFunctionPtr_t& constraint)
    {
      ConfigProjector::addConstraint (constraint);
      resize ();
    }

    void MotionProjector::resize ()
    {
      offset_.resize(value_.size());
    }

    bool MotionProjector::impl_compute (ConfigurationOut_t configuration)
    {
      return false;
    }

    std::ostream& MotionProjector::print (std::ostream& os) const
    {
      os << "Motion projector: " << name () << ", contains" << std::endl;
      for (NumericalConstraints_t::const_iterator it = constraints_.begin ();
	   it != constraints_.end (); it++) {
	const DifferentiableFunction& f (*(it->function));
	os << f << std::endl;
      }
      return os;
    }

  } // namespace manipulation
} // namespace hpp
