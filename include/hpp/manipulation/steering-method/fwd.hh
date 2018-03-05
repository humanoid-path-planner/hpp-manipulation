//
// Copyright (c) 2017 CNRS
// Authors: Joseph Mirabel
//
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_STEERING_METHOD_FWD_HH
# define HPP_MANIPULATION_STEERING_METHOD_FWD_HH

# include <map>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace steeringMethod {
      HPP_PREDEF_CLASS (Graph);
      typedef boost::shared_ptr < Graph > GraphPtr_t;
      HPP_PREDEF_CLASS (CrossStateOptimization);
      typedef boost::shared_ptr < CrossStateOptimization > CrossStateOptimizationPtr_t;
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_FWD_HH
