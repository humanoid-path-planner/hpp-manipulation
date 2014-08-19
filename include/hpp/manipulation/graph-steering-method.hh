// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
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
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.


#ifndef HPP_MANIPULATION_GRAPH_STEERING_METHOD_HH
# define HPP_MANIPULATION_GRAPH_STEERING_METHOD_HH

# include <hpp/core/steering-method.hh>

#include "hpp/manipulation/fwd.hh"

namespace hpp {
  namespace manipulation {
    class GraphSteeringMethod : public SteeringMethod
    {
      public:
        GraphSteeringMethod (SteeringMethod)
      protected:
        virtual PathPtr_t impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;
    }
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STEERING_METHOD_HH
