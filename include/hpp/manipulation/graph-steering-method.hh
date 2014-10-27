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
# include <hpp/core/weighed-distance.hh>
# include <hpp/model/device.hh>

# include "hpp/manipulation/fwd.hh"

namespace hpp {
  namespace manipulation {
    using core::SteeringMethod;
    using core::PathPtr_t;

    class HPP_MANIPULATION_DLLAPI GraphSteeringMethod : public SteeringMethod
    {
      public:
        /// Constructor
        GraphSteeringMethod (const DevicePtr_t& robot);

        /// \name Graph of constraints applicable to the robot.
        /// \{

        /// Set constraint graph
        void constraintGraph (const graph::GraphPtr_t& graph)
        {
          graph_ = graph;
        }

        /// Get constraint graph
        const graph::GraphPtr_t& constraintGraph () const
        {
          return graph_;
        }
        /// \}

        const core::WeighedDistancePtr_t& distance () const;

      protected:
        virtual PathPtr_t impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

      private:
        /// A pointer to the graph of constraint.
        graph::GraphPtr_t graph_;
        /// Pointer to the Robot.
        core::DeviceWkPtr_t robot_;
        /// Metric in configuration space.
        core::WeighedDistancePtr_t distance_;
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STEERING_METHOD_HH
