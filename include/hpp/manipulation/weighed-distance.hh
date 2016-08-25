// Copyright (c) 2015 CNRS
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

#ifndef HPP_MANIPULATION_DISTANCE_HH
# define HPP_MANIPULATION_DISTANCE_HH

# include <hpp/core/weighed-distance.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    /// \addtogroup steering_method
    /// \{

    /// Class for distance between configurations
    class HPP_MANIPULATION_DLLAPI WeighedDistance : public core::WeighedDistance
    {
    public:
      static WeighedDistancePtr_t create (const DevicePtr_t& robot,
          const graph::GraphPtr_t& graph);

      static WeighedDistancePtr_t createCopy
	(const WeighedDistancePtr_t& distance);

      virtual core::DistancePtr_t clone () const;

      /// Set the graph of constraints
      void constraintGraph (const graph::GraphPtr_t& graph)
      {
        graph_ = graph;
      }

      /// Get the graph of constraints
      graph::GraphPtr_t constraintGraph () const
      {
        return graph_;
      }

    protected:
      WeighedDistance (const DevicePtr_t& robot, const graph::GraphPtr_t graph);

      WeighedDistance (const WeighedDistance& distance);

      /// Derived class should implement this function
      virtual value_type impl_distance (
          ConfigurationIn_t q1, ConfigurationIn_t q2) const;
      virtual value_type impl_distance (
          core::NodePtr_t n1, core::NodePtr_t n2) const;

      void init (WeighedDistanceWkPtr_t self);

    private:
      graph::GraphPtr_t graph_;
      WeighedDistanceWkPtr_t weak_;
    }; // class Distance
    /// \}
  } //   namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_DISTANCE_HH
