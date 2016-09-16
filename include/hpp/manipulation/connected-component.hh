//
// Copyright (c) 2015 CNRS
// Authors: Anna Seppala (seppala@laas.fr)
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_CONNECTED_COMPONENT_HH
#define HPP_MANIPULATION_CONNECTED_COMPONENT_HH

#include <hpp/core/connected-component.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    /// Extension of hpp::core::connected-component. Adds a list of roadmap nodes for
    /// every contraint graph state within the connected component. Thus every roadmap
    /// node is assigned to a grahp state, which minimises computation time.
class HPP_MANIPULATION_DLLAPI ConnectedComponent : public core::ConnectedComponent 
{ 
  public:
      /// Map of graph states within the connected component
      typedef std::map <graph::StatePtr_t, RoadmapNodes_t> GraphStates_t;
     
      ConnectedComponent() {}

      /// return a shared pointer to new instance of manipulation::ConnectedComponent 
      static ConnectedComponentPtr_t create (const RoadmapWkPtr_t& roadmap);

      /// Merge two connected components (extension of core::ConnectedComponent::merge)
      /// \param other manipulation connected component to merge into this one.
      /// \note other will be empty after calling this method.
      void merge (const core::ConnectedComponentPtr_t& otherCC); 
         
      /// Add roadmap node to connected component
      /// \param roadmap node to be added
      void addNode (const core::NodePtr_t& node);

      const RoadmapNodes_t& getRoadmapNodes (const graph::StatePtr_t graphState) const;
     
  protected:
  private:
      bool check () const;
	GraphStates_t graphStateMap_;
	RoadmapPtr_t roadmap_;
        static RoadmapNodes_t empty_;
    }; // class ConnectedComponent
  } //   namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_CONNECTED_COMPONENT_HH
