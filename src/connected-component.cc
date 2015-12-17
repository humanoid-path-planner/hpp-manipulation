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

#include <hpp/manipulation/connected-component.hh>

namespace hpp {
  namespace manipulation {

    ConnectedComponent::ManipulationConnectedComponentPtr_t 
      ConnectedComponent::create(const RoadmapWkPtr_t& Roadmap)
    {
      ConnectedComponent* ptr = new ConnectedComponent ();
      ConnectedComponent::ManipulationConnectedComponentPtr_t shPtr (ptr);
      // calls init function in core::ConnectedComponent that saves 
      // shPtr into the class variable weak_ (weak pointer). Reimplement?
      ptr->init (shPtr);
      Roadmap_ = Roadmap.lock ();
      return shPtr;
    }
    
    void ConnectedComponent::merge (const ManipulationConnectedComponentPtr_t& other)
    {
      core::ConnectedComponent::merge(other);

// take all graph nodes in other->GraphNodeMap_ and put them in this->GraphNodeMap_
// if they already exist in this->GraphNodeMap_, append roadmap nodes from graph node in other
// to graph node in this. 

      other->GraphNodeMap_.clear();
    } 

    void ConnectedComponent::addNode(const RoadmapNodePtr_t& node)      
    {
      core::ConnectedComponent::addNode(node);
      // Find right graph node in map and add roadmap node to corresponding vector
      GraphNodes_t::iterator mapIt = GraphNodeMap_.find(Roadmap_->getNode(node));
      if (mapIt != GraphNodeMap_.end()) {
        mapIt->second.push_back(node);
      // if graph node not found, add new map element with one roadmap node
      } else {
	RoadmapNodes_t newRoadmapNodeVector;
	newRoadmapNodeVector.push_back(node);
	GraphNodeMap_.insert(std::pair<graph::NodePtr_t, RoadmapNodes_t>
	  (Roadmap_->getNode(node), newRoadmapNodeVector));
      }

    }


  } // namespace manipulation
} // namespace hpp

