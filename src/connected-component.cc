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

#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/roadmap-node.hh"

namespace hpp {
  namespace manipulation {

    ConnectedComponentPtr_t ConnectedComponent::create(const RoadmapWkPtr_t& roadmap)
    {
      ConnectedComponent* ptr = new ConnectedComponent ();
      ConnectedComponentPtr_t shPtr (ptr);
      // calls init function in core::ConnectedComponent that saves 
      // shPtr into the class variable weak_ (weak pointer). Reimplement?
      ptr->init (shPtr);
      shPtr->roadmap_ = roadmap.lock();
      return shPtr;
    }

//    void ConnectedComponent::setRoadmap (const RoadmapWkPtr_t& roadmap, ConnectedComponentPtr_t CC)
//    {
//      CC->rdmp_ = roadmap.lock ();
//    }
 
    void ConnectedComponent::merge (const core::ConnectedComponentPtr_t& otherCC)
    {
      core::ConnectedComponent::merge(otherCC);
      const ConnectedComponentPtr_t other = boost::static_pointer_cast <ConnectedComponent> (otherCC);
      /// take all graph nodes in other->graphNodeMap_ and put them in this->graphNodeMap_
      /// if they already exist in this->graphNodeMap_, append roadmap nodes from other graph node
      /// to graph node in this. 
      for (GraphNodes_t::iterator otherIt = other->graphNodeMap_.begin(); 
	otherIt != other->graphNodeMap_.end(); otherIt++)
      {
	// find other graph node in this-graphNodeMap_ -> merge their roadmap nodes
	GraphNodes_t::iterator mapIt = this->graphNodeMap_.find(otherIt->first);
	if (mapIt != this->graphNodeMap_.end())	{
	  mapIt->second.insert(mapIt->second.end(), otherIt->second.begin(), otherIt->second.end());
	} else {
	  this->graphNodeMap_.insert(*otherIt);
	}
      }
      other->graphNodeMap_.clear();
    } 

    void ConnectedComponent::addNode(const core::NodePtr_t& node)      
    {
      core::ConnectedComponent::addNode(node);
      // Find right graph node in map and add roadmap node to corresponding vector
      const RoadmapNodePtr_t& n = static_cast <const RoadmapNodePtr_t> (node);
      GraphNodes_t::iterator mapIt = graphNodeMap_.find(roadmap_->getNode(n));
      if (mapIt != graphNodeMap_.end()) {
        mapIt->second.push_back(n);
      // if graph node not found, add new map element with one roadmap node
      } else {
	RoadmapNodes_t newRoadmapNodeVector;
	newRoadmapNodeVector.push_back(n);
	graphNodeMap_.insert(std::pair<graph::NodePtr_t, RoadmapNodes_t>
	  (roadmap_->getNode(n), newRoadmapNodeVector));
      }

    }

    ConnectedComponent::RoadmapNodes_t ConnectedComponent::getRoadmapNodes (const graph::NodePtr_t graphNode)
    {
      RoadmapNodes_t res;
      GraphNodes_t::iterator mapIt = graphNodeMap_.find(graphNode);
      if (mapIt != graphNodeMap_.end()) {
        res = mapIt->second;
      }
      return res;
    }


  } // namespace manipulation
} // namespace hpp

