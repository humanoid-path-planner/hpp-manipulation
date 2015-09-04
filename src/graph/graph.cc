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

#include "hpp/manipulation/graph/graph.hh"

#include <hpp/util/assertion.hh>

#include "hpp/manipulation/graph/node-selector.hh"
#include "hpp/manipulation/graph/node.hh"
#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      GraphPtr_t Graph::create(const std::string& name, DevicePtr_t robot,
			       const core::SteeringMethodPtr_t& sm)
      {
        Graph* ptr = new Graph (name, sm);
        GraphPtr_t shPtr (ptr);
        ptr->init (shPtr, robot);
        return shPtr;
      }

      void Graph::init (const GraphWkPtr_t& weak, DevicePtr_t robot)
      {
        GraphComponent::init (weak);
        robot_ = robot;
        wkPtr_ = weak;
      }

      NodeSelectorPtr_t Graph::createNodeSelector (const std::string& name)
      {
        nodeSelector_ = NodeSelector::create (name);
        nodeSelector_->parentGraph (wkPtr_);
        return nodeSelector_;
      }

      void Graph::nodeSelector (NodeSelectorPtr_t ns)
      {
        nodeSelector_ = ns;
        nodeSelector_->parentGraph (wkPtr_);
      }

      void Graph::maxIterations (size_type iterations)
      {
        maxIterations_ = iterations;
      }

      size_type Graph::maxIterations () const
      {
        return maxIterations_;
      }

      void Graph::errorThreshold (const value_type& threshold)
      {
        errorThreshold_ = threshold;
      }

      value_type Graph::errorThreshold () const
      {
        return errorThreshold_;
      }

      const DevicePtr_t& Graph::robot () const
      {
        return robot_;
      }

      const core::SteeringMethodPtr_t& Graph::steeringMethod () const
      {
	return steeringMethod_;
      }

      NodePtr_t Graph::getNode (ConfigurationIn_t config) const
      {
        return nodeSelector_->getNode (config);
      }

      NodePtr_t Graph::getNode(RoadmapNodePtr_t coreNode) const
      {
        return nodeSelector_->getNode (coreNode);
      }

      Edges_t Graph::getEdges (const NodePtr_t& from, const NodePtr_t& to) const
      {
        Edges_t edges;
        for (Neighbors_t::const_iterator it = from->neighbors ().begin ();
            it != from->neighbors ().end (); ++it) {
          if (it->second->to () == to)
            edges.push_back (it->second);
        }
        return edges;
      }

      EdgePtr_t Graph::chooseEdge (RoadmapNodePtr_t from) const
      {
        return nodeSelector_->chooseEdge (from);
      }

      ConstraintSetPtr_t Graph::configConstraint (const NodePtr_t& node)
      {
        return node->configConstraint ();
      }

      bool Graph::getConfigErrorForNode (ConfigurationIn_t config,
					 const NodePtr_t& node, vector_t& error)
      {
	return configConstraint (node)->isSatisfied (config, error);
      }

      ConstraintSetPtr_t Graph::configConstraint (const EdgePtr_t& edge)
      {
        return edge->configConstraint ();
      }

      ConstraintSetPtr_t Graph::pathConstraint (const EdgePtr_t& edge)
      {
        return edge->pathConstraint ();
      }

      std::ostream& Graph::dotPrint (std::ostream& os, dot::DrawingAttributes da) const
      {
        da.separator = "; ";
        da.openSection = "\n";
        da.closeSection = ";\n";
        dot::Tooltip tp; tp.addLine ("Graph constains:");
        populateTooltip (tp);
        da.insertWithQuote ("tooltip", tp.toStr());
        os << "digraph " << id() << " {" << da;
        nodeSelector_->dotPrint (os);
        os << "}" << std::endl;
        return os;
      }

      std::ostream& Graph::print (std::ostream& os) const
      {
        return GraphComponent::print (os) << std::endl << *nodeSelector_;
      }
    } // namespace graph
  } // namespace manipulation

} // namespace hpp
