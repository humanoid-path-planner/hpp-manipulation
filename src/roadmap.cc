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

#include <hpp/manipulation/roadmap.hh>

#include <hpp/util/pointer.hh>

#include <hpp/core/edge.hh>
#include <hpp/core/distance.hh>

#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/roadmap-node.hh>
#include <hpp/manipulation/symbolic-component.hh>
#include <hpp/manipulation/connected-component.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/statistics.hh>

namespace hpp {
  namespace manipulation {
    Roadmap::Roadmap (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot) :
      core::Roadmap (distance, robot), weak_ () {}

    RoadmapPtr_t Roadmap::create (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot)
    {
      Roadmap* ptr = new Roadmap (distance, robot);
      RoadmapPtr_t shPtr (ptr);
      ptr->init(shPtr);
      return shPtr; 
    }

    void Roadmap::clear ()
    {
      Parent::clear ();
      Histograms_t::const_iterator it;
      for (it = histograms_.begin(); it != histograms_.end(); ++it)
        (*it)->clear ();
      if (graph_) {
        const Histograms_t& hs = graph_->histograms();
        for (it = hs.begin(); it != hs.end(); ++it)
          (*it)->clear ();
      }
    }

    void Roadmap::push_node (const core::NodePtr_t& n)
    {
      Parent::push_node (n);
      const RoadmapNodePtr_t& node = 
        static_cast <const RoadmapNodePtr_t> (n);
      statInsert (node);
      symbolicCCs_.insert(node->symbolicComponent());
    }

    void Roadmap::statInsert (const RoadmapNodePtr_t& n)
    {
      Histograms_t::const_iterator it;
      for (it = histograms_.begin(); it != histograms_.end(); ++it)
        (*it)->add (n);
      if (graph_) {
        const Histograms_t& hs = graph_->histograms();
        for (it = hs.begin(); it != hs.end(); ++it)
          (*it)->add (n);
      }
    }

    void Roadmap::insertHistogram (const graph::HistogramPtr_t hist)
    {
      histograms_.push_back (hist);
      core::Nodes_t::const_iterator _node;
      for (_node = nodes().begin(); _node != nodes().end(); ++_node)
        hist->add (static_cast <const RoadmapNodePtr_t>(*_node));
    }

    void Roadmap::constraintGraph (const graph::GraphPtr_t& graph)
    {
      graph_ = graph;
      // FIXME Add the current nodes() to the graph->histograms()
      // The main issue is that new histograms may be added to
      // graph->histograms() and this class will not know it.
    }

    RoadmapNodePtr_t Roadmap::nearestNode (const ConfigurationPtr_t& configuration,
        const ConnectedComponentPtr_t& connectedComponent,
        const graph::NodePtr_t& node,
        value_type& minDistance) const
    {
      core::NodePtr_t result = NULL;
      minDistance = std::numeric_limits <value_type>::infinity ();
      const RoadmapNodes_t& roadmapNodes = connectedComponent->getRoadmapNodes (node);
      for (RoadmapNodes_t::const_iterator itNode = roadmapNodes.begin ();
          itNode != roadmapNodes.end (); ++itNode) {
        value_type d = (*distance()) (*(*itNode)->configuration (),
            *configuration);
        if (d < minDistance) {
          minDistance = d;
          result = *itNode;
        }
      }
      return static_cast <RoadmapNode*> (result);
    }

    core::NodePtr_t Roadmap::createNode (const ConfigurationPtr_t& q) const
    {
      // call RoadmapNode constructor with new manipulation connected component
      RoadmapNodePtr_t node = new RoadmapNode (q, ConnectedComponent::create(weak_));
      SymbolicComponentPtr_t sc = WeighedSymbolicComponent::create (weak_.lock());
      node->symbolicComponent (sc);
      sc->setFirstNode(node);
      return node;
    }

    graph::NodePtr_t Roadmap::getNode(RoadmapNodePtr_t node)
    {
      return graph_->getNode(node);
    }

    void Roadmap::addEdge (const core::EdgePtr_t& edge)
    {
      Parent::addEdge(edge);
      const RoadmapNodePtr_t& f = static_cast <const RoadmapNodePtr_t> (edge->from());
      const RoadmapNodePtr_t& t = static_cast <const RoadmapNodePtr_t> (edge->to());
      SymbolicComponentPtr_t scf = f->symbolicComponent();
      SymbolicComponentPtr_t sct = t->symbolicComponent();
      scf->canReach(sct);
      if (scf->canMerge(sct)) {
        if (scf->nodes().size() > sct->nodes().size()) {
          scf->merge(sct);
          symbolicCCs_.erase(sct);
        } else {
          sct->merge(scf);
          symbolicCCs_.erase(scf);
        }
      }
    }
  } // namespace manipulation
} // namespace hpp
