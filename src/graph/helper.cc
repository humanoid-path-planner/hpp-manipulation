// Copyright (c) 2016, Joseph Mirabel
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

#include <hpp/manipulation/graph/helper.hh>

#include <hpp/util/debug.hh>

#include "hpp/manipulation/graph/node.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/node-selector.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      template <bool forPath>
        void NumericalConstraintsAndPassiveDofs::addToComp
        (GraphComponentPtr_t comp) const
      {
        if (nc.empty ()) return;
        NodePtr_t n;
        if (forPath) {
          n = HPP_DYNAMIC_PTR_CAST (Node, comp);
          if (!n) throw std::logic_error ("Wrong type: expect a Node");
        }
        NumericalConstraints_t::const_iterator it;
        IntervalsContainer_t::const_iterator itpdof = pdof.begin ();
        for (it = nc.begin (); it != nc.end (); ++it) {
          if (forPath) n->addNumericalConstraintForPath (*it, *itpdof);
          else      comp->addNumericalConstraint (*it, *itpdof);
          ++itpdof;
        }
        assert (itpdof == pdof.end ());
      }

      template <bool param>
        void NumericalConstraintsAndPassiveDofs::specifyFoliation
        (LevelSetEdgePtr_t lse) const
      {
        NumericalConstraints_t::const_iterator it;
        IntervalsContainer_t::const_iterator itpdof = pdof.begin ();
        for (it = nc.begin (); it != nc.end (); ++it) {
          if (param) lse->insertParamConstraint (*it, *itpdof);
          else   lse->insertConditionConstraint (*it, *itpdof);
          ++itpdof;
        }
        assert (itpdof == pdof.end ());
      }

      void FoliatedManifold::addToNode (NodePtr_t comp) const
      {
        nc.addToComp <false> (comp);
        for (LockedJoints_t::const_iterator it = lj.begin ();
            it != lj.end (); ++it)
          comp->addLockedJointConstraint (*it);
        nc_path.addToComp <true> (comp);
      }

      void FoliatedManifold::addToEdge (EdgePtr_t comp) const
      {
        nc_fol.addToComp <false> (comp);
        for (LockedJoints_t::const_iterator it = lj_fol.begin ();
            it != lj_fol.end (); ++it)
          comp->addLockedJointConstraint (*it);
      }

      void FoliatedManifold::specifyFoliation (LevelSetEdgePtr_t lse) const
      {
        nc.specifyFoliation <false> (lse);
        for (LockedJoints_t::const_iterator it = lj.begin ();
            it != lj.end (); ++it)
          lse->insertConditionConstraint (*it);

        nc_fol.specifyFoliation <true> (lse);
        for (LockedJoints_t::const_iterator it = lj_fol.begin ();
            it != lj_fol.end (); ++it)
          lse->insertParamConstraint (*it);
      }

      Helper::WaypointEdgePair_t Helper::createWaypoints (
          const std::string& forwName,   const std::string& backName,
          const NodePtr_t& from,         const NodePtr_t& to,
          const size_type& wForw,        const size_type& wBack,
          const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
          const FoliatedManifold& place, const FoliatedManifold& preplace,
          const bool levelSetPlace,      const bool levelSetGrasp,
          const FoliatedManifold& submanifoldDef)
      {
        // Create the edges
        WaypointEdgePtr_t weForw = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
            from->linkTo (forwName,    to, wForw, WaypointEdge::create)),

                          weBack = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
            to->  linkTo (backName, from, wBack, WaypointEdge::create));

        weForw->nbWaypoints (3);
        weBack->nbWaypoints (3);

        std::string name = forwName;
        NodeSelectorPtr_t ns = weForw->parentGraph ()->nodeSelector ();
        NodePtr_t n0 = from,
                  n1 = ns->createNode (name + "_pregrasp", true),
                  n2 = ns->createNode (name + "_intersec", true),
                  n3 = ns->createNode (name + "_preplace",  true),
                  n4 = to;

        EdgePtr_t e01 = n0->linkTo (name + "_e01", n1, -1, Edge::create),
                  e12 = n1->linkTo (name + "_e12", n2, -1,
                      (levelSetGrasp?(Node::EdgeFactory)LevelSetEdge::create:Edge::create)),
                  e23 = n2->linkTo (name + "_e23", n3, -1, Edge::create),
                  e34 = weForw;

        // Set the edges properties
        e01->node (n0);
        e12->node (n0); e12->setShort (true);
        e23->node (n4); e23->setShort (true);
        e34->node (n4);

        // set the nodes constraints
        // From and to are not populated automatically to avoid duplicates.
           place.addToNode (n1);
        pregrasp.addToNode (n1);
        submanifoldDef.addToNode (n1);
           place.addToNode (n2);
           grasp.addToNode (n2);
        submanifoldDef.addToNode (n2);
        preplace.addToNode (n3);
           grasp.addToNode (n3);
        submanifoldDef.addToNode (n3);

        // Set the edges constraints
           place.addToEdge (e01);
        submanifoldDef.addToEdge (e01);
           place.addToEdge (e12);
        submanifoldDef.addToEdge (e12);
           grasp.addToEdge (e23);
        submanifoldDef.addToEdge (e23);
           grasp.addToEdge (e34);
        submanifoldDef.addToEdge (e34);

        // Set the waypoints
        weForw->setWaypoint (0, e01, n1);
        weForw->setWaypoint (1, e12, n2);
        weForw->setWaypoint (2, e23, n3);

        // Populate bacward edge
        EdgePtr_t e43 = n4->linkTo (name + "_e43", n3, -1, Edge::create),
                  e32 = n3->linkTo (name + "_e12", n2, -1,
                      (levelSetPlace?(Node::EdgeFactory)LevelSetEdge::create:Edge::create)),
                  e21 = n2->linkTo (name + "_e23", n1, -1, Edge::create),
                  e10 = weBack;

        e43->node (n4);
        e32->node (n4); e32->setShort (true);
        e21->node (n0); e21->setShort (true);
        e10->node (n0);

           place.addToEdge (e10);
        submanifoldDef.addToEdge (e10);
           place.addToEdge (e21);
        submanifoldDef.addToEdge (e21);
           grasp.addToEdge (e32);
        submanifoldDef.addToEdge (e32);
           grasp.addToEdge (e43);
        submanifoldDef.addToEdge (e43);

        weBack->setWaypoint (0, e43, n3);
        weBack->setWaypoint (1, e32, n2);
        weBack->setWaypoint (2, e21, n1);

        if (levelSetPlace && !place.isFoliated ()) {
          hppDout (warning, "You asked for a LevelSetEdge for placement, "
              "but did not specify the target foliation. "
              "It will have no effect");
        } else {
          place.specifyFoliation (HPP_DYNAMIC_PTR_CAST (LevelSetEdge, e32));
        }
        if (levelSetGrasp && !grasp.isFoliated ()) {
          hppDout (warning, "You asked for a LevelSetEdge for grasping, "
              "but did not specify the target foliation. "
              "It will have no effect");
          grasp.specifyFoliation (HPP_DYNAMIC_PTR_CAST (LevelSetEdge, e12));
        }

        return std::make_pair (weForw, weBack);
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
