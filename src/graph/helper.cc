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

#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/node-selector.hh>

namespace hpp {
  namespace manipulation {
    namespace graph {
      namespace helper {
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

        template <> EdgePair_t
          createEdges <WithPreGrasp | WithPrePlace> (
              const std::string& forwName,   const std::string& backName,
              const NodePtr_t& from,         const NodePtr_t& to,
              const size_type& wForw,        const size_type& wBack,
              const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
              const FoliatedManifold& place, const FoliatedManifold& preplace,
              const bool levelSetGrasp,      const bool levelSetPlace,
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
                      e12 = n1->linkTo (name + "_e12", n2, -1, Edge::create),
                      e23 = n2->linkTo (name + "_e23", n3, -1, Edge::create),
                      e34 = weForw;
            LevelSetEdgePtr_t e12_ls;
            if (levelSetGrasp)
              e12_ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge,
                  n1->linkTo (name + "_e12_ls", n2, -1, LevelSetEdge::create));

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
            weForw->setWaypoint (1, (levelSetGrasp)?e12_ls:e12, n2);
            weForw->setWaypoint (2, e23, n3);

            // Populate bacward edge
            EdgePtr_t e43 = n4->linkTo (name + "_e43", n3, -1, Edge::create),
                      e32 = n3->linkTo (name + "_e32", n2, -1, Edge::create),
                      e21 = n2->linkTo (name + "_e21", n1, -1, Edge::create),
                      e10 = weBack;
            LevelSetEdgePtr_t e32_ls;
            if (levelSetPlace)
              e32_ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge,
                  n3->linkTo (name + "_e32_ls", n2, -1, LevelSetEdge::create));

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
            weForw->setWaypoint (1, (levelSetGrasp)?e32_ls:e32, n2);
            weBack->setWaypoint (2, e21, n1);

            if (levelSetPlace) {
              if (!place.isFoliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for placement, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              e32_ls->node (n4);
              e32_ls->setShort (true);
              grasp.addToEdge (e32_ls);
              place.specifyFoliation (e32_ls);
              submanifoldDef.addToEdge (e32_ls);
            }
            if (levelSetGrasp) {
              if (!grasp.isFoliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              e12_ls->node (n0);
              e12_ls->setShort (true);
              place.addToEdge (e12_ls);
              grasp.specifyFoliation (e12_ls);
              submanifoldDef.addToEdge (e12_ls);
            }

            return std::make_pair (weForw, weBack);
          }

        template <> EdgePair_t
          createEdges <GraspOnly | PlaceOnly>(
            const std::string& forwName,   const std::string& backName,
            const NodePtr_t& from,         const NodePtr_t& to,
            const size_type& wForw,        const size_type& wBack,
            const FoliatedManifold& grasp, const FoliatedManifold& ,
            const FoliatedManifold& place, const FoliatedManifold& ,
            const bool levelSetGrasp,      const bool levelSetPlace,
            const FoliatedManifold& submanifoldDef)
        {
          // Create the edges
          WaypointEdgePtr_t weForw = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
              from->linkTo (forwName,    to, wForw, WaypointEdge::create)),

          weBack = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
              to->  linkTo (backName, from, wBack, WaypointEdge::create));

          weForw->nbWaypoints (1);
          weBack->nbWaypoints (1);

          std::string name = forwName;
          NodeSelectorPtr_t ns = weForw->parentGraph ()->nodeSelector ();
          NodePtr_t n0 = from,
                    n1 = ns->createNode (name + "_intersec", true),
                    n2 = to;

          EdgePtr_t e01 = n0->linkTo (name + "_e01", n1, -1, Edge::create),
                    e12 = weForw;
          LevelSetEdgePtr_t e01_ls;
          if (levelSetGrasp)
            e01_ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge,
                n0->linkTo (name + "_e01_ls", n1, -1, LevelSetEdge::create));

          // Set the edges properties
          e01->node (n0);
          e12->node (n1);

          // set the nodes constraints
          // From and to are not populated automatically to avoid duplicates.
          place.addToNode (n1);
          grasp.addToNode (n1);
          submanifoldDef.addToNode (n1);

          // Set the edges constraints
          place.addToEdge (e01);
          submanifoldDef.addToEdge (e01);
          grasp.addToEdge (e12);
          submanifoldDef.addToEdge (e12);

          // Set the waypoints
          weForw->setWaypoint (0, (levelSetGrasp)?e01_ls:e01, n1);

          // Populate bacward edge
          EdgePtr_t e21 = n2->linkTo (name + "_e21", n1, -1, Edge::create),
                    e10 = weBack;
          LevelSetEdgePtr_t e21_ls;
          if (levelSetPlace)
            e21_ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge,
                n2->linkTo (name + "_e21_ls", n1, -1, LevelSetEdge::create));

          e21->node (n2);
          e10->node (n0);

          place.addToEdge (e10);
          submanifoldDef.addToEdge (e10);
          grasp.addToEdge (e21);
          submanifoldDef.addToEdge (e21);

          weForw->setWaypoint (0, (levelSetGrasp)?e21_ls:e21, n1);

          if (levelSetPlace) {
            if (!place.isFoliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for placement, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            e21_ls->node (n2);
            e21_ls->setShort (true);
            grasp.addToEdge (e21_ls);
            place.specifyFoliation (e21_ls);
            submanifoldDef.addToEdge (e21_ls);
          }
          if (levelSetGrasp) {
            if (!grasp.isFoliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            e01_ls->node (n0);
            e01_ls->setShort (true);
            place.addToEdge (e01_ls);
            grasp.specifyFoliation (e01_ls);
            submanifoldDef.addToEdge (e01_ls);
          }

          return std::make_pair (weForw, weBack);
        }

        template <> EdgePair_t
          createEdges <WithPreGrasp | NoPlace>(
            const std::string& forwName,   const std::string& backName,
            const NodePtr_t& from,         const NodePtr_t& to,
            const size_type& wForw,        const size_type& wBack,
            const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
            const FoliatedManifold&,       const FoliatedManifold&,
            const bool levelSetGrasp,      const bool,
            const FoliatedManifold& submanifoldDef)
        {
          // Create the edges
          WaypointEdgePtr_t weForw = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
              from->linkTo (forwName,    to, wForw, WaypointEdge::create)),

          weBack = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
              to->  linkTo (backName, from, wBack, WaypointEdge::create));

          weForw->nbWaypoints (1);
          weBack->nbWaypoints (1);

          std::string name = forwName;
          NodeSelectorPtr_t ns = weForw->parentGraph ()->nodeSelector ();
          NodePtr_t n0 = from,
                    n1 = ns->createNode (name + "_pregrasp", true),
                    n2 = to;

          EdgePtr_t e01 = n0->linkTo (name + "_e01", n1, -1, Edge::create),
                    e12 = weForw;
          LevelSetEdgePtr_t e12_ls;
          if (levelSetGrasp)
            e12_ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge,
                n1->linkTo (name + "_e12_ls", n2, -1, LevelSetEdge::create));

          // Set the edges properties
          e01->node (n0);
          e12->node (n0); e12->setShort (true);

          // set the nodes constraints
          // From and to are not populated automatically to avoid duplicates.
          pregrasp.addToNode (n1);
          submanifoldDef.addToNode (n1);

          // Set the edges constraints
          submanifoldDef.addToEdge (e01);
          submanifoldDef.addToEdge (e12);

          // Set the waypoints
          weForw->setWaypoint (0, e01, n1);
          // weForw->setWaypoint (1, (levelSetGrasp)?e12_ls:e12, n2);

          // Populate bacward edge
          EdgePtr_t e21 = n2->linkTo (name + "_e21", n1, -1, Edge::create),
                    e10 = weBack;

          e21->node (n0); e21->setShort (true);
          e10->node (n0);

          submanifoldDef.addToEdge (e10);
          submanifoldDef.addToEdge (e21);

          weForw->setWaypoint (0, e21, n1);

          if (levelSetGrasp) {
            hppDout (error, "You specified a foliated grasp with no placement. "
                "This is currently unsupported.");
            if (!grasp.isFoliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            e12_ls->node (n0);
            e12_ls->setShort (true);
            grasp.specifyFoliation (e12_ls);
            submanifoldDef.addToEdge (e12_ls);
          }

          return std::make_pair (weForw, weBack);
        }


        template <> EdgePair_t
          createEdges <GraspOnly | NoPlace>(
            const std::string& forwName,   const std::string& backName,
            const NodePtr_t& from,         const NodePtr_t& to,
            const size_type& wForw,        const size_type& wBack,
            const FoliatedManifold& grasp, const FoliatedManifold&,
            const FoliatedManifold&,       const FoliatedManifold&,
            const bool levelSetGrasp,      const bool,
            const FoliatedManifold& submanifoldDef)
        {
          // Create the edges
          EdgePtr_t eForw;
          if (levelSetGrasp)
                    eForw = from->linkTo (forwName,   to, wForw, LevelSetEdge::create);
          else      eForw = from->linkTo (forwName,   to, wForw, Edge::create);
          EdgePtr_t eBack = to->  linkTo (backName, from, wBack, Edge::create);

          std::string name = forwName;

          eForw->node (from);
          submanifoldDef.addToEdge (eForw);
          eBack->node (from);
          submanifoldDef.addToEdge (eBack);

          if (levelSetGrasp) {
            if (!grasp.isFoliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            grasp.specifyFoliation (HPP_DYNAMIC_PTR_CAST (LevelSetEdge, eForw));
          }

          return std::make_pair (eForw, eBack);
        }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
