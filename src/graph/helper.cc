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

#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

#include <hpp/util/debug.hh>

#include <hpp/model/gripper.hh>

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/node-selector.hh>
#include <hpp/manipulation/problem-solver.hh>

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

        namespace {
          inline WaypointEdgePtr_t makeWE (
              const std::string& name,
              const NodePtr_t& from, const NodePtr_t& to,
              const size_type& w,    const std::size_t& nbWaypoints)
          {
            WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
                from->linkTo (name, to, w, WaypointEdge::create));
            we->nbWaypoints (nbWaypoints);
            return we;
          }

          template <bool pregrasp, bool intersec, bool preplace>
            struct node_array {
              static const std::size_t size = 1 + (pregrasp?1:0) + (intersec?1:0) + (preplace?1:0) + 1;
              typedef boost::array <NodePtr_t, size> Type;
              };

          template <bool pregrasp, bool intersec, bool preplace>
          inline typename node_array<pregrasp, intersec, preplace>::Type
          makeNodes (NodeSelectorPtr_t ns,
              const NodePtr_t& from, const NodePtr_t& to,
              const std::string& name)
          {
            const std::size_t N = 1 + (pregrasp?1:0) + (intersec?1:0) + (preplace?1:0) + 1;
            boost::array <NodePtr_t, N> nodes;
            std::size_t r = 0;
            nodes[r] = from; ++r;
            if (pregrasp) {
              nodes[r] = ns->createNode (name + "_pregrasp", true);
              ++r;
            }
            if (intersec) {
              nodes[r] = ns->createNode (name + "_intersec", true);
              ++r;
            }
            if (preplace) {
              nodes[r] = ns->createNode (name + "_preplace", true);
              ++r;
            }
            nodes[r] = to;
            return nodes;
          }

          template <typename EdgeType, std::size_t N>
          boost::shared_ptr<EdgeType> makeE (
              const boost::array <NodePtr_t, N>& nodes,
              const std::size_t& iF, const std::size_t& iT,
              const std::string& prefix,
              const size_type& w = -1,
              const std::string& suffix = "")
          {
            std::stringstream ss;
            ss << prefix << "_" << iF << iT;
            if (suffix.length () > 0) ss << "_" << suffix;
            return HPP_DYNAMIC_PTR_CAST (EdgeType,
              nodes[iF]->linkTo (ss.str(), nodes[iT], w, EdgeType::create));
          }
        }

        template <> Edges_t
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
            WaypointEdgePtr_t weForw = makeWE (forwName, from, to, wForw, 3),
                              weBack = makeWE (backName, to, from, wBack, 3),
                              weForwLs, weBackLs;

            if (levelSetGrasp)
              weForwLs = makeWE (forwName + "_ls", from, to, 10*wForw, 3);
            if (levelSetPlace)
              weBackLs = makeWE (backName + "_ls", to, from, 10*wBack, 3);

            std::string name = forwName;
            NodeSelectorPtr_t ns = weForw->parentGraph ()->nodeSelector ();
            boost::array <NodePtr_t, 5> n = makeNodes <true, true, true>
              (weForw->parentGraph ()->nodeSelector (), from, to, name);

            EdgePtr_t e01 = makeE <Edge> (n, 0, 1, name, -1),
                      e12 = makeE <Edge> (n, 1, 2, name, -1),
                      e23 = makeE <Edge> (n, 2, 3, name, -1),
                      e34 = weForw;
            LevelSetEdgePtr_t e12_ls;
            if (levelSetGrasp)
              e12_ls = makeE <LevelSetEdge> (n, 1, 2, name, -1, "ls");

            // Set the edges properties
            e01->node (n[0]);
            e12->node (n[0]); e12->setShort (true);
            e23->node (n[4]); e23->setShort (true);
            e34->node (n[4]);

            // set the nodes constraints
            // From and to are not populated automatically to avoid duplicates.
            place.addToNode (n[1]);
            pregrasp.addToNode (n[1]);
            // submanifoldDef.addToNode (n[1]);
            place.addToNode (n[2]);
            grasp.addToNode (n[2]);
            // submanifoldDef.addToNode (n[2]);
            preplace.addToNode (n[3]);
            grasp.addToNode (n[3]);
            // submanifoldDef.addToNode (n[3]);

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
            weForw->setWaypoint (0, e01, n[1]);
            weForw->setWaypoint (1, e12, n[2]);
            weForw->setWaypoint (2, e23, n[3]);

            // Populate bacward edge
            name = backName;
            EdgePtr_t e43 = makeE <Edge> (n, 4, 3, name, -1),
                      e32 = makeE <Edge> (n, 3, 2, name, -1),
                      e21 = makeE <Edge> (n, 2, 1, name, -1),
                      e10 = weBack;
            LevelSetEdgePtr_t e32_ls;
            if (levelSetPlace)
              e32_ls = makeE <LevelSetEdge> (n, 3, 2, name, -1, "ls");

            e43->node (n[4]);
            e32->node (n[4]); e32->setShort (true);
            e21->node (n[0]); e21->setShort (true);
            e10->node (n[0]);

            place.addToEdge (e10);
            submanifoldDef.addToEdge (e10);
            place.addToEdge (e21);
            submanifoldDef.addToEdge (e21);
            grasp.addToEdge (e32);
            submanifoldDef.addToEdge (e32);
            grasp.addToEdge (e43);
            submanifoldDef.addToEdge (e43);

            weBack->setWaypoint (0, e43, n[3]);
            weBack->setWaypoint (1, e32, n[2]);
            weBack->setWaypoint (2, e21, n[1]);

            Edges_t ret = boost::assign::list_of (weForw)(weBack);

            if (levelSetPlace) {
              if (!place.foliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for placement, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              e32_ls->node (n[4]);
              e32_ls->setShort (true);
              grasp.addToEdge (e32_ls);
              place.specifyFoliation (e32_ls);
              submanifoldDef.addToEdge (e32_ls);
              e32_ls->buildHistogram ();
              weBackLs->setWaypoint (0, e43   , n[3]);
              weBackLs->setWaypoint (1, e32_ls, n[2]);
              weBackLs->setWaypoint (2, e21   , n[1]);
              ret.push_back (weBackLs);
            }
            if (levelSetGrasp) {
              if (!grasp.foliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              e12_ls->node (n[0]);
              e12_ls->setShort (true);
              place.addToEdge (e12_ls);
              grasp.specifyFoliation (e12_ls);
              submanifoldDef.addToEdge (e12_ls);
              e12_ls->buildHistogram ();
              weForwLs->setWaypoint (0, e01   , n[1]);
              weForwLs->setWaypoint (1, e12_ls, n[2]);
              weForwLs->setWaypoint (2, e23   , n[3]);
              ret.push_back (weForwLs);
            }

            return ret;
          }

        template <> Edges_t
          createEdges <WithPreGrasp | PlaceOnly> (
              const std::string& forwName,   const std::string& backName,
              const NodePtr_t& from,         const NodePtr_t& to,
              const size_type& wForw,        const size_type& wBack,
              const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
              const FoliatedManifold& place, const FoliatedManifold&,
              const bool levelSetGrasp,      const bool levelSetPlace,
              const FoliatedManifold& submanifoldDef)
          {
            // Create the edges
            WaypointEdgePtr_t weForw = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
                from->linkTo (forwName,    to, wForw, WaypointEdge::create)),

            weBack = HPP_DYNAMIC_PTR_CAST (WaypointEdge,
                to->  linkTo (backName, from, wBack, WaypointEdge::create));

            weForw->nbWaypoints (2);
            weBack->nbWaypoints (2);

            std::string name = forwName;
            NodeSelectorPtr_t ns = weForw->parentGraph ()->nodeSelector ();
            NodePtr_t n0 = from,
                      n1 = ns->createNode (name + "_pregrasp", true),
                      n2 = ns->createNode (name + "_intersec", true),
                      n3 = to;

            EdgePtr_t e01 = n0->linkTo (name + "_e01", n1, -1, Edge::create),
                      e12 = n1->linkTo (name + "_e12", n2, -1, Edge::create),
                      e23 = weForw;
            LevelSetEdgePtr_t e12_ls;
            if (levelSetGrasp)
              e12_ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge,
                  n1->linkTo (name + "_e12_ls", n2, -1, LevelSetEdge::create));

            // Set the edges properties
            e01->node (n0);
            e12->node (n0); e12->setShort (true);
            e23->node (n3);

            // set the nodes constraints
            // From and to are not populated automatically to avoid duplicates.
            place.addToNode (n1);
            pregrasp.addToNode (n1);
            submanifoldDef.addToNode (n1);
            place.addToNode (n2);
            grasp.addToNode (n2);
            submanifoldDef.addToNode (n2);

            // Set the edges constraints
            place.addToEdge (e01);
            submanifoldDef.addToEdge (e01);
            place.addToEdge (e12);
            submanifoldDef.addToEdge (e12);
            grasp.addToEdge (e23);
            submanifoldDef.addToEdge (e23);

            // Set the waypoints
            weForw->setWaypoint (0, e01, n1);
            weForw->setWaypoint (1, (levelSetGrasp)?e12_ls:e12, n2);

            // Populate bacward edge
            name = backName;
            EdgePtr_t e32 = n3->linkTo (name + "_e32", n2, -1, Edge::create),
                      e21 = n2->linkTo (name + "_e21", n1, -1, Edge::create),
                      e10 = weBack;
            LevelSetEdgePtr_t e32_ls;
            if (levelSetPlace)
              e32_ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge,
                  n3->linkTo (name + "_e32_ls", n2, -1, LevelSetEdge::create));

            e32->node (n3);
            e21->node (n0); e21->setShort (true);
            e10->node (n0);

            place.addToEdge (e10);
            submanifoldDef.addToEdge (e10);
            place.addToEdge (e21);
            submanifoldDef.addToEdge (e21);
            grasp.addToEdge (e32);
            submanifoldDef.addToEdge (e32);

            weBack->setWaypoint (0, (levelSetPlace)?e32_ls:e32, n2);
            weBack->setWaypoint (1, e21, n1);

            if (levelSetPlace) {
              if (!place.foliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for placement, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              e32_ls->node (n3);
              e32_ls->setShort (true);
              grasp.addToEdge (e32_ls);
              place.specifyFoliation (e32_ls);
              submanifoldDef.addToEdge (e32_ls);
              e32_ls->buildHistogram ();
            }
            if (levelSetGrasp) {
              if (!grasp.foliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              e12_ls->node (n0);
              e12_ls->setShort (true);
              place.addToEdge (e12_ls);
              grasp.specifyFoliation (e12_ls);
              submanifoldDef.addToEdge (e12_ls);
              e12_ls->buildHistogram ();
            }

            return boost::assign::list_of (weForw)(weBack);
          }

        template <> Edges_t
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
          name = backName;
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

          weBack->setWaypoint (0, (levelSetPlace)?e21_ls:e21, n1);

          if (levelSetPlace) {
            if (!place.foliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for placement, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            e21_ls->node (n2);
            e21_ls->setShort (true);
            grasp.addToEdge (e21_ls);
            place.specifyFoliation (e21_ls);
            submanifoldDef.addToEdge (e21_ls);
            e21_ls->buildHistogram ();
          }
          if (levelSetGrasp) {
            if (!grasp.foliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            e01_ls->node (n0);
            e01_ls->setShort (true);
            place.addToEdge (e01_ls);
            grasp.specifyFoliation (e01_ls);
            submanifoldDef.addToEdge (e01_ls);
            e01_ls->buildHistogram ();
          }

          return boost::assign::list_of (weForw)(weBack);
        }

        template <> Edges_t
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
          name = backName;
          EdgePtr_t e21 = n2->linkTo (name + "_e21", n1, -1, Edge::create),
                    e10 = weBack;

          e21->node (n0); e21->setShort (true);
          e10->node (n0);

          submanifoldDef.addToEdge (e10);
          submanifoldDef.addToEdge (e21);

          weBack->setWaypoint (0, e21, n1);

          if (levelSetGrasp) {
            hppDout (error, "You specified a foliated grasp with no placement. "
                "This is currently unsupported.");
            if (!grasp.foliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            e12_ls->node (n0);
            e12_ls->setShort (true);
            grasp.specifyFoliation (e12_ls);
            submanifoldDef.addToEdge (e12_ls);
            e12_ls->buildHistogram ();
          }

          return boost::assign::list_of (weForw)(weBack);
        }

        template <> Edges_t
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
            if (!grasp.foliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            LevelSetEdgePtr_t ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge, eForw);
            grasp.specifyFoliation (ls);
            ls->buildHistogram ();
          }

          return boost::assign::list_of (eForw)(eBack);
        }

        EdgePtr_t createLoopEdge (
              const std::string& loopName,
              const NodePtr_t& node,
              const size_type& w,
              const bool levelSet,
              const FoliatedManifold& submanifoldDef)
        {
          // Create the edges
          EdgePtr_t loop;
          if (levelSet)
               loop = node->linkTo (loopName, node, w, LevelSetEdge::create);
          else loop = node->linkTo (loopName, node, w, Edge::create);

          loop->node (node);
          submanifoldDef.addToEdge (loop);

          if (levelSet) {
            if (!submanifoldDef.foliated ()) {
              hppDout (warning, "You asked for a LevelSetEdge for looping, "
                  "but did not specify the target foliation. "
                  "It will have no effect");
            }
            LevelSetEdgePtr_t ls = HPP_DYNAMIC_PTR_CAST (LevelSetEdge, loop);
            submanifoldDef.specifyFoliation (ls);
            ls->buildHistogram ();
          }

          return loop;
        }

        void graspManifold (
            const GripperPtr_t& gripper, const HandlePtr_t& handle,
            FoliatedManifold& grasp, FoliatedManifold& pregrasp)
        {
          NumericalConstraintPtr_t gc  = handle->createGrasp (gripper);
          grasp.nc.nc.push_back (gc);
          grasp.nc.pdof.push_back (SizeIntervals_t ());
          grasp.nc_path.nc.push_back (gc);
          // TODO: see function declaration
          grasp.nc_path.pdof.push_back (SizeIntervals_t ());
          NumericalConstraintPtr_t gcc = handle->createGraspComplement (gripper);
          if (gcc->function ().outputSize () > 0) {
            grasp.nc_fol.nc.push_back (gcc);
            grasp.nc_fol.pdof.push_back (SizeIntervals_t());
          }

          const value_type c = handle->clearance () + gripper->clearance ();
          NumericalConstraintPtr_t pgc = handle->createPreGrasp (gripper, c);
          pregrasp.nc.nc.push_back (pgc);
          pregrasp.nc.pdof.push_back (SizeIntervals_t());
          pregrasp.nc_path.nc.push_back (pgc);
          pregrasp.nc_path.pdof.push_back (SizeIntervals_t());
        }

        void strictPlacementManifold (
            const NumericalConstraintPtr_t placement,
            const NumericalConstraintPtr_t preplacement,
            const NumericalConstraintPtr_t placementComplement,
            FoliatedManifold& place, FoliatedManifold& preplace)
        {
          place.nc.nc.push_back (placement);
          place.nc.pdof.push_back (SizeIntervals_t());
          place.nc_path.nc.push_back (placement);
          place.nc_path.pdof.push_back (SizeIntervals_t());
          if (placementComplement && placementComplement->function().outputSize () > 0) {
            place.nc_fol.nc.push_back (placementComplement);
            place.nc_fol.pdof.push_back (SizeIntervals_t());
          }

          preplace.nc.nc.push_back (preplacement);
          preplace.nc.pdof.push_back (SizeIntervals_t());
          preplace.nc_path.nc.push_back (preplacement);
          preplace.nc_path.pdof.push_back (SizeIntervals_t());
        }

        void relaxedPlacementManifold (
            const NumericalConstraintPtr_t placement,
            const NumericalConstraintPtr_t preplacement,
            const LockedJoints_t objectLocks,
            FoliatedManifold& place, FoliatedManifold& preplace)
        {
          place.nc.nc.push_back (placement);
          place.nc.pdof.push_back (SizeIntervals_t());
          place.nc_path.nc.push_back (placement);
          place.nc_path.pdof.push_back (SizeIntervals_t());
          std::copy (objectLocks.begin(), objectLocks.end(), std::back_inserter(place.lj_fol));

          preplace.nc.nc.push_back (preplacement);
          preplace.nc.pdof.push_back (SizeIntervals_t());
          preplace.nc_path.nc.push_back (preplacement);
          preplace.nc_path.pdof.push_back (SizeIntervals_t());
        }

        namespace {
          typedef std::size_t index_t;
          typedef std::vector <index_t> IndexV_t;
          typedef std::list <index_t> IndexL_t;
          typedef std::pair <index_t, index_t> Grasp_t;
          typedef boost::tuple <NodePtr_t,
                  FoliatedManifold>
                    NodeAndManifold_t;
          //typedef std::vector <index_t, index_t> GraspV_t;
          /// GraspV_t corresponds to a unique ID of a  permutation.
          /// - its size is the number of grippers,
          /// - the values correpond to the index of the handle (0..nbHandle-1), or
          ///   nbHandle to mean no handle. 
          typedef std::vector <index_t> GraspV_t;

          struct Result {
            GraphPtr_t graph;
            std::vector<NodeAndManifold_t> nodes;
            std::vector< boost::array<NumericalConstraintPtr_t,3> > graspCs;
            index_t nG, nOH;
            GraspV_t dims;
            const Grippers_t& gs;
            const Objects_t& ohs;

            Result (const Grippers_t& grippers, const Objects_t& objects, GraphPtr_t g) :
              graph (g), nG (grippers.size ()), nOH (0), gs (grippers), ohs (objects)
            {
              BOOST_FOREACH (const Object_t& o, objects) {
                nOH += o.get<1>().size();
              }
              dims.resize (nG);
              dims[0] = nOH + 1;
              for (index_t i = 1; i < nG; ++i)
                dims[i] = dims[i-1] * (nOH + 1);
              nodes.resize (dims[nG-1] * (nOH + 1));
              graspCs.resize (nG * nOH);
            }

            NodeAndManifold_t& operator() (const GraspV_t& iG)
            {
              index_t iGOH = iG[0];
              for (index_t i = 1; i < nG; ++i)
                iGOH += dims[i] * (iG[i]);
              return nodes [iGOH];
            }

            inline boost::array<NumericalConstraintPtr_t,3>& graspConstraint (
                const index_t& iG, const index_t& iOH)
            {
              boost::array<NumericalConstraintPtr_t,3>& gcs =
                graspCs [iG * nOH + iOH];
              if (!gcs[0]) {
                hppDout (info, "Create grasps constraints for ("
                    << iG << ", " << iOH << ")");
                const GripperPtr_t& g (gs[iG]);
                const HandlePtr_t& h (handle (iOH));
                gcs[0] = h->createGrasp (g);
                gcs[1] = h->createGraspComplement (g);
                const value_type c = h->clearance () + g->clearance ();
                gcs[2] = h->createPreGrasp (g, c);
              }
              return gcs;
            }

            const Object_t& object (const index_t& iOH) const {
              index_t iH = iOH;
              BOOST_FOREACH (const Object_t& o, ohs) {
                if (iH < o.get<1>().size()) return o;
                iH -= o.get<1>().size();
              }
              throw std::out_of_range ("Handle index");
            }

            const HandlePtr_t& handle (const index_t& iOH) const {
              index_t iH = iOH;
              BOOST_FOREACH (const Object_t& o, ohs) {
                if (iH < o.get<1>().size()) return o.get<1>()[iH];
                iH -= o.get<1>().size();
              }
              throw std::out_of_range ("Handle index");
            }

            /// Get a node name from a set of grasps
            std::string name (const GraspV_t& idxOH, bool abbrev = false) const {
              assert (idxOH.size () == nG);
              std::stringstream ss;
              bool first = true;
              std::string sepGOH = (abbrev?"-":" grasps "),
                          sep    = (abbrev?":":" : ");
              for (std::size_t i = 0; i < idxOH.size (); ++i) {
                if (idxOH[i] < nOH) { // This grippers grasps an object
                  if (first) first = false; else ss << sep; 
                  if (abbrev) ss << i << sepGOH << idxOH[i];
                  else
                    ss << gs[i]->name() << sepGOH << handle (idxOH[i])->name ();
                }
              }
              if (first) return (abbrev?"f":"free");
              return ss.str();
            }

            /// Get an edge name from a set of grasps
            std::pair<std::string, std::string> name
              (const GraspV_t& gFrom, const GraspV_t& gTo, const index_t iG)
            {
              const std::string nf (name (gFrom, true)),
                                nt (name (gTo, true));
              std::stringstream ssForw, ssBack;
              const char sep[] = " | ";
              ssForw << gs[iG]->name() << " > " << handle (gTo[iG])->name () << sep << nf;
              ssBack << gs[iG]->name() << " < " << handle (gTo[iG])->name () << sep << nt;
              return std::make_pair (ssForw.str(), ssBack.str ());
            }

            std::string nameLoopEdge (const GraspV_t& gFrom)
            {
              const std::string nf (name (gFrom, true));
              std::stringstream ss;
              const char sep[] = " | ";
              ss << "Loop" << sep << nf;
              return ss.str();
            }

            void graspManifold (const index_t& iG, const index_t& iOH,
                FoliatedManifold& grasp, FoliatedManifold& pregrasp)
            {
              boost::array<NumericalConstraintPtr_t,3>& gcs
                = graspConstraint (iG, iOH);
              grasp.nc.nc.push_back (gcs[0]);
              grasp.nc.pdof.push_back (SizeIntervals_t ());
              grasp.nc_path.nc.push_back (gcs[0]);
              // TODO: see function declaration
              grasp.nc_path.pdof.push_back (SizeIntervals_t ());
              if (gcs[1]->function ().outputSize () > 0) {
                grasp.nc_fol.nc.push_back (gcs[1]);
                grasp.nc_fol.pdof.push_back (SizeIntervals_t());
              }

              pregrasp.nc.nc.push_back (gcs[2]);
              pregrasp.nc.pdof.push_back (SizeIntervals_t());
              pregrasp.nc_path.nc.push_back (gcs[2]);
              pregrasp.nc_path.pdof.push_back (SizeIntervals_t());
            }
          };

          const NodeAndManifold_t& makeNode (Result& r, const GraspV_t& g)
          {
            NodeAndManifold_t& nam = r (g);
            if (!nam.get<0>()) {
              hppDout (info, "Creating node " << r.name (g));
              nam.get<0>() = r.graph->nodeSelector ()->createNode (r.name (g));
              // Loop over the grippers and create grasping constraints if required
              FoliatedManifold unused;
              std::set <index_t> idxsOH;
              for (index_t i = 0; i < r.nG; ++i) {
                if (g[i] < r.nOH) {
                  idxsOH.insert (g[i]);
                  r.graspManifold (i, g[i], nam.get<1>(), unused);
                }
              }
              index_t iOH = 0;
              BOOST_FOREACH (const Object_t& o, r.ohs) {
                bool oIsGrasped = false;
                // TODO: use the fact that the set is sorted.
                // BOOST_FOREACH (const HandlePtr_t& h, o.get<1>())
                for (index_t i = 0; i < o.get<1>().size(); ++i) {
                  if (idxsOH.erase (iOH) == 1) oIsGrasped = true;
                  ++iOH;
                }
                if (!oIsGrasped)
                  relaxedPlacementManifold (o.get<0>().get<0>(),
                      o.get<0>().get<1>(),
                      o.get<0>().get<2>(),
                      nam.get<1>(), unused);
              }
              nam.get<1>().addToNode (nam.get<0>());

              createLoopEdge (r.nameLoopEdge (g),
                  nam.get<0>(), 1, 
                  false,
                  // TODO nam.get<1>().foliated(),
                  nam.get<1>());
            }
            return nam;
          }

          /// Arguments are such that
          /// \li gTo[iG] != gFrom[iG]
          /// \li for all i != iG, gTo[iG] == gFrom[iG]
          void makeEdge (Result& r,
              const GraspV_t& gFrom, const GraspV_t& gTo,
              const index_t iG)
          {
            const NodeAndManifold_t& from = makeNode (r, gFrom),
            to   = makeNode (r, gTo);
            FoliatedManifold grasp, pregrasp, place, preplace,
                             submanifold;
            r.graspManifold (iG, gTo[iG], grasp, pregrasp);
            const Object_t& o = r.object (gTo[iG]);
            relaxedPlacementManifold (o.get<0>().get<0>(),
                o.get<0>().get<1>(),
                o.get<0>().get<2>(),
                place, preplace);
            std::pair<std::string, std::string> names =
              r.name (gFrom, gTo, iG);
            {
              FoliatedManifold unused;
              std::set <index_t> idxsOH;
              for (index_t i = 0; i < r.nG; ++i) {
                if (gFrom[i] < r.nOH) {
                  idxsOH.insert (gFrom[i]);
                  r.graspManifold (i, gFrom[i], submanifold, unused);
                }
              }
              index_t iOH = 0;
              BOOST_FOREACH (const Object_t& o, r.ohs) {
                bool oIsGrasped = false;
                const index_t iOHstart = iOH;
                for (; iOH < iOHstart + o.get<1>().size(); ++iOH) {
                  if (iOH == gTo [iG]) {
                    oIsGrasped = true;
                    iOH = iOHstart + o.get<1>().size();
                    break;
                  }
                  if (idxsOH.erase (iOH) == 1) oIsGrasped = true;
                }
                if (!oIsGrasped)
                  relaxedPlacementManifold (o.get<0>().get<0>(),
                      o.get<0>().get<1>(),
                      o.get<0>().get<2>(),
                      submanifold, unused);
              }
            }
            if (pregrasp.empty ()) {
              if (preplace.empty ())
                createEdges <GraspOnly | PlaceOnly> (
                    names.first           , names.second,
                    from.get<0> ()        , to.get<0>(),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
              else {
                hppDout (error, "GraspOnly | WithPrePlace not implemeted yet");
                /*
                   createEdges <GraspOnly | WithPrePlace> (
                   names.first           , names.second,
                   from.get<0> ()        , to.get<0>(),
                   1                     , 1,
                   grasp                 , pregrasp,
                   place                 , preplace,
                   grasp.foliated ()     , place.foliated(),
                   submanifold); // */
              }
            } else {
              if (preplace.empty ())
                createEdges <WithPreGrasp | PlaceOnly> (
                    names.first           , names.second,
                    from.get<0> ()        , to.get<0>(),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
              else
                createEdges <WithPreGrasp | WithPrePlace> (
                    names.first           , names.second,
                    from.get<0> ()        , to.get<0>(),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
            }
          }

          /// idx are the available grippers
          void recurseGrippers (Result& r,
              const IndexV_t& idx_g, const IndexV_t& idx_oh,
              const GraspV_t& grasps)
          {
            if (idx_g.empty () || idx_oh.empty ()) return;
            IndexV_t nIdx_g (idx_g.size() - 1);
            IndexV_t nIdx_oh (idx_oh.size() - 1);
            for (IndexV_t::const_iterator itx_g = idx_g.begin ();
                itx_g != idx_g.end (); ++itx_g) {
              // Copy all element except itx_g
              std::copy (boost::next (itx_g), idx_g.end (),
                  std::copy (idx_g.begin (), itx_g, nIdx_g.begin ())
                  );
              for (IndexV_t::const_iterator itx_oh = idx_oh.begin ();
                  itx_oh != idx_oh.end (); ++itx_oh) {
                // Copy all element except itx_oh
                std::copy (boost::next (itx_oh), idx_oh.end (),
                    std::copy (idx_oh.begin (), itx_oh, nIdx_oh.begin ())
                    );

                // Create the edge for the selected grasp
                GraspV_t nGrasps = grasps;
                nGrasps [*itx_g] = *itx_oh;
                makeEdge (r, grasps, nGrasps, *itx_g);

                // Do all the possible combination below this new grasp
                recurseGrippers (r, nIdx_g, nIdx_oh, nGrasps);
              }
            }
          }
        }

        void graphBuilder (
            const Objects_t& objects,
            const Grippers_t& grippers,
            GraphPtr_t graph)
        {
          if (!graph) throw std::logic_error ("The graph must be initialized");
          NodeSelectorPtr_t ns = graph->nodeSelector ();
          if (!ns) throw std::logic_error ("The graph does not have a NodeSelector");

          Result r (grippers, objects, graph);

          IndexV_t availG (r.nG), availOH (r.nOH);
          for (index_t i = 0; i < r.nG; ++i) availG[i] = i;
          for (index_t i = 0; i < r.nOH; ++i) availOH[i] = i;

          GraspV_t iG (r.nG, r.nOH);

          recurseGrippers (r, availG, availOH, iG);
        }

        GraphPtr_t graphBuilder (
            const ProblemSolverPtr_t& ps,
            const std::string& graphName,
            const StringList_t& griNames,
            const std::list <ObjectDef_t>& objs,
            const StringList_t& envNames,
            const value_type& prePlaceWidth)
        {
          const Device& robot = *(ps->robot ());
          Grippers_t grippers (griNames.size());
          index_t i = 0;
          BOOST_FOREACH (const std::string& gn, griNames) {
            grippers[i] = robot.get <GripperPtr_t> (gn);
            ++i;
          }
          Objects_t objects (objs.size());
          i = 0;
          const value_type margin = 1e-3;
          bool prePlace = (prePlaceWidth > 0);
          BOOST_FOREACH (const ObjectDef_t& od, objs) {
            // Create handles
            objects[i].get<1> ().resize (od.handles.size());
            Handles_t::iterator it = objects[i].get<1> ().begin();
            BOOST_FOREACH (const std::string hn, od.handles) {
              *it = robot.get <HandlePtr_t> (hn);
              ++it;
            }
            // Create placement
            if (!od.shapes.empty ()) {
              const std::string placeN = "place_" + od.name;
              ps->createPlacementConstraint (placeN,
                  od.shapes, envNames, margin);
              objects[i].get<0> ().get<0> () =
                ps->get <NumericalConstraintPtr_t> (placeN);
              if (prePlace) {
                ps->createPrePlacementConstraint ("pre" + placeN,
                    od.shapes, envNames, margin, prePlaceWidth);
                objects[i].get<0> ().get<1> () =
                  ps->get <NumericalConstraintPtr_t> ("pre" + placeN);
              }
            }
            // Create object lock
            using model::JointVector_t;
            assert (robot.has <JointVector_t> (od.name));
            BOOST_FOREACH (const JointPtr_t& oj, robot.get<JointVector_t> (od.name)) {
              LockedJointPtr_t lj = core::LockedJoint::create (oj,
                  robot.currentConfiguration()
                  .segment (oj->rankInConfiguration (), oj->configSize ()));
              ps->add <LockedJointPtr_t> ("lock_" + oj->name (), lj);
              objects[i].get<0> ().get<2> ().push_back (lj);
            }
            ++i;
          }
          GraphPtr_t graph = Graph::create (graphName,
              ps->robot(), ps->problem());
          graph->createNodeSelector ("nodeSelector");
          graph->maxIterations  (ps->maxIterations ());
          graph->errorThreshold (ps->errorThreshold ());

          graphBuilder (objects, grippers, graph);
          ps->constraintGraph (graph);
          return graph;
        }
      } // namespace helper
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
