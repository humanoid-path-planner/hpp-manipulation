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

#include <tr1/unordered_map>
#include <tr1/unordered_set>

#include <boost/array.hpp>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

#include <hpp/util/debug.hh>

#include <hpp/model/gripper.hh>

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/node-selector.hh>
#include <hpp/manipulation/graph/guided-node-selector.hh>
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
            if (*it) {
              if (forPath) n->addNumericalConstraintForPath (*it, *itpdof);
              else      comp->addNumericalConstraint (*it, *itpdof);
            }
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
            if (*it) {
              if (param) lse->insertParamConstraint (*it, *itpdof);
              else   lse->insertConditionConstraint (*it, *itpdof);
            }
            ++itpdof;
          }
          assert (itpdof == pdof.end ());
        }

        void FoliatedManifold::addToNode (NodePtr_t comp) const
        {
          nc.addToComp <false> (comp);
          for (LockedJoints_t::const_iterator it = lj.begin ();
              it != lj.end (); ++it)
            if (*it && (*it)->numberDof() > 0)
              comp->addLockedJointConstraint (*it);
          nc_path.addToComp <true> (comp);
        }

        void FoliatedManifold::addToEdge (EdgePtr_t comp) const
        {
          nc_fol.addToComp <false> (comp);
          for (LockedJoints_t::const_iterator it = lj_fol.begin ();
              it != lj_fol.end (); ++it)
            if (*it && (*it)->numberDof() > 0)
              comp->addLockedJointConstraint (*it);
        }

        void FoliatedManifold::specifyFoliation (LevelSetEdgePtr_t lse) const
        {
          nc.specifyFoliation <false> (lse);
          for (LockedJoints_t::const_iterator it = lj.begin ();
              it != lj.end (); ++it)
            if (*it && (*it)->numberDof() > 0)
              lse->insertConditionConstraint (*it);

          nc_fol.specifyFoliation <true> (lse);
          for (LockedJoints_t::const_iterator it = lj_fol.begin ();
              it != lj_fol.end (); ++it)
            lse->insertParamConstraint (*it);
        }

        namespace {
          template <int gCase>
            struct CaseTraits {
              static const bool pregrasp = (gCase & WithPreGrasp);
              static const bool preplace = (gCase & WithPrePlace);
              /// FIXME
              // It should be
              // static const bool intersec = !((gCase & NoGrasp) || (gCase & NoPlace));
              // but when NoPlace | WithPreGrasp, we need a LevelSetEdge after
              // the pregrasp waypoint node. Sadly the current implementation of
              // WaypointEdge does not allow the last edge of type other than Edge.
              static const bool intersec = pregrasp || preplace || ((gCase & GraspOnly) && (gCase & PlaceOnly));

              static const bool valid =
                   ( (gCase & WithPreGrasp) || (gCase & GraspOnly) || (gCase & NoGrasp) )
                && ( (gCase & WithPrePlace) || (gCase & PlaceOnly) || (gCase & NoPlace) )
                && !((gCase & NoGrasp) && (gCase & NoPlace));

              static const std::size_t nbWaypoints = (pregrasp?1:0) + (intersec?1:0) + (preplace?1:0);
              static const std::size_t Nnodes = 2 + nbWaypoints;
              static const std::size_t Nedges = 1 + nbWaypoints;
              // static const std::size_t iNpregrasp = pregrasp?1 + 1:nbWaypoints;
              // static const std::size_t iNpreplace = pregrasp?1 + 1:nbWaypoints;
              typedef boost::array <NodePtr_t, Nnodes> NodeArray;
              typedef boost::array <EdgePtr_t, Nedges> EdgeArray;

              static inline const NodePtr_t& Npregrasp (const NodeArray& n) { assert (pregrasp); return n[1]; }
              static inline const NodePtr_t& Nintersec (const NodeArray& n) { assert (intersec); return n[1 + (pregrasp?1:0)]; }
              static inline const NodePtr_t& Npreplace (const NodeArray& n) { assert (preplace); return n[1 + (pregrasp?1:0) + (intersec?1:0)]; }

              static inline EdgePtr_t makeWE (
                  const std::string& name,
                  const NodePtr_t& from, const NodePtr_t& to,
                  const size_type& w)
              {
                if (Nedges > 1) {
                  WaypointEdgePtr_t we = boost::static_pointer_cast <WaypointEdge>
                      (from->linkTo (name, to, w, WaypointEdge::create));
                  we->nbWaypoints (nbWaypoints);
                  return we;
                } else return from->linkTo (name, to, w, Edge::create);
              }

              static inline NodeArray makeWaypoints (
                  const NodePtr_t& from, const NodePtr_t& to,
                  const std::string& name)
              {
                NodeSelectorPtr_t ns = from->parentGraph ()->nodeSelector ();
                NodeArray nodes;
                std::size_t r = 0;
                nodes[r] = from; ++r;
                if (pregrasp) {
                  nodes[r] = ns->createNode (name + "_pregrasp", true); ++r;
                }
                if (intersec) {
                  nodes[r] = ns->createNode (name + "_intersec", true); ++r;
                }
                if (preplace) {
                  nodes[r] = ns->createNode (name + "_preplace", true); ++r;
                }
                nodes[r] = to;
                return nodes;
              }

              static inline EdgePtr_t makeLSEgrasp (const std::string& name,
                  const NodeArray& n, const EdgeArray& e,
                  const size_type w, LevelSetEdgePtr_t& gls)
              {
                if (Nedges > 1) {
                  const std::size_t T = (pregrasp?1:0) + (intersec?1:0);
                  WaypointEdgePtr_t we = boost::static_pointer_cast <WaypointEdge>
                    (n.front()->linkTo (name + "_ls", n.back(), w,
                                        WaypointEdge::create));
                  we->nbWaypoints (nbWaypoints);
                  gls = linkWaypoint <LevelSetEdge> (n, T-1, T, name, "ls");
                  for (std::size_t i = 0; i < Nedges - 1; ++i)
                    we->setWaypoint (i, e[i], n[i]);
                  we->setWaypoint (T-1, gls, n[T]);
                  gls->node (n.front());
                  gls->setShort (pregrasp);
                  return we;
                } else {
                  assert (gCase == (GraspOnly | NoPlace)
                      && "Cannot implement a LevelSetEdge for grasping");
                  gls = boost::static_pointer_cast <LevelSetEdge>
                    (n.front()->linkTo (name + "_ls", n.back(), w,
                                        LevelSetEdge::create));
                  return gls;
                }
              }

              static inline EdgePtr_t makeLSEplace (const std::string& name,
                  const NodeArray& n, const EdgeArray& e,
                  const size_type w, LevelSetEdgePtr_t& pls)
              {
                if (Nedges > 1) {
                  const std::size_t T = (pregrasp?1:0) + (intersec?1:0);
                  WaypointEdgePtr_t we = boost::static_pointer_cast <WaypointEdge>
                    (n.back()->linkTo (name + "_ls", n.front(), w,
                                       WaypointEdge::create));
                  we->nbWaypoints (nbWaypoints);
                  pls = linkWaypoint <LevelSetEdge> (n, T+1, T, name, "ls");
                  for (std::size_t i = Nedges - 1; i != 0; --i)
                    we->setWaypoint (Nedges - 1 - i, e[i], n[i]);
                  we->setWaypoint (Nedges - 1 - T, pls, n[T]);
                  pls->node (n.back ());
                  pls->setShort (preplace);
                  return we;
                } else {
                  assert (gCase == (NoGrasp | PlaceOnly)
                      && "Cannot implement a LevelSetEdge for placement");
                  pls = boost::static_pointer_cast <LevelSetEdge>
                    (n.back()->linkTo (name + "_ls", n.front(), w,
                                       LevelSetEdge::create));
                  return pls;
                }
              }

              template <typename EdgeType>
              static inline boost::shared_ptr<EdgeType> linkWaypoint (
                  const NodeArray& nodes,
                  const std::size_t& iF, const std::size_t& iT,
                  const std::string& prefix,
                  const std::string& suffix = "")
              {
                std::stringstream ss;
                ss << prefix << "_" << iF << iT;
                if (suffix.length () > 0) ss << "_" << suffix;
                return boost::static_pointer_cast <EdgeType>
                    (nodes[iF]->linkTo (ss.str(), nodes[iT], -1, EdgeType::create));
              }

              template <bool forward>
              static inline EdgeArray linkWaypoints (
                  const NodeArray& nodes, const EdgePtr_t& edge,
                  const std::string& name)
              {
                EdgeArray e;
                WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST(WaypointEdge, edge);
                if (forward)
                  for (std::size_t i = 0; i < Nedges - 1; ++i) {
                    e[i] = linkWaypoint <Edge> (nodes, i, i + 1, name);
                    we->setWaypoint (i, e[i], nodes[i+1]);
                  }
                else
                  for (std::size_t i = Nedges - 1; i != 0; --i) {
                    e[i] = linkWaypoint <Edge> (nodes, i + 1, i, name);
                    we->setWaypoint (Nedges - 1 - i, e[i], nodes[i]);
                  }
                e[(forward?Nedges - 1:0)] = we;
                return e;
              }

              static inline void setNodeConstraints (const NodeArray& n,
                  const FoliatedManifold& g, const FoliatedManifold& pg,
                  const FoliatedManifold& p, const FoliatedManifold& pp,
                  const FoliatedManifold& m)
              {
                // From and to are not populated automatically
                // to avoid duplicates.
                if (pregrasp) {
                  p .addToNode (Npregrasp (n));
                  pg.addToNode (Npregrasp (n));
                  m .addToNode (Npregrasp (n));
                }
                if (intersec) {
                  p .addToNode (Nintersec (n));
                  g .addToNode (Nintersec (n));
                  m .addToNode (Nintersec (n));
                }
                if (preplace) {
                  pp.addToNode (Npreplace (n));
                  g .addToNode (Npreplace (n));
                  m .addToNode (Npreplace (n));
                }
              }

              static inline void setEdgeConstraints (const EdgeArray& e,
                  const FoliatedManifold& g, const FoliatedManifold& p,
                  const FoliatedManifold& m)
              {
                // The border B
                const std::size_t B = (pregrasp?1:0) + (intersec?1:0);
                for (std::size_t i = 0; i < B     ; ++i) p.addToEdge (e[i]);
                for (std::size_t i = B; i < Nedges; ++i) g.addToEdge (e[i]);
                for (std::size_t i = 0; i < Nedges; ++i) m.addToEdge (e[i]);
              }

              template <bool forward>
              static inline void setEdgeProp
              (const EdgeArray& e, const NodeArray& n)
              {
                /// Last is short
                const std::size_t K = (forward?1:0);
                for (std::size_t i = K; i < Nedges - 1 + K; ++i)
                  e[i]->setShort (true);
                // The border B
                std::size_t B;
                if ((gCase & NoGrasp)) // There is no grasp
                  B = 0;
                else // There is a grasp
                  B = 1 + (pregrasp?1:0);
                for (std::size_t i = 0; i < B     ; ++i) e[i]->node (n[0]);
                for (std::size_t i = B; i < Nedges; ++i) e[i]->node (n[Nnodes-1]);
              }
            };
        }

        template <int gCase> Edges_t createEdges (
              const std::string& forwName,   const std::string& backName,
              const NodePtr_t& from,         const NodePtr_t& to,
              const size_type& wForw,        const size_type& wBack,
              const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
              const FoliatedManifold& place, const FoliatedManifold& preplace,
              const bool levelSetGrasp,      const bool levelSetPlace,
              const FoliatedManifold& submanifoldDef)
          {
            typedef CaseTraits<gCase> T;
            assert (T::valid && "Not a valid case.");
            typedef typename T::NodeArray NodeArray;
            typedef typename T::EdgeArray EdgeArray;

            // Create the edges
            EdgePtr_t weForw = T::makeWE (forwName, from, to, wForw),
                      weBack = T::makeWE (backName, to, from, wBack),
                      weForwLs, weBackLs;

            std::string name = forwName;
            NodeArray n = T::makeWaypoints (from, to, name);

            EdgeArray eF = T::template linkWaypoints <true> (n, weForw, name);

            // Set the nodes constraints
            // Note that submanifold is not taken into account for nodes
            // because the edges constraints will enforce configuration to stay
            // in a leaf, and so in the manifold itself.
            T::setNodeConstraints (n, grasp, pregrasp, place, preplace,
                submanifoldDef);

            // Set the edges properties
            T::template setEdgeProp <true> (eF, n);

            // Set the edges constraints
            T::setEdgeConstraints (eF, grasp, place, submanifoldDef);

            LevelSetEdgePtr_t gls;
            if (levelSetGrasp)
              weForwLs = T::makeLSEgrasp (name, n, eF, 10*wForw, gls);

            // Populate bacward edge
            name = backName;
            EdgeArray eB = T::template linkWaypoints <false> (n, weBack, name);

            T::template setEdgeProp <false> (eB, n);

            T::setEdgeConstraints (eB, grasp, place, submanifoldDef);

            LevelSetEdgePtr_t pls;
            if (levelSetPlace)
              weBackLs = T::makeLSEplace (name, n, eB, 10*wBack, pls);

            Edges_t ret = boost::assign::list_of (weForw)(weBack);

            if (levelSetPlace) {
              if (!place.foliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for placement, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              grasp.addToEdge (pls);
              place.specifyFoliation (pls);
              submanifoldDef.addToEdge (pls);
              pls->buildHistogram ();
              place.addToEdge (weBackLs);
              submanifoldDef.addToEdge (weBackLs);
              ret.push_back (weBackLs);
            }
            if (levelSetGrasp) {
              if (!grasp.foliated ()) {
                hppDout (warning, "You asked for a LevelSetEdge for grasping, "
                    "but did not specify the target foliation. "
                    "It will have no effect");
              }
              place.addToEdge (gls);
              grasp.specifyFoliation (gls);
              submanifoldDef.addToEdge (gls);
              gls->buildHistogram ();
              grasp.addToEdge (weForwLs);
              submanifoldDef.addToEdge (weForwLs);
              ret.push_back (weForwLs);
            }

            return ret;
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
          if (placement) {
            place.nc.nc.push_back (placement);
            place.nc.pdof.push_back (SizeIntervals_t());
            // The placement constraints are not required in the path, as long as
            // they are satisfied at both ends and the object does not move. The
            // former condition is ensured by the placement constraints on both
            // ends and the latter is ensure by the LockedJoint constraints.
            place.nc_path.nc.push_back (placement);
            place.nc_path.pdof.push_back (SizeIntervals_t());
          }
          std::copy (objectLocks.begin(), objectLocks.end(), std::back_inserter(place.lj_fol));

          if (placement && preplacement) {
            preplace.nc.nc.push_back (preplacement);
            preplace.nc.pdof.push_back (SizeIntervals_t());
            // preplace.nc_path.nc.push_back (preplacement);
            // preplace.nc_path.pdof.push_back (SizeIntervals_t());
          }
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
          struct CompiledRule {
            enum Result {
              Accept,
              Refuse,
              NoMatch,
              Undefined
            };
            boost::regex gripper, handle;
            bool link;
            CompiledRule (const Rule& r) :
              gripper (r.gripper_), handle (r.handle_), link (r.link_) {}
            Result check (const std::string& g, const std::string& h) const
            {
              if (boost::regex_match(g, gripper))
                if (boost::regex_match(h, handle))
                  return (link ? Accept : Refuse);
              return NoMatch;
            }
          };
          typedef std::vector<CompiledRule> CompiledRules_t;

          struct Result {
            GraphPtr_t graph;
            typedef unsigned long nodeid_type;
            std::tr1::unordered_map<nodeid_type, NodeAndManifold_t> nodes;
            typedef std::pair<nodeid_type, nodeid_type> edgeid_type;
            struct edgeid_hash {
              std::tr1::hash<edgeid_type::first_type> first;
              std::tr1::hash<edgeid_type::second_type> second;
              std::size_t operator() (const edgeid_type& eid) const {
                return first(eid.first) + second(eid.second);
              }
            };
            std::tr1::unordered_set<edgeid_type, edgeid_hash> edges;
            std::vector< boost::array<NumericalConstraintPtr_t,3> > graspCs;
            index_t nG, nOH;
            GraspV_t dims;
            const Grippers_t& gs;
            const Objects_t& ohs;
            CompiledRules_t rules;
            mutable Eigen::MatrixXi rulesCache;

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
              graspCs.resize (nG * nOH);
              rulesCache = Eigen::MatrixXi::Constant(nG, nOH + 1, CompiledRule::Undefined);
            }

            void setRules (const Rules_t& r)
            {
              for (Rules_t::const_iterator _r = r.begin(); _r != r.end(); ++_r)
                rules.push_back (CompiledRule(*_r));
            }

            bool graspIsAllowed (const GraspV_t& idxOH) const
            {
              assert (idxOH.size () == nG);
              for (std::size_t i = 0; i < nG; ++i) {
                const std::string& g = gs[i]->name(),
                                   h = (idxOH[i] == nOH) ? "" : handle (idxOH[i])->name ();
                if ((CompiledRule::Result)rulesCache(i, idxOH[i]) == CompiledRule::Undefined) {
                  CompiledRule::Result status = CompiledRule::Accept;
                  for (std::size_t r = 0; r < rules.size(); ++r) {
                    status = rules[r].check(g,h);
                    if (status == CompiledRule::Accept) break;
                    else if (status == CompiledRule::Refuse) break;
                    status = CompiledRule::Accept;
                  }
                  rulesCache(i, idxOH[i]) = status;
                }
                bool keep = ((CompiledRule::Result)rulesCache(i, idxOH[i]) == CompiledRule::Accept);
                if (!keep) return false;
              }
              return true;
            }

            inline nodeid_type nodeid (const GraspV_t& iG)
            {
              nodeid_type iGOH = iG[0];
              nodeid_type res;
              for (index_t i = 1; i < nG; ++i) {
                res = iGOH + dims[i] * (iG[i]);
                if (res < iGOH) {
                  hppDout (info, "Node ID overflowed. There are too many states...");
                }
                iGOH = res;
                // iGOH += dims[i] * (iG[i]);
              }
              return iGOH;
            }

            bool hasNode (const GraspV_t& iG)
            {
              return nodes.count(nodeid(iG)) > 0;
            }

            NodeAndManifold_t& operator() (const GraspV_t& iG)
            {
              return nodes [nodeid(iG)];
            }

            bool hasEdge (const GraspV_t& g1, const GraspV_t& g2)
            {
              return edges.count(edgeid_type(nodeid(g1), nodeid(g2))) > 0;
            }

            void addEdge (const GraspV_t& g1, const GraspV_t& g2)
            {
              edges.insert(edgeid_type(nodeid(g1), nodeid(g2)));
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

            /// Check if an object can be placed
            bool objectCanBePlaced (const Object_t& o) const
            {
              return o.get<0>().get<0>();
            }

            /// Check is an object is grasped by the GraspV_t
            bool isObjectGrasped (const GraspV_t& idxOH,
                const Object_t& o) const
            {
              assert (idxOH.size () == nG);
              for (std::size_t i = 0; i < idxOH.size (); ++i)
                if (idxOH[i] < nOH) // This grippers grasps an object
                  if (o.get<2>() == object(idxOH[i]).get<2>())
                    return true;
              return false;
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

          const NodeAndManifold_t& makeNode (Result& r, const GraspV_t& g,
              const int priority)
          {
            NodeAndManifold_t& nam = r (g);
            if (!nam.get<0>()) {
              hppDout (info, "Creating node " << r.name (g));
              nam.get<0>() = r.graph->nodeSelector ()->createNode
                (r.name (g), false, priority);
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
                if (!r.objectCanBePlaced(o)) continue;
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
                  nam.get<0>(), 0, 
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
              const index_t iG, const int priority)
          {
            if (r.hasEdge(gFrom, gTo)) {
              hppDout (warning, "Prevented creation of duplicated edge\nfrom "
                  << r.name (gFrom) << "\nto " << r.name (gTo));
              return;
            }
            const NodeAndManifold_t& from = makeNode (r, gFrom, priority),
                                     to   = makeNode (r, gTo, priority+1);
            const Object_t& o = r.object (gTo[iG]);

            // Detect when grasping an object already grasped.
            // or when there is no placement information for it.
            bool noPlace = !r.objectCanBePlaced(o)
                         || r.isObjectGrasped (gFrom, o);

            FoliatedManifold grasp, pregrasp, place, preplace,
                             submanifold;
            r.graspManifold (iG, gTo[iG], grasp, pregrasp);
            if (!noPlace) {
              relaxedPlacementManifold (o.get<0>().get<0>(),
                  o.get<0>().get<1>(),
                  o.get<0>().get<2>(),
                  place, preplace);
            }
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
                if (!r.objectCanBePlaced(o)) continue;
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
              if (noPlace)
                createEdges <GraspOnly | NoPlace> (
                    names.first           , names.second,
                    from.get<0> ()        , to.get<0>(),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
              else if (preplace.empty ())
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
              if (noPlace)
                createEdges <WithPreGrasp | NoPlace> (
                    names.first           , names.second,
                    from.get<0> ()        , to.get<0>(),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
              else if (preplace.empty ())
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
            r.addEdge(gFrom, gTo);
          }

          /// idx are the available grippers
          void recurseGrippers (Result& r,
              const IndexV_t& idx_g, const IndexV_t& idx_oh,
              const GraspV_t& grasps, const int depth)
          {
            if (idx_g.empty () || idx_oh.empty ()) return;
            IndexV_t nIdx_g (idx_g.size() - 1);
            IndexV_t nIdx_oh (idx_oh.size() - 1);
            bool curGraspIsAllowed = r.graspIsAllowed(grasps);
            if (curGraspIsAllowed) makeNode (r, grasps, depth);

            for (IndexV_t::const_iterator itx_g = idx_g.begin ();
                itx_g != idx_g.end (); ++itx_g) {
              // Copy all element except itx_g
              std::copy (boost::next (itx_g), idx_g.end (),
                  std::copy (idx_g.begin (), itx_g, nIdx_g.begin ())
                  );
              for (IndexV_t::const_iterator itx_oh = idx_oh.begin ();
                  itx_oh != idx_oh.end (); ++itx_oh) {
                // Create the edge for the selected grasp
                GraspV_t nGrasps = grasps;
                nGrasps [*itx_g] = *itx_oh;

                bool nextGraspIsAllowed = r.graspIsAllowed(nGrasps);
                if (nextGraspIsAllowed) makeNode (r, nGrasps, depth + 1);

                if (curGraspIsAllowed && nextGraspIsAllowed)
                  makeEdge (r, grasps, nGrasps, *itx_g, depth);

                // Copy all element except itx_oh
                std::copy (boost::next (itx_oh), idx_oh.end (),
                    std::copy (idx_oh.begin (), itx_oh, nIdx_oh.begin ())
                    );
                // Do all the possible combination below this new grasp
                recurseGrippers (r, nIdx_g, nIdx_oh, nGrasps, depth + 2);
              }
            }
          }
        }

        void graphBuilder (
            const Objects_t& objects,
            const Grippers_t& grippers,
            GraphPtr_t graph,
            const Rules_t& rules)
        {
          if (!graph) throw std::logic_error ("The graph must be initialized");
          NodeSelectorPtr_t ns = graph->nodeSelector ();
          if (!ns) throw std::logic_error ("The graph does not have a NodeSelector");

          Result r (grippers, objects, graph);
          r.setRules (rules);

          IndexV_t availG (r.nG), availOH (r.nOH);
          for (index_t i = 0; i < r.nG; ++i) availG[i] = i;
          for (index_t i = 0; i < r.nOH; ++i) availOH[i] = i;

          GraspV_t iG (r.nG, r.nOH);

          recurseGrippers (r, availG, availOH, iG, 0);

          hppDout (info, "Created a graph with " << r.nodes.size() << " states "
              "and " << r.edges.size() << " edges.");
        }

        GraphPtr_t graphBuilder (
            const ProblemSolverPtr_t& ps,
            const std::string& graphName,
            const StringList_t& griNames,
            const std::list <ObjectDef_t>& objs,
            const StringList_t& envNames,
	    const std::vector <Rule>& rules,
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
            objects[i].get<2> () = i;
            objects[i].get<1> ().resize (od.handles.size());
            Handles_t::iterator it = objects[i].get<1> ().begin();
            BOOST_FOREACH (const std::string hn, od.handles) {
              *it = robot.get <HandlePtr_t> (hn);
              ++it;
            }
            // Create placement
            if (!envNames.empty() && !od.shapes.empty ()) {
              const std::string placeN = "place_" + od.name;
              ps->createPlacementConstraint (placeN,
                  od.shapes, envNames, margin);
              objects[i].get<0> ().get<0> () =
                ps->core::ProblemSolver::get <NumericalConstraintPtr_t> (placeN);
              if (prePlace) {
                ps->createPrePlacementConstraint ("pre" + placeN,
                    od.shapes, envNames, margin, prePlaceWidth);
                objects[i].get<0> ().get<1> () =
                  ps->core::ProblemSolver::get <NumericalConstraintPtr_t> ("pre" + placeN);
              }
            }
            // Create object lock
            using model::JointVector_t;
            assert (robot.has <JointVector_t> (od.name));
            BOOST_FOREACH (const JointPtr_t& oj, robot.get<JointVector_t> (od.name)) {
              LockedJointPtr_t lj = core::LockedJoint::create (oj,
                  robot.currentConfiguration()
                  .segment (oj->rankInConfiguration (), oj->configSize ()));
              ps->ProblemSolver::ThisC_t::add <LockedJointPtr_t> ("lock_" + oj->name (), lj);
              objects[i].get<0> ().get<2> ().push_back (lj);
            }
            ++i;
          }
          GraphPtr_t graph = Graph::create (graphName,
              ps->robot(), ps->problem());
          graph->nodeSelector (
              GuidedNodeSelector::create ("nodeSelector",
              ps->roadmap ()));
          graph->maxIterations  (ps->maxIterations ());
          graph->errorThreshold (ps->errorThreshold ());

          graphBuilder (objects, grippers, graph, rules);
          ps->constraintGraph (graph);
          return graph;
        }
      } // namespace helper
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
