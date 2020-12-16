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

#include <iterator>
#include <array>

#include <boost/regex.hpp>

#include <pinocchio/multibody/model.hpp>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/gripper.hh>
#include <pinocchio/multibody/model.hpp>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/locked-joint.hh>

#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state-selector.hh>
#include <hpp/manipulation/graph/guided-state-selector.hh>
#include <hpp/manipulation/problem-solver.hh>

#define CASE_TO_STRING(var, value) ( (var & value) ? std::string(#value) : std::string() )

namespace hpp {
  namespace manipulation {
    namespace graph {
      namespace helper {
        typedef constraints::Implicit Implicit;
        typedef constraints::ImplicitPtr_t ImplicitPtr_t;
        template <bool forPath>
          void NumericalConstraintsAndPassiveDofs::addToComp
          (GraphComponentPtr_t comp) const
        {
          if (nc.empty ()) return;
          StatePtr_t n;
          if (forPath) {
            n = HPP_DYNAMIC_PTR_CAST (State, comp);
            if (!n) throw std::logic_error ("Wrong type: expect a State");
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

        void FoliatedManifold::addToState (StatePtr_t comp) const
        {
          nc.addToComp <false> (comp);
          nc_path.addToComp <true> (comp);
        }

        void FoliatedManifold::addToEdge (EdgePtr_t comp) const
        {
          nc_fol.addToComp <false> (comp);
        }

        void FoliatedManifold::specifyFoliation (LevelSetEdgePtr_t lse) const
        {
          nc.specifyFoliation <false> (lse);
          nc_fol.specifyFoliation <true> (lse);
        }

        namespace {
          template <int gCase>
            struct CaseTraits {
              static const bool pregrasp = (gCase & WithPreGrasp);
              static const bool preplace = (gCase & WithPrePlace);
              static const bool intersec = !((gCase & NoGrasp) || (gCase & NoPlace));

              static const bool valid =
                   ( (gCase & WithPreGrasp) || (gCase & GraspOnly) || (gCase & NoGrasp) )
                && ( (gCase & WithPrePlace) || (gCase & PlaceOnly) || (gCase & NoPlace) )
                && !((gCase & NoGrasp) && (gCase & NoPlace));

              static const std::size_t nbWaypoints = (pregrasp?1:0) + (intersec?1:0) + (preplace?1:0);
              static const std::size_t Nstates = 2 + nbWaypoints;
              static const std::size_t Nedges = 1 + nbWaypoints;
              // static const std::size_t iNpregrasp = pregrasp?1 + 1:nbWaypoints;
              // static const std::size_t iNpreplace = pregrasp?1 + 1:nbWaypoints;
              typedef std::array <StatePtr_t, Nstates> StateArray;
              typedef std::array <EdgePtr_t, Nedges> EdgeArray;

              static inline const StatePtr_t& Npregrasp (const StateArray& n) { assert (pregrasp); return n[1]; }
              static inline const StatePtr_t& Nintersec (const StateArray& n) { assert (intersec); return n[1 + (pregrasp?1:0)]; }
              static inline const StatePtr_t& Npreplace (const StateArray& n) { assert (preplace); return n[1 + (pregrasp?1:0) + (intersec?1:0)]; }

              static inline std::string caseToString ()
              {
                return CASE_TO_STRING (gCase, NoGrasp)
                  +    CASE_TO_STRING (gCase, GraspOnly)
                  +    CASE_TO_STRING (gCase, WithPreGrasp)
                  + " - "
                  +    CASE_TO_STRING (gCase, NoPlace)
                  +    CASE_TO_STRING (gCase, PlaceOnly)
                  +    CASE_TO_STRING (gCase, WithPrePlace);
              }

              static inline EdgePtr_t makeWE (
                  const std::string& name,
                  const StatePtr_t& from, const StatePtr_t& to,
                  const size_type& w)
              {
                if (Nedges > 1) {
                  WaypointEdgePtr_t we = static_pointer_cast <WaypointEdge>
                      (from->linkTo (name, to, w, WaypointEdge::create));
                  we->nbWaypoints (nbWaypoints);
                  return we;
                } else return from->linkTo (name, to, w, Edge::create);
              }

              static inline StateArray makeWaypoints (
                  const StatePtr_t& from, const StatePtr_t& to,
                  const std::string& name)
              {
                StateSelectorPtr_t ns = from->parentGraph ()->stateSelector ();
                StateArray states;
                std::size_t r = 0;
                states[r] = from; ++r;
                if (pregrasp) {
                  states[r] = ns->createState (name + "_pregrasp", true); ++r;
                }
                if (intersec) {
                  states[r] = ns->createState (name + "_intersec", true); ++r;
                }
                if (preplace) {
                  states[r] = ns->createState (name + "_preplace", true); ++r;
                }
                states[r] = to;
                return states;
              }

              static inline EdgePtr_t makeLSEgrasp (const std::string& name,
                  const StateArray& n, const EdgeArray& e,
                  const size_type w, LevelSetEdgePtr_t& gls)
              {
                if (Nedges > 1) {
                  const std::size_t T = (pregrasp?1:0) + (intersec?1:0);
                  WaypointEdgePtr_t we = static_pointer_cast <WaypointEdge>
                    (n.front()->linkTo (name + "_ls", n.back(), w,
                                        WaypointEdge::create));
                  we->nbWaypoints (nbWaypoints);
                  gls = linkWaypoint <LevelSetEdge> (n, T-1, T, name, "ls");
                  for (std::size_t i = 0; i < Nedges; ++i)
                    we->setWaypoint (i, e[i], n[i+1]);
                  we->setWaypoint (T-1, gls, n[T]);
                  gls->state (n.front());
                  gls->setShort (pregrasp);
                  return we;
                } else {
                  assert (gCase == (GraspOnly | NoPlace)
                      && "Cannot implement a LevelSetEdge for grasping");
                  gls = static_pointer_cast <LevelSetEdge>
                    (n.front()->linkTo (name + "_ls", n.back(), w,
                                        LevelSetEdge::create));
                  return gls;
                }
              }

              static inline EdgePtr_t makeLSEplace (const std::string& name,
                  const StateArray& n, const EdgeArray& e,
                  const size_type w, LevelSetEdgePtr_t& pls)
              {
                if (Nedges > 1) {
                  const std::size_t T = (pregrasp?1:0) + (intersec?1:0);
                  WaypointEdgePtr_t we = static_pointer_cast <WaypointEdge>
                    (n.back()->linkTo (name + "_ls", n.front(), w,
                                       WaypointEdge::create));
                  we->nbWaypoints (nbWaypoints);
                  pls = linkWaypoint <LevelSetEdge> (n, T+1, T, name, "ls");
                  // for (std::size_t i = Nedges - 1; i != 0; --i)
                  for (std::size_t k = 0; k < Nedges; ++k) {
                    std::size_t i = Nedges  - 1 - k;
                    we->setWaypoint (Nedges - 1 - i, e[i], n[i]);
                  }
                  we->setWaypoint (Nedges - 1 - T, pls, n[T]);
                  pls->state (n.back ());
                  pls->setShort (preplace);
                  return we;
                } else {
                  assert (gCase == (NoGrasp | PlaceOnly)
                      && "Cannot implement a LevelSetEdge for placement");
                  pls = static_pointer_cast <LevelSetEdge>
                    (n.back()->linkTo (name + "_ls", n.front(), w,
                                       LevelSetEdge::create));
                  return pls;
                }
              }

              template <typename EdgeType>
              static inline shared_ptr<EdgeType> linkWaypoint (
                  const StateArray& states,
                  const std::size_t& iF, const std::size_t& iT,
                  const std::string& prefix,
                  const std::string& suffix = "")
              {
                std::stringstream ss;
                ss << prefix << "_" << iF << iT;
                if (suffix.length () > 0) ss << "_" << suffix;
                return static_pointer_cast <EdgeType>
                    (states[iF]->linkTo (ss.str(), states[iT], -1, EdgeType::create));
              }

              template <bool forward>
              static inline EdgeArray linkWaypoints (
                  const StateArray& states, const EdgePtr_t& edge,
                  const std::string& name)
              {
                EdgeArray e;
                WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST(WaypointEdge, edge);
                if (forward)
                  for (std::size_t i = 0; i < Nedges; ++i) {
                    e[i] = linkWaypoint <Edge> (states, i, i + 1, name);
                    we->setWaypoint (i, e[i], states[i+1]);
                  }
                else
                  // for (std::size_t i = Nedges - 1; i != 0; --i) {
                  for (std::size_t k = 0; k < Nedges; ++k) {
                    std::size_t i = Nedges  - 1 - k;
                    e[i] = linkWaypoint <Edge> (states, i + 1, i, name);
                    we->setWaypoint (Nedges - 1 - i, e[i], states[i]);
                  }
                return e;
              }

              static inline void setStateConstraints (const StateArray& n,
                  const FoliatedManifold& g, const FoliatedManifold& pg,
                  const FoliatedManifold& p, const FoliatedManifold& pp,
                  const FoliatedManifold& m)
              {
                // From and to are not populated automatically
                // to avoid duplicates.
                if (pregrasp) {
                  p .addToState (Npregrasp (n));
                  pg.addToState (Npregrasp (n));
                  m .addToState (Npregrasp (n));
                }
                if (intersec) {
                  p .addToState (Nintersec (n));
                  g .addToState (Nintersec (n));
                  m .addToState (Nintersec (n));
                }
                if (preplace) {
                  pp.addToState (Npreplace (n));
                  g .addToState (Npreplace (n));
                  m .addToState (Npreplace (n));
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
              (const EdgeArray& e, const StateArray& n)
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
                for (std::size_t i = 0; i < B     ; ++i) e[i]->state (n[0]);
                for (std::size_t i = B; i < Nedges; ++i) e[i]->state (n[Nstates-1]);
              }
            };
        }

        template <int gCase> Edges_t createEdges (
              const std::string& forwName,   const std::string& backName,
              const StatePtr_t& from,         const StatePtr_t& to,
              const size_type& wForw,        const size_type& wBack,
              const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
              const FoliatedManifold& place, const FoliatedManifold& preplace,
              const bool levelSetGrasp,      const bool levelSetPlace,
              const FoliatedManifold& submanifoldDef)
          {
            typedef CaseTraits<gCase> T;
            hppDout (info, "Creating edges " << forwName << " and " << backName
               << "\ncase is " << T::caseToString ());
            assert (T::valid && "Not a valid case.");
            typedef typename T::StateArray StateArray;
            typedef typename T::EdgeArray EdgeArray;

            // Create the edges
            EdgePtr_t weForw = T::makeWE (forwName, from, to, wForw),
                      weBack = T::makeWE (backName, to, from, wBack),
                      weForwLs, weBackLs;

            std::string name = forwName;
            StateArray n = T::makeWaypoints (from, to, name);

            EdgeArray eF = T::template linkWaypoints <true> (n, weForw, name);

            // Set the states constraints
            // Note that submanifold is not taken into account for states
            // because the edges constraints will enforce configuration to stay
            // in a leaf, and so in the manifold itself.
            T::setStateConstraints (n, grasp, pregrasp, place, preplace,
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

            Edges_t ret { weForw, weBack };

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
              const StatePtr_t& state,
              const size_type& w,
              const bool levelSet,
              const FoliatedManifold& submanifoldDef)
        {
          // Create the edges
          EdgePtr_t loop;
          if (levelSet)
               loop = state->linkTo (loopName, state, w, LevelSetEdge::create);
          else loop = state->linkTo (loopName, state, w, Edge::create);

          loop->state (state);
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
          ImplicitPtr_t gc  = handle->createGrasp (gripper, "");
          grasp.nc.nc.push_back (gc);
          grasp.nc.pdof.push_back (segments_t ());
          grasp.nc_path.nc.push_back (gc);
          // TODO: see function declaration
          grasp.nc_path.pdof.push_back (segments_t ());
          ImplicitPtr_t gcc = handle->createGraspComplement
            (gripper, "");
          if (gcc->function ().outputSize () > 0) {
            grasp.nc_fol.nc.push_back (gcc);
            grasp.nc_fol.pdof.push_back (segments_t());
          }

          const value_type c = handle->clearance () + gripper->clearance ();
          ImplicitPtr_t pgc = handle->createPreGrasp (gripper, c, "");
          pregrasp.nc.nc.push_back (pgc);
          pregrasp.nc.pdof.push_back (segments_t());
          pregrasp.nc_path.nc.push_back (pgc);
          pregrasp.nc_path.pdof.push_back (segments_t());
        }

        void strictPlacementManifold (
            const ImplicitPtr_t placement,
            const ImplicitPtr_t preplacement,
            const ImplicitPtr_t placementComplement,
            FoliatedManifold& place, FoliatedManifold& preplace)
        {
          place.nc.nc.push_back (placement);
          place.nc.pdof.push_back (segments_t());
          place.nc_path.nc.push_back (placement);
          place.nc_path.pdof.push_back (segments_t());
          if (placementComplement && placementComplement->function().outputSize () > 0) {
            place.nc_fol.nc.push_back (placementComplement);
            place.nc_fol.pdof.push_back (segments_t());
          }

          preplace.nc.nc.push_back (preplacement);
          preplace.nc.pdof.push_back (segments_t());
          preplace.nc_path.nc.push_back (preplacement);
          preplace.nc_path.pdof.push_back (segments_t());
        }

        void relaxedPlacementManifold (
            const ImplicitPtr_t placement,
            const ImplicitPtr_t preplacement,
            const LockedJoints_t objectLocks,
            FoliatedManifold& place, FoliatedManifold& preplace)
        {
          if (placement) {
            place.nc.nc.push_back (placement);
            place.nc.pdof.push_back (segments_t());
            // The placement constraints are not required in the path, as long as
            // they are satisfied at both ends and the object does not move. The
            // former condition is ensured by the placement constraints on both
            // ends and the latter is ensure by the LockedJoint constraints.
            place.nc_path.nc.push_back (placement);
            place.nc_path.pdof.push_back (segments_t());
          }
          std::copy (objectLocks.begin(), objectLocks.end(), std::back_inserter(place.lj_fol));

          if (placement && preplacement) {
            preplace.nc.nc.push_back (preplacement);
            preplace.nc.pdof.push_back (segments_t());
            // preplace.nc_path.nc.push_back (preplacement);
            // preplace.nc_path.pdof.push_back (segments_t());
          }
        }

        namespace {
          typedef std::size_t index_t;
          typedef std::vector <index_t> IndexV_t;
          typedef std::list <index_t> IndexL_t;
          typedef std::pair <index_t, index_t> Grasp_t;
          typedef std::tuple <StatePtr_t, FoliatedManifold> StateAndManifold_t;
          //typedef std::vector <index_t, index_t> GraspV_t;
          /// GraspV_t corresponds to a unique ID of a  permutation.
          /// - its size is the number of grippers,
          /// - the values correpond to the index of the handle (0..nbHandle-1), or
          ///   nbHandle to mean no handle. 
          typedef std::vector <index_t> GraspV_t;
          struct Result;
          struct CompiledRule {
            enum Status {
              Accept,
              Refuse,
              NoMatch,
              Undefined
            };
            std::vector<boost::regex> handles;
            Status status;
            CompiledRule (const Result& res, const Rule& r);
            Status check (const std::vector<std::string>& names, const GraspV_t& g) const
            {
              const std::size_t nG = g.size();
              assert(nG == handles.size());
              for (std::size_t i = 0; i < nG; ++i) {
                if (handles[i].empty()) continue;
                if (!boost::regex_match(names[g[i]], handles[i]))
                  return NoMatch;
              }
              return status;
            }
          };
          typedef std::vector<CompiledRule> CompiledRules_t;

          struct Result {
            ProblemSolverPtr_t ps;
            GraphPtr_t graph;
            typedef unsigned long stateid_type;
            std::tr1::unordered_map<stateid_type, StateAndManifold_t> states;
            typedef std::pair<stateid_type, stateid_type> edgeid_type;
            struct edgeid_hash {
              std::tr1::hash<edgeid_type::first_type> first;
              std::tr1::hash<edgeid_type::second_type> second;
              std::size_t operator() (const edgeid_type& eid) const {
                return first(eid.first) + second(eid.second);
              }
            };
            std::tr1::unordered_set<edgeid_type, edgeid_hash> edges;
            std::vector< std::array<ImplicitPtr_t,3> > graspCs;
            index_t nG, nOH;
            GraspV_t dims;
            const Grippers_t& gs;
            const Objects_t& ohs;
            std::vector<std::string> handleNames;
            CompiledRules_t rules;
            CompiledRule::Status defaultAcceptationPolicy;

            Result (const ProblemSolverPtr_t problem, const Grippers_t& grippers, const Objects_t& objects, GraphPtr_t g) :
              ps (problem), graph (g), nG (grippers.size ()), nOH (0), gs (grippers), ohs (objects),
              defaultAcceptationPolicy (CompiledRule::Refuse)
            {
              for (const Object_t& o : objects) {
                nOH += std::get<1>(o).size();
                for (const HandlePtr_t& h : std::get<1>(o))
                  handleNames.push_back(h->name());
              }
              handleNames.push_back("");
              dims.resize (nG);
              dims[0] = nOH + 1;
              for (index_t i = 1; i < nG; ++i)
                dims[i] = dims[i-1] * (nOH + 1);
              graspCs.resize (nG * nOH);
            }

            void setRules (const Rules_t& r)
            {
              for (Rules_t::const_iterator _r = r.begin(); _r != r.end(); ++_r)
                rules.push_back (CompiledRule(*this, *_r));
            }

            bool graspIsAllowed (const GraspV_t& idxOH) const
            {
              assert (idxOH.size () == nG);
              for (std::size_t r = 0; r < rules.size(); ++r) {
                switch (rules[r].check(handleNames,idxOH)) {
                  case CompiledRule::Accept : return true;
                  case CompiledRule::Refuse : return false;
                  case CompiledRule::NoMatch: continue; // Check next rule
                  default: throw std::invalid_argument ("Rules are ill-defined.");
                }
              }
              return (defaultAcceptationPolicy == CompiledRule::Accept);
            }

            inline stateid_type stateid (const GraspV_t& iG)
            {
              stateid_type iGOH = iG[0];
              stateid_type res;
              for (index_t i = 1; i < nG; ++i) {
                res = iGOH + dims[i] * (iG[i]);
                if (res < iGOH) {
                  hppDout (info, "State ID overflowed. There are too many states...");
                }
                iGOH = res;
                // iGOH += dims[i] * (iG[i]);
              }
              return iGOH;
            }

            bool hasState (const GraspV_t& iG)
            {
              return states.count(stateid(iG)) > 0;
            }

            StateAndManifold_t& operator() (const GraspV_t& iG)
            {
              return states [stateid(iG)];
            }

            bool hasEdge (const GraspV_t& g1, const GraspV_t& g2)
            {
              return edges.count(edgeid_type(stateid(g1), stateid(g2))) > 0;
            }

            void addEdge (const GraspV_t& g1, const GraspV_t& g2)
            {
              edges.insert(edgeid_type(stateid(g1), stateid(g2)));
            }

            inline std::array<ImplicitPtr_t,3>& graspConstraint (
                const index_t& iG, const index_t& iOH)
            {
              std::array<ImplicitPtr_t,3>& gcs =
                graspCs [iG * nOH + iOH];
              if (!gcs[0]) {
                hppDout (info, "Create grasps constraints for ("
                    << iG << ", " << iOH << ")");
                const GripperPtr_t& g (gs[iG]);
                const HandlePtr_t& h (handle (iOH));
                const std::string& grasp = g->name() + " grasps " + h->name();
                if (!ps->numericalConstraints.has(grasp)) {
                  ps->createGraspConstraint (grasp, g->name(), h->name());
                }
                gcs[0] = ps->numericalConstraints.get(grasp);
                gcs[1] = ps->numericalConstraints.get(grasp + "/complement");
                const std::string& pregrasp = g->name() + " pregrasps " + h->name();
                if (!ps->numericalConstraints.has(pregrasp)) {
                  ps->createPreGraspConstraint (pregrasp, g->name(), h->name());
                }
                gcs[2] = ps->numericalConstraints.get(pregrasp);
              }
              return gcs;
            }

            const Object_t& object (const index_t& iOH) const {
              index_t iH = iOH;
              for (const Object_t& o : ohs) {
                if (iH < std::get<1>(o).size()) return o;
                iH -= std::get<1>(o).size();
              }
              throw std::out_of_range ("Handle index");
            }

            const HandlePtr_t& handle (const index_t& iOH) const {
              index_t iH = iOH;
              for (const Object_t& o : ohs) {
                if (iH < std::get<1>(o).size()) return std::get<1>(o)[iH];
                iH -= std::get<1>(o).size();
              }
              throw std::out_of_range ("Handle index");
            }

            /// Check if an object can be placed
            bool objectCanBePlaced (const Object_t& o) const
            {
              // If the object has no joint, then it cannot be placed.
              return (std::get<2>(std::get<0>(o)).size() > 0);
            }

            /// Check is an object is grasped by the GraspV_t
            bool isObjectGrasped (const GraspV_t& idxOH,
                const Object_t& o) const
            {
              assert (idxOH.size () == nG);
              for (std::size_t i = 0; i < idxOH.size (); ++i)
                if (idxOH[i] < nOH) // This grippers grasps an object
                  if (std::get<2>(o) == std::get<2>(object(idxOH[i])))
                    return true;
              return false;
            }

            /// Get a state name from a set of grasps
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
              std::array<ImplicitPtr_t,3>& gcs
                = graspConstraint (iG, iOH);
              grasp.nc.nc.push_back (gcs[0]);
              grasp.nc.pdof.push_back (segments_t ());
              grasp.nc_path.nc.push_back (gcs[0]);
              // TODO: see function declaration
              grasp.nc_path.pdof.push_back (segments_t ());
              if (gcs[1]->function ().outputSize () > 0) {
                grasp.nc_fol.nc.push_back (gcs[1]);
                grasp.nc_fol.pdof.push_back (segments_t());
              }

              pregrasp.nc.nc.push_back (gcs[2]);
              pregrasp.nc.pdof.push_back (segments_t());
              pregrasp.nc_path.nc.push_back (gcs[2]);
              pregrasp.nc_path.pdof.push_back (segments_t());
            }
          };

          CompiledRule::CompiledRule (const Result& res, const Rule& r) :
            handles(res.nG), status (r.link_ ? Accept : Refuse)
          {
            assert(r.grippers_.size() == r.handles_.size());
            for (std::size_t j = 0; j < r.grippers_.size(); ++j) {
              boost::regex gripper (r.grippers_[j]);
              for (std::size_t i = 0; i < res.nG; ++i) {
                if (boost::regex_match(res.gs[i]->name(), gripper)) {
                  assert(handles[i].empty() && "Two gripper regex match the different gripper names.");
                  handles[i] = r.handles_[j];
                }
              }
            }
          }

          const StateAndManifold_t& makeState (Result& r, const GraspV_t& g,
              const int priority)
          {
            StateAndManifold_t& nam = r (g);
            if (!std::get<0>(nam)) {
              hppDout (info, "Creating state " << r.name (g));
              std::get<0>(nam) = r.graph->stateSelector ()->createState
                (r.name (g), false, priority);
              // Loop over the grippers and create grasping constraints if required
              FoliatedManifold unused;
              std::set <index_t> idxsOH;
              for (index_t i = 0; i < r.nG; ++i) {
                if (g[i] < r.nOH) {
                  idxsOH.insert (g[i]);
                  r.graspManifold (i, g[i], std::get<1>(nam), unused);
                }
              }
              index_t iOH = 0;
              for (const Object_t& o : r.ohs) {
                if (!r.objectCanBePlaced(o)) continue;
                bool oIsGrasped = false;
                // TODO: use the fact that the set is sorted.
                // for (const HandlePtr_t& h : std::get<0>(o))
                for (index_t i = 0; i < std::get<1>(o).size(); ++i) {
                  if (idxsOH.erase (iOH) == 1) oIsGrasped = true;
                  ++iOH;
                }
                if (!oIsGrasped) {
                  const auto& pc (std::get<0>(o));
                  relaxedPlacementManifold (std::get<0>(pc),
                      std::get<1>(pc),
                      std::get<2>(pc),
                      std::get<1>(nam), unused);
                }
              }
              std::get<1>(nam).addToState (std::get<0>(nam));

              createLoopEdge (r.nameLoopEdge (g),
                  std::get<0>(nam), 0,
                  false,
                  // TODO std::get<1>(nam).foliated(),
                  std::get<1>(nam));
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
            const StateAndManifold_t& from = makeState (r, gFrom, priority),
                                     to   = makeState (r, gTo, priority+1);
            const Object_t& o = r.object (gTo[iG]);

            // Detect when grasping an object already grasped.
            // or when there is no placement information for it.
            bool noPlace = !r.objectCanBePlaced(o)
                         || r.isObjectGrasped (gFrom, o);

            FoliatedManifold grasp, pregrasp, place, preplace,
                             submanifold;
            r.graspManifold (iG, gTo[iG], grasp, pregrasp);
            if (!noPlace) {
              const auto& pc (std::get<0>(o));
              relaxedPlacementManifold (std::get<0>(pc),
                  std::get<1>(pc),
                  std::get<2>(pc),
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
              for (const Object_t& o : r.ohs) {
                if (!r.objectCanBePlaced(o)) continue;
                bool oIsGrasped = false;
                const index_t iOHstart = iOH;
                for (; iOH < iOHstart + std::get<1>(o).size(); ++iOH) {
                  if (iOH == gTo [iG]) {
                    oIsGrasped = true;
                    iOH = iOHstart + std::get<1>(o).size();
                    break;
                  }
                  if (idxsOH.erase (iOH) == 1) oIsGrasped = true;
                }
                if (!oIsGrasped) {
                  const auto& pc (std::get<0>(o));
                  relaxedPlacementManifold (std::get<0>(pc),
                      std::get<1>(pc),
                      std::get<2>(pc),
                      submanifold, unused);
                }
              }
            }
            if (pregrasp.empty ()) {
              if (noPlace)
                createEdges <GraspOnly | NoPlace> (
                    names.first           , names.second,
                    std::get<0>(from)     , std::get<0>(to),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
              else if (preplace.empty ())
                createEdges <GraspOnly | PlaceOnly> (
                    names.first           , names.second,
                    std::get<0>(from)     , std::get<0>(to),
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
                   std::get<0>(from)     , std::get<0>(to),
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
                    std::get<0>(from)     , std::get<0>(to),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
              else if (preplace.empty ())
                createEdges <WithPreGrasp | PlaceOnly> (
                    names.first           , names.second,
                    std::get<0>(from)     , std::get<0>(to),
                    1                     , 1,
                    grasp                 , pregrasp,
                    place                 , preplace,
                    grasp.foliated ()     , place.foliated(),
                    submanifold);
              else
                createEdges <WithPreGrasp | WithPrePlace> (
                    names.first           , names.second,
                    std::get<0>(from)     , std::get<0>(to),
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
            bool curGraspIsAllowed = r.graspIsAllowed(grasps);
            if (curGraspIsAllowed) makeState (r, grasps, depth);

            if (idx_g.empty () || idx_oh.empty ()) return;
            IndexV_t nIdx_g (idx_g.size() - 1);
            IndexV_t nIdx_oh (idx_oh.size() - 1);
            for (IndexV_t::const_iterator itx_g = idx_g.begin ();
                itx_g != idx_g.end (); ++itx_g) {
              // Copy all element except itx_g
              std::copy (std::next (itx_g), idx_g.end (),
                  std::copy (idx_g.begin (), itx_g, nIdx_g.begin ())
                  );
              for (IndexV_t::const_iterator itx_oh = idx_oh.begin ();
                  itx_oh != idx_oh.end (); ++itx_oh) {
                // Create the edge for the selected grasp
                GraspV_t nGrasps = grasps;
                nGrasps [*itx_g] = *itx_oh;

                bool nextGraspIsAllowed = r.graspIsAllowed(nGrasps);
                if (nextGraspIsAllowed) makeState (r, nGrasps, depth + 1);

                if (curGraspIsAllowed && nextGraspIsAllowed)
                  makeEdge (r, grasps, nGrasps, *itx_g, depth);

                // Copy all element except itx_oh
                std::copy (std::next (itx_oh), idx_oh.end (),
                    std::copy (idx_oh.begin (), itx_oh, nIdx_oh.begin ())
                    );
                // Do all the possible combination below this new grasp
                recurseGrippers (r, nIdx_g, nIdx_oh, nGrasps, depth + 2);
              }
            }
          }
        }

        void graphBuilder (
            const ProblemSolverPtr_t& ps,
            const Objects_t& objects,
            const Grippers_t& grippers,
            GraphPtr_t graph,
            const Rules_t& rules)
        {
          if (!graph) throw std::logic_error ("The graph must be initialized");
          StateSelectorPtr_t ns = graph->stateSelector ();
          if (!ns) throw std::logic_error ("The graph does not have a StateSelector");

          Result r (ps, grippers, objects, graph);
          r.setRules (rules);

          IndexV_t availG (r.nG), availOH (r.nOH);
          for (index_t i = 0; i < r.nG; ++i) availG[i] = i;
          for (index_t i = 0; i < r.nOH; ++i) availOH[i] = i;

          GraspV_t iG (r.nG, r.nOH);

          recurseGrippers (r, availG, availOH, iG, 0);

          hppDout (info, "Created a graph with " << r.states.size() << " states "
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
          if (ps->graphs.has (graphName))
            throw std::invalid_argument ("A graph named " + graphName + " already exists.");

          const Device& robot = *(ps->robot ());
          const pinocchio::Model& model = robot.model();
          Grippers_t grippers (griNames.size());
          index_t i = 0;
          for (const std::string& gn : griNames) {
            grippers[i] = robot.grippers.get (gn);
            ++i;
          }
          Objects_t objects (objs.size());
          i = 0;
          const value_type margin = 1e-3;
          bool prePlace = (prePlaceWidth > 0);
          for (const ObjectDef_t& od : objs) {
            // Create handles
            std::get<2>(objects[i]) = i;
            std::get<1>(objects[i]).resize (od.handles.size());
            Handles_t::iterator it = std::get<1>(objects[i]).begin();
            for (const std::string hn : od.handles) {
              *it = robot.handles.get (hn);
              ++it;
            }
            // Create placement
            const std::string placeN = "place_" + od.name;
            const std::string preplaceN = "pre" + placeN;
            // If user provides constraint "place_objectName",
            // then
            //   use this as placement and use "preplace_objectName" for
            //   pre-placement if defined.
            // else if contact surfaces are defined and selected
            //   create default placement constraint using the ProblemSolver
            //   methods createPlacementConstraint and createPrePlacementConstraint
            auto& pc (std::get<0>(objects[i]));
            if (ps->numericalConstraints.has(placeN)) {
              std::get<0>(pc) = ps->numericalConstraints.get (placeN);
              if (ps->numericalConstraints.has(preplaceN)) {
                std::get<1>(pc) = ps->numericalConstraints.get (preplaceN);
              }
            } else if (!envNames.empty() && !od.shapes.empty ()) {
              ps->createPlacementConstraint (placeN,
                  od.shapes, envNames, margin);
              std::get<0>(pc) = ps->numericalConstraints.get (placeN);
              if (prePlace) {
                ps->createPrePlacementConstraint (preplaceN,
                    od.shapes, envNames, margin, prePlaceWidth);
                std::get<1>(pc) = ps->numericalConstraints.get (preplaceN);
              }
            }
            // Create object lock
	    // Loop over all frames of object, detect joint and create locked
	    // joint.
            assert (robot.robotFrames (od.name).size () != 0);
            for (const FrameIndex& f : robot.robotFrames (od.name)) {
              if (model.frames[f].type != ::pinocchio::JOINT) continue;
              const JointIndex j = model.frames[f].parent;
              JointPtr_t oj (Joint::create (ps->robot(), j));
              LiegroupSpacePtr_t space (oj->configurationSpace ());
              LiegroupElement lge (robot.currentConfiguration()
                                   .segment (oj->rankInConfiguration (),
                                             oj->configSize ()), space);
              LockedJointPtr_t lj = core::LockedJoint::create (oj, lge);
              ps->numericalConstraints.add ("lock_" + oj->name (), lj);
              std::get<2>(pc).push_back (lj);
            }
            ++i;
          }
          GraphPtr_t graph = Graph::create (graphName,
              ps->robot(), ps->problem());
          ps->graphs.add (graphName, graph);
          ps->constraintGraph (graphName);
          graph->stateSelector (
              GuidedStateSelector::create ("stateSelector",
              ps->roadmap ()));
          graph->maxIterations  (ps->maxIterProjection ());
          graph->errorThreshold (ps->errorThreshold ());

          graphBuilder (ps, objects, grippers, graph, rules);
          return graph;
        }
      } // namespace helper
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
