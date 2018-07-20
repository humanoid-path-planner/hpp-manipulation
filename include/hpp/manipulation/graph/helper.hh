// Copyright (c) 2016, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_GRAPH_HELPER_HH
# define HPP_MANIPULATION_GRAPH_HELPER_HH

# include <string>
# include <algorithm>

# include <boost/tuple/tuple.hpp>

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    namespace graph {
      namespace helper {
        /// \defgroup helper Helpers to build the graph of constraints
        /// \addtogroup helper
        /// \{

        struct NumericalConstraintsAndPassiveDofs {
          NumericalConstraints_t nc;
          IntervalsContainer_t pdof;
          NumericalConstraintsAndPassiveDofs merge
            (const NumericalConstraintsAndPassiveDofs& other) {
              NumericalConstraintsAndPassiveDofs ret;
              // ret.nc.reserve (nc.size() + other.nc.size());
              ret.pdof.reserve (pdof.size() + other.pdof.size());

              std::copy (nc.begin(), nc.end(), ret.nc.begin());
              std::copy (other.nc.begin(), other.nc.end(), ret.nc.begin());

              std::copy (pdof.begin(), pdof.end(), ret.pdof.begin());
              std::copy (other.pdof.begin(), other.pdof.end(), ret.pdof.begin());
              return ret;
            }

          template <bool forPath> void addToComp (GraphComponentPtr_t comp) const;

          template <bool param> void specifyFoliation (LevelSetEdgePtr_t lse) const;
        };

        struct FoliatedManifold {
          // Manifold definition
          NumericalConstraintsAndPassiveDofs nc;
          LockedJoints_t lj;
          NumericalConstraintsAndPassiveDofs nc_path;
          // Foliation definition
          NumericalConstraintsAndPassiveDofs nc_fol;
          LockedJoints_t lj_fol;

          FoliatedManifold merge (const FoliatedManifold& other) {
            FoliatedManifold both;
            both.nc = nc.merge (other.nc);
            both.nc_path = nc_path.merge (other.nc_path);

            std::copy (lj.begin (), lj.end (), both.lj.end ());
            std::copy (other.lj.begin (), other.lj.end (), both.lj.end ());
            return both;
          }

          void addToState (StatePtr_t comp) const;
          void addToEdge (EdgePtr_t comp) const;
          void specifyFoliation (LevelSetEdgePtr_t lse) const;

          bool foliated () const {
            return !lj_fol.empty () || !nc_fol.nc.empty ();
          }
          bool empty () const {
            return lj.empty () && nc.nc.empty ();
          }
        };

        enum GraspingCase {
          NoGrasp = 1 << 0,
          GraspOnly = 1 << 1,
          WithPreGrasp = 1 << 2
        };
        enum PlacementCase {
          NoPlace = 1 << 3,
          PlaceOnly = 1 << 4,
          WithPrePlace = 1 << 5
        };

	struct Rule {
          std::vector<std::string> grippers_;
          std::vector<std::string> handles_;
	  bool link_;
	  Rule() : grippers_(), handles_(), link_(false) {}
	};

        typedef std::vector<Rule> Rules_t;

        /// Create edges according to the case.
        /// gCase is a logical OR combination of GraspingCase and PlacementCase
        ///
        /// When an argument is not relevant, use the default constructor
        /// of FoliatedManifold
        template < int gCase >
          Edges_t createEdges (
              const std::string& forwName,   const std::string& backName,
              const StatePtr_t& from,         const StatePtr_t& to,
              const size_type& wForw,        const size_type& wBack,
              const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
              const FoliatedManifold& place, const FoliatedManifold& preplace,
              const bool levelSetGrasp,      const bool levelSetPlace,
              const FoliatedManifold& submanifoldDef = FoliatedManifold ()
              );

        EdgePtr_t createLoopEdge (
              const std::string& loopName,
              const StatePtr_t& state,
              const size_type& w,
              const bool levelSet,
              const FoliatedManifold& submanifoldDef = FoliatedManifold ()
              );

        /// Create a waypoint edge taking into account:
        /// \li grasp
        /// \li placement
        /// \li preplacement

        /// Create a waypoint edge taking into account
        /// \li grasp
        /// \li pregrasp
        /// \li placement

        /// \todo when the handle is a free flying object, add the robot DOFs
        ///       as passive dofs to the numerical constraints for paths
        void graspManifold (
            const GripperPtr_t& gripper, const HandlePtr_t& handle,
            FoliatedManifold& grasp, FoliatedManifold& pregrasp);

        /// The placement foliation constraint is built using
        /// hpp::constraints::ConvexShapeMatcherComplement
        void strictPlacementManifold (
            const ImplicitPtr_t placement,
            const ImplicitPtr_t preplacement,
            const ImplicitPtr_t placementComplement,
            FoliatedManifold& place, FoliatedManifold& preplace);

        /// The placement foliation constraint is built locked joints
        /// It is faster than strictPlacementManifold but the foliation
        /// parametrisation is redundant.
        void relaxedPlacementManifold (
            const ImplicitPtr_t placement,
            const ImplicitPtr_t preplacement,
            const LockedJoints_t objectLocks,
            FoliatedManifold& place, FoliatedManifold& preplace);

        typedef boost::tuple <ImplicitPtr_t,
                              ImplicitPtr_t,
                              LockedJoints_t>
                              PlacementConstraint_t;
        typedef std::vector <HandlePtr_t> Handles_t;
        typedef std::vector <GripperPtr_t> Grippers_t;
        /// Tuple representing an object as follows:
        /// \li PlacementConstraint_t constraint to place the object
        /// \li Handles_t             list of handles of the object
        /// \li std::size_t           the index of this tuple in Objects_t.
        /// \note the index must be unique, as object equallity is checked using this index.
        typedef boost::tuple <PlacementConstraint_t, Handles_t, std::size_t> Object_t;
        typedef std::vector <Object_t> Objects_t;

        /// Fill a Graph 
        ///
        /// \note It is assumed that a gripper can grasp only one handle and each
        /// handle cannot be grasped by several grippers at the same time.
        ///
        /// \param[in,out] graph must be an initialized empty Graph.
        void graphBuilder (
            const ProblemSolverPtr_t& ps,
            const Objects_t& objects,
            const Grippers_t& grippers,
            GraphPtr_t graph,
            const Rules_t& rules = Rules_t ());

        struct ObjectDef_t {
          std::string name;
          StringList_t handles, shapes;
        };

        GraphPtr_t graphBuilder (
            const ProblemSolverPtr_t& ps,
            const std::string& graphName,
            const StringList_t& griNames,
            const std::list <ObjectDef_t>& objs,
            const StringList_t& envNames,
	    const Rules_t& rules,
            const value_type& prePlaceWidth = 0.05);
        /// \}
      } // namespace helper
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_HELPER_HH
