// Copyright (c) 2016, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_MANIPULATION_GRAPH_HELPER_HH
#define HPP_MANIPULATION_GRAPH_HELPER_HH

#include <algorithm>
#include <string>
#include <tuple>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
namespace manipulation {
typedef constraints::ImplicitPtr_t ImplicitPtr_t;
namespace graph {
namespace helper {
/// \defgroup helper Helpers to build the graph of constraints
/// \addtogroup helper
/// \{

NumericalConstraints_t merge_nc(const NumericalConstraints_t& a,
                                const NumericalConstraints_t& b) {
  NumericalConstraints_t nc;
  nc.reserve(a.size() + b.size());
  std::copy(a.begin(), a.end(), nc.begin());
  std::copy(b.begin(), b.end(), nc.begin());
  return nc;
}

struct FoliatedManifold {
  // Manifold definition
  NumericalConstraints_t nc;
  LockedJoints_t lj;
  NumericalConstraints_t nc_path;
  // Foliation definition
  NumericalConstraints_t nc_fol;
  LockedJoints_t lj_fol;

  FoliatedManifold merge(const FoliatedManifold& other) {
    FoliatedManifold both;
    both.nc = merge_nc(nc, other.nc);
    both.nc_path = merge_nc(nc_path, other.nc_path);

    std::copy(lj.begin(), lj.end(), both.lj.end());
    std::copy(other.lj.begin(), other.lj.end(), both.lj.end());
    return both;
  }

  void addToState(StatePtr_t comp) const;
  void addToEdge(EdgePtr_t comp) const;
  void specifyFoliation(LevelSetEdgePtr_t lse) const;

  bool foliated() const { return !lj_fol.empty() || !nc_fol.empty(); }
  bool empty() const { return lj.empty() && nc.empty(); }
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
template <int gCase>
Edges_t createEdges(
    const std::string& forwName, const std::string& backName,
    const StatePtr_t& from, const StatePtr_t& to, const size_type& wForw,
    const size_type& wBack, const FoliatedManifold& grasp,
    const FoliatedManifold& pregrasp, const FoliatedManifold& place,
    const FoliatedManifold& preplace, const bool levelSetGrasp,
    const bool levelSetPlace,
    const FoliatedManifold& submanifoldDef = FoliatedManifold());

EdgePtr_t createLoopEdge(
    const std::string& loopName, const StatePtr_t& state, const size_type& w,
    const bool levelSet,
    const FoliatedManifold& submanifoldDef = FoliatedManifold());

/// Create a waypoint edge taking into account:
/// \li grasp
/// \li placement
/// \li preplacement

/// Create a waypoint edge taking into account
/// \li grasp
/// \li pregrasp
/// \li placement

void graspManifold(const GripperPtr_t& gripper, const HandlePtr_t& handle,
                   FoliatedManifold& grasp, FoliatedManifold& pregrasp);

/// The placement foliation constraint is built using
/// hpp::constraints::ConvexShapeMatcherComplement
void strictPlacementManifold(const ImplicitPtr_t placement,
                             const ImplicitPtr_t preplacement,
                             const ImplicitPtr_t placementComplement,
                             FoliatedManifold& place,
                             FoliatedManifold& preplace);

/// The placement foliation constraint is built locked joints
/// It is faster than strictPlacementManifold but the foliation
/// parametrisation is redundant.
void relaxedPlacementManifold(const ImplicitPtr_t placement,
                              const ImplicitPtr_t preplacement,
                              const LockedJoints_t objectLocks,
                              FoliatedManifold& place,
                              FoliatedManifold& preplace);

typedef std::tuple<ImplicitPtr_t, ImplicitPtr_t, LockedJoints_t>
    PlacementConstraint_t;
typedef std::vector<HandlePtr_t> Handles_t;
typedef std::vector<GripperPtr_t> Grippers_t;
/// Tuple representing an object as follows:
/// \li PlacementConstraint_t constraint to place the object
/// \li Handles_t             list of handles of the object
/// \li std::size_t           the index of this tuple in Objects_t.
/// \note the index must be unique, as object equallity is checked using this
/// index.
typedef std::tuple<PlacementConstraint_t, Handles_t, std::size_t> Object_t;
typedef std::vector<Object_t> Objects_t;

/// Fill a Graph
///
/// \note It is assumed that a gripper can grasp only one handle and each
/// handle cannot be grasped by several grippers at the same time.
///
/// \param[in,out] graph must be an initialized empty Graph.
void graphBuilder(const ProblemSolverPtr_t& ps, const Objects_t& objects,
                  const Grippers_t& grippers, GraphPtr_t graph,
                  const Rules_t& rules = Rules_t());

struct ObjectDef_t {
  std::string name;
  Strings_t handles, shapes;
};

GraphPtr_t graphBuilder(const ProblemSolverPtr_t& ps,
                        const std::string& graphName, const Strings_t& griNames,
                        const std::vector<ObjectDef_t>& objs,
                        const Strings_t& envNames, const Rules_t& rules,
                        const value_type& prePlaceWidth = 0.05);
/// \}
}  // namespace helper
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_HELPER_HH
