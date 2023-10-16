// Copyright (c) 2014, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_MANIPULATION_PLANNER_HH
#define HPP_MANIPULATION_MANIPULATION_PLANNER_HH

#include <hpp/core/path-planner.hh>
#include <hpp/statistics/success-bin.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/statistics.hh"

namespace hpp {
namespace manipulation {
/// \addtogroup path_planning
/// \{

class HPP_MANIPULATION_DLLAPI ManipulationPlanner
    : public hpp::core::PathPlanner {
 public:
  typedef std::list<std::size_t> ErrorFreqs_t;

  /// Create an instance and return a shared pointer to the instance
  static ManipulationPlannerPtr_t create(const core::ProblemConstPtr_t& problem,
                                         const core::RoadmapPtr_t& roadmap);

  /// One step of extension.
  ///
  /// A set of constraints is chosen using the graph of constraints.
  /// A constraint extension is done using a chosen set.
  ///
  virtual void oneStep();

  /// Extend configuration q_near toward q_rand.
  /// \param q_near the configuration to be extended.
  /// \param q_rand the configuration toward extension is performed.
  /// \retval validPath the longest valid path (possibly of length 0),
  ///         resulting from the extension.
  /// \return True if the returned path is valid.
  bool extend(RoadmapNodePtr_t q_near, ConfigurationIn_t q_rand,
              core::PathPtr_t& validPath);

  /// Get the number of occurrence of each errors.
  ///
  /// \sa ManipulationPlanner::errorList
  ErrorFreqs_t getEdgeStat(const graph::EdgePtr_t& edge) const;

  /// Get the list of possible outputs of the extension step.
  ///
  /// \sa ManipulationPlanner::getEdgeStat
  static StringList_t errorList();

 protected:
  /// Protected constructor
  ManipulationPlanner(const ProblemConstPtr_t& problem,
                      const RoadmapPtr_t& roadmap);

  /// Store weak pointer to itself
  void init(const ManipulationPlannerWkPtr_t& weak);

 private:
  /// Try to connect nodes of the roadmap to other connected components.
  /// \return the number of connection made.
  std::size_t tryConnectToRoadmap(const core::Nodes_t nodes);
  /// Try to connect nodes in a list between themselves.
  /// \return the number of connection made.
  std::size_t tryConnectNewNodes(const core::Nodes_t nodes);

  /// Configuration shooter
  ConfigurationShooterPtr_t shooter_;
  /// Pointer to the problem
  ProblemConstPtr_t problem_;
  /// Pointer to the roadmap
  RoadmapPtr_t roadmap_;
  /// weak pointer to itself
  ManipulationPlannerWkPtr_t weakPtr_;

  /// Keep track of success and failure for method
  /// extend.
  typedef ::hpp::statistics::SuccessStatistics SuccessStatistics;
  typedef ::hpp::statistics::SuccessBin SuccessBin;
  typedef ::hpp::statistics::SuccessBin::Reason Reason;
  SuccessStatistics& edgeStat(const graph::EdgePtr_t& edge);
  std::vector<size_type> indexPerEdgeStatistics_;
  std::vector<SuccessStatistics> perEdgeStatistics_;

  /// A Reason is associated to each EdgePtr_t that generated a failure.
  enum TypeOfFailure {
    PATH_PROJECTION_SHORTER = 0,
    PATH_VALIDATION_SHORTER = 1,
    REACHED_DESTINATION_NODE = 2,
    FAILURE = 3,
    PROJECTION = 4,
    STEERING_METHOD = 5,
    PATH_VALIDATION_ZERO = 6,
    PATH_PROJECTION_ZERO = 7
  };
  static const std::vector<Reason> reasons_;

  value_type extendStep_;

  mutable Configuration_t qProj_;
};
/// \}
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_MANIPULATION_PLANNER_HH
