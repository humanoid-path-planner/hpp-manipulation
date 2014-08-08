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

#include "hpp/manipulation/graph-path-validation.hh"

namespace hpp {
  namespace manipulation {
    GraphPathValidationPtr_t GraphPathValidation::create (
        const PathValidationPtr_t& pathValidation, const GraphPtr_t& graph)
    {
      GraphPathValidation* p = new GraphPathValidation (pathValidation, graph);
      return GraphPathValidationPtr_t (p);
    }

    GraphPathValidation::GraphPathValidation (
        const PathValidationPtr_t& pathValidation, const GraphPtr_t& graph) :
      pathValidation_ (pathValidation), constraintGraph_ (graph)
    {}

    bool GraphPathValidation::validate (
          const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      PathPtr_t pathGraphValid;
      bool graphValid = impl_validate (path, reverse, pathGraphValid);
      bool collisionValid = pathValidation_->validate (pathGraphValid, reverse, validPart); 
      return graphValid && collisionValid;
    }

    bool GraphPathValidation::impl_validate (
        const PathVectorPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      size_t start = 0,
             end = path->numberPaths ();
      int inc = 1;
      if (reverse) {
        std::swap (start, end);
        inc = -1;
      }
      PathPtr_t validSubPart;
      value_type timeOffset = path->timeRange().first; 
      for (size_t index = start; index != end; index += inc) {
        // We should stop at the first non valid subpath.
        if (!impl_validate (path->pathAtRank (index), reverse, validSubPart)) {
          if (reverse)
            validPart = path->extract (
                std::make_pair (timeOffset + validSubPart->timeRange().first,
                                path->timeRange().second));
          else
            validPart = path->extract (
                std::make_pair (path->timeRange().first,
                                timeOffset + validSubPart->timeRange().second));
          return false;
        }
        timeOffset += path->pathAtRank (index)->length(); 
      }
      // Here, every subpath is valid.
      validPart = path;
      return true;
    }

    bool GraphPathValidation::impl_validate (
        const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      PathVectorPtr_t pathVector (path->as <PathVector> ());
      if (pathVector)
        return impl_validate (pathVector, reverse, validPart);

      value_type tmin = path->timeRange ().first;
      value_type tmax = path->timeRange ().second;
      if (reverse)
        std::swap (tmin, tmax);
      const Path& configAt (*path);
      graph::Nodes_t origNodes = constraintGraph_->getNode (configAt (tmin));
      graph::Nodes_t destNodes = constraintGraph_->getNode (configAt (tmax));
      std::vector <graph::Edges_t> possibleEdges (constraintGraph_->getEdge (origNodes, destNodes));
      // We check for all of them if both nodes are on the same leaf.
      ConstraintSetPtr_t constraints;
      while (!possibleEdges.empty ()) {
        constraints = constraintGraph_->pathConstraint (possibleEdges.back(), configAt (tmin));
        // TODO: We need a quick way of checking that a configuration
        // statisfies a constraint.
        Configuration_t cfg = configAt (tmax);
        if (constraints->apply(cfg) && ( cfg == configAt(tmax) )) {
          validPart = path;
          return true;
        }
        possibleEdges.pop_back();
      }
      validPart = path->extract (std::make_pair (tmin, tmin));
      return false;
    }
  } // namespace manipulation
} // namespace hpp
