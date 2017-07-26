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

#include <hpp/manipulation/path-optimization/keypoints.hh>

#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>

#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph-steering-method.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      using std::make_pair;

      PathVectorPtr_t Keypoints::optimize (const PathVectorPtr_t& path)
      {
        PathVectorPtr_t input = PathVector::create (
              path->outputSize(), path->outputDerivativeSize()); 
        PathVectorPtr_t output = path;
        path->flatten (input);
        // Step 1: find the key waypoints
        // paths is a vector of 
        IKPvector_t paths = split (input);

        // Step 2: for each couple of consecutive path between keywaypoints,
        //         try to find a better intermediate key waypoint.
        std::size_t i1 = 0;
        bool success = false;
        for (std::size_t i2 = 1; i2 < paths.size(); ++i2) {
          if (paths[i2].isShort) continue; // So that i2 = i1 + 2
          // Try to generate a new key waypoint
          PathVectorPtr_t shortcut = shorten (paths, i1, i2);
          if (shortcut) {
            paths = replaceInPath (paths, shortcut, i1, i2);
            success = true;
            while (i2 < paths.size() && !paths[i2].isShort) ++i2;
          }
          i1 = i2;
        }

        if (success) {
          // Build path
          PathVectorPtr_t output = PathVector::create (path->outputSize (),
              path->outputDerivativeSize ());
          for (std::size_t j = 0; j < paths.size(); ++j)
            output->concatenate (paths[j].path);
        }
        return output;
      }

      Keypoints::IKPvector_t Keypoints::split (PathVectorPtr_t input) const
      {
        IKPvector_t paths;

        ConstraintSetPtr_t c;
        for (std::size_t i_s = 0; i_s < input->numberPaths ();) {
          InterKeypointPath ikpp;
          ikpp.path = PathVector::create (
              input->outputSize(), input->outputDerivativeSize()); 
          PathPtr_t current = input->pathAtRank (i_s);
          ikpp.path->appendPath (current);
          c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
          if (c) ikpp.edge = c->edge ();
          ikpp.isShort = ikpp.edge && ikpp.edge->isShort();
          std::size_t i_e = i_s + 1;
          for (; i_e < input->numberPaths (); ++i_e) {
            current = input->pathAtRank (i_e);
            c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
            if (!c && ikpp.edge) {
              hppDout(info, "No manipulation::ConstraintSet");
              break;
            }
            if (c && ikpp.edge->state() != c->edge ()->state()) break;
            if (ikpp.isShort != c->edge()->isShort()) // We do not optimize edges marked as short
              break;
            ikpp.path->appendPath (current);
          }
          hppDout(info, "Edge name: " << ikpp.edge->name());
          i_s = i_e;
          paths.push_back (ikpp);
        }
        return paths;
      }

      PathVectorPtr_t Keypoints::shorten (const IKPvector_t& paths,
          std::size_t i1, std::size_t i2) const
      {
        PathVectorPtr_t result;
        const std::size_t maxTrials = 10;
        value_type cur_length = 0;
        for (std::size_t i = i1; i < i2 + 1; ++i)
          cur_length += paths[i].path->length();

        const core::SteeringMethodPtr_t& sm (problem_.steeringMethod ());
        PathProjectorPtr_t pathProjector (problem().pathProjector ());
        core::PathValidationPtr_t pathValidation (problem ().pathValidation ());

        // Get all the possible edges of the graph.
        const InterKeypointPath& ikpp1 = paths[i1];
        const InterKeypointPath& ikpp2 = paths[i2];
        graph::Edges_t edges;
        graph::Graph& graph = *problem_.constraintGraph ();
        try {
          edges = graph.getEdges (ikpp1.edge->state(), ikpp2.edge->state());
        } catch (const std::logic_error& e) {
          hppDout (error, e.what ());
          return PathVectorPtr_t ();
        }

        value_type t0, t1;
        Configuration_t q  (ikpp1.path->outputSize()),
                        q1 (ikpp1.path->outputSize()),
                        q4 (ikpp1.path->outputSize());

        // Unused variables
        PathValidationReportPtr_t report;
        PathPtr_t unusedP;

        for (std::size_t n1 = 0; n1 < maxTrials; ++n1) {
          // Generate a random parameter on p1.
          t0 = ikpp1.path->timeRange().first;
          t1 = t0 + ikpp1.path->timeRange().second * rand ()/RAND_MAX; 
          // Get the configuration and project it.
          if (!(*ikpp1.path) (q1, t1)) continue;
          for (graph::Edges_t::const_iterator _edges = edges.begin();
              _edges != edges.end(); ++_edges) {
            // TODO: For level set edges, something different should be done.
            // Maybe, we must add a virtual generateInIntersection (qfrom, qto, qrand)
            // that would be specialized for each edge.
            q = q1;
            if ((*_edges)->applyConstraints (q1,q)) {
              PathPtr_t short1, proj1;
              if ((*_edges)->build (short1, q1, q)) {
                if (!short1 || !pathProjector->apply(short1, proj1)) continue;
                if (!pathValidation->validate (proj1, false, unusedP, report)) continue;

                // short1 is a valid path leading to the target node.
                // Try to connect this to ikpp2.path
                for (std::size_t n2 = 0; n2 < maxTrials; ++n2) {
                  value_type t3, t4, t5;
                  t3 = ikpp2.path->timeRange().first;
                  t4 = t3 + ikpp2.path->timeRange().second * rand ()/RAND_MAX; 
                  t5 = t3 + ikpp2.path->timeRange().second;
                  if (!(*ikpp2.path) (q4, t4)) continue;
                  PathPtr_t short2, proj2;
                  short2 = (*sm)(q, q4);
                  if (!short2 || !pathProjector->apply(short2, proj2)) continue;
                  if (!pathValidation->validate (proj2, false, unusedP, report)) continue;
                  // short2 is possible. Build the path and check it is shorter.

                  PathVectorPtr_t tmp = PathVector::create (ikpp1.path->outputSize (),
                      ikpp1.path->outputDerivativeSize ());
                  tmp->concatenate (ikpp1.path->extract
                        (make_pair (t0, t1))-> as <PathVector> ());
                  tmp->appendPath (short1);
                  tmp->appendPath (short2);
                  tmp->concatenate (ikpp2.path->extract
                        (make_pair (t4, t5))-> as <PathVector> ());

                  if (cur_length >= tmp->length()) {
                    cur_length = tmp->length();
                    result = tmp;
                  }
                }
              }
            }
          }
        }
        return result;
      }

      Keypoints::IKPvector_t Keypoints::replaceInPath(const IKPvector_t& input,
          const PathVectorPtr_t& shortcut,
          std::size_t i1, std::size_t i2) const
      {
        PathVectorPtr_t flat = PathVector::create (shortcut->outputSize (),
            shortcut->outputDerivativeSize ());
        shortcut->flatten(flat);

        PathVectorPtr_t output = PathVector::create (shortcut->outputSize (),
            shortcut->outputDerivativeSize ());
        for (std::size_t j = 0; j < i1; ++j)
          output->concatenate (input[j].path);
        output->concatenate (flat);
        for (std::size_t j = i2 + 1; j < input.size(); ++j)
          output->concatenate (input[j].path);

        return split (output);
      }
    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp
