// Copyright (c) 2019, LAAS-CNRS
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

#define HPP_DEBUG 1

#include "hpp/manipulation/graph/validation.hh"

#include <sstream>

#include <hpp/util/indent.hh>

#include <hpp/core/collision-validation.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/relative-motion.hh>

#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/state.hh"
#include "hpp/manipulation/graph/state-selector.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      std::ostream& Validation::print (std::ostream& os) const
      {
        for (std::size_t i = 0; i < warnings_.size(); ++i) {
          os << "Warning " << i
            << " (" << (warnings_[i].first ? warnings_[i].first->name() : "no component")
            << ')' << incendl << warnings_[i].second << decendl;
        }
        for (std::size_t i = 0; i < errors_.size(); ++i) {
          os << "Error " << i
            << " (" << (errors_[i].first ? errors_[i].first->name() : "no component")
            << ')' << incendl << errors_[i].second << decendl;
        }
        return os;
      }

      bool Validation::validate (const GraphComponentPtr_t& comp)
      {
        if      (HPP_DYNAMIC_PTR_CAST(State, comp))
          return validateState (HPP_DYNAMIC_PTR_CAST(State, comp));
        else if (HPP_DYNAMIC_PTR_CAST(Edge, comp))
          return validateEdge  (HPP_DYNAMIC_PTR_CAST(Edge, comp));
        else if (HPP_DYNAMIC_PTR_CAST(Graph, comp))
          return validateGraph (HPP_DYNAMIC_PTR_CAST(Graph, comp));
        else
          return true;
      }

      bool Validation::validateState (const StatePtr_t& state)
      {
        std::ostringstream oss;
        oss << incindent;
        bool success = true;

        // 1. try to generate a configuration in this state.
        bool projOk;
        Configuration_t q;
        std::size_t i, Nrand = 100;

        for (i = 0; i < Nrand; i++) {
          problem_->configurationShooter()->shoot(q);
          projOk = state->configConstraint()->apply (q);
          if (projOk) break;
        }
        if (!projOk) {
          oss << "Failed to apply the constraints to " << Nrand << "random configurations.";
          addError (state, oss.str());
          return false;
        }
        if (4 * i > 3 * Nrand) {
          oss << "Success rate of constraint projection is " << i / Nrand << '.';
          addWarning (state, oss.str());
          oss.clear();
        }

        // 2. check the collision pairs which will be disabled because of the
        //    constraint.
        core::CollisionValidationPtr_t colValidation (
            core::CollisionValidation::create (problem_->robot()));
        for (std::size_t i = 0; i < problem_->collisionObstacles().size(); ++i)
          colValidation->addObstacle (problem_->collisionObstacles()[i]);
        colValidation->computeAllContacts (true);

        typedef core::RelativeMotion RelativeMotion;
        RelativeMotion::matrix_type relMotion = RelativeMotion::matrix (problem_->robot());
        RelativeMotion::fromConstraint (relMotion, problem_->robot(),
            state->configConstraint());

        // Invert the relative motions.
        hppDout (info, "Relative motion matrix:\n" << relMotion);
        for (size_type r = 0; r < relMotion.rows(); ++r)
          for (size_type c = 0; c < r; ++c) {
            if (relMotion(r,c) != relMotion(c,r)) {
              hppDout (error, "Relative motion matrix not symmetric.");
            }
            switch (relMotion(r,c)) {
              case RelativeMotion::Constrained:
                relMotion(r,c) = relMotion(c,r) = RelativeMotion::Unconstrained;
                break;
              case RelativeMotion::Parameterized:
              case RelativeMotion::Unconstrained:
                relMotion(r,c) = relMotion(c,r) = RelativeMotion::Constrained;
                break;
              default:
                throw std::logic_error ("Relative motion not understood");
            }
          }
        hppDout (info, "Relative motion matrix:\n" << relMotion);

        colValidation->filterCollisionPairs (relMotion);
        core::ValidationReportPtr_t colReport;
        bool colOk = colValidation->validate (q, colReport);

        if (!colOk) {
          oss << "The following collision pairs will always collide." << incendl << *colReport << decindent;
          addError (state, oss.str());
          success = false;
        }

        return success;
      }

      bool Validation::validateEdge  (const EdgePtr_t &)
      {
        return true;
      }

      bool Validation::validateGraph (const GraphPtr_t& graph)
      {
        if (!graph) return false;
        bool success = true;

        States_t states = graph->stateSelector()->getStates();
        for (std::size_t i = 1; i < graph->nbComponents(); ++i)
          if (!validate(graph->get(i).lock())) success = false;

        return success;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
