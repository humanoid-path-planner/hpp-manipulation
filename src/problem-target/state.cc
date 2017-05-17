// Copyright (c) 2017, Joseph Mirabel
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

#include <hpp/manipulation/problem-target/state.hh>

#include <stdexcept>

#include <hpp/util/debug.hh>

#include <hpp/core/node.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>

#include "astar.hh"

namespace hpp {
  namespace manipulation {
    namespace problemTarget {
      StatePtr_t State::create (const core::ProblemPtr_t& problem)
      {
        State* tt = new State (problem);
        StatePtr_t shPtr (tt);
        tt->init (shPtr);
        return shPtr;
      }

      void State::check (const core::RoadmapPtr_t&) const
      {
        if (!state_) {
          HPP_THROW(std::runtime_error, "No state: task not specified.");
        }
      }

      bool State::reached (const core::RoadmapPtr_t& roadmap) const
      {
        const core::ConnectedComponentPtr_t& _cc = roadmap->initNode()->connectedComponent();
        const ConnectedComponentPtr_t cc = HPP_DYNAMIC_PTR_CAST(ConnectedComponent, _cc);
        assert (cc);
        return !cc->getRoadmapNodes(state_).empty();
      }

      core::PathVectorPtr_t State::computePath(const core::RoadmapPtr_t& roadmap) const
      {
        Astar astar (roadmap, problem_->distance (), state_);
        return astar.solution ();
      }
    } // namespace problemTarget
  } // namespace manipulation
} // namespace hpp
