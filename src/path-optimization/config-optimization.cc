// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/manipulation/path-optimization/config-optimization.hh>

#include <hpp/util/pointer.hh>

#include <hpp/core/path.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/constraint-set.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      ConfigProjectorPtr_t ConfigOptimizationTraits::getConfigProjector
        (const PathPtr_t& before, const PathPtr_t& after, bool& reverse)
      {
        ConstraintSetPtr_t setB =
          HPP_STATIC_PTR_CAST (ConstraintSet, before->constraints ());
        ConstraintSetPtr_t setA =
          HPP_STATIC_PTR_CAST (ConstraintSet, after->constraints ());
        assert (setA->edge () && setB->edge());
        graph::GraphPtr_t graph = setA->edge()->parentGraph ();
        graph::NodePtr_t n0 = graph->getNode (before->initial ()),
                         n1 = graph->getNode (before->end ()),
                         n2 = graph->getNode (after->initial ()),
                         n3 = graph->getNode (after->end ());
        /// Find if path were computed from init or goal config
        assert ((n0 == setB->edge()->from () && n1 == setB->edge()->to ())
                || (n1 == setB->edge()->from () && n0 == setB->edge()->to ()));
        assert ((n2 == setA->edge()->from () && n3 == setA->edge()->to ())
                || (n3 == setA->edge()->from () && n2 == setA->edge()->to ()));
        bool reverseB =
          (n1 == setB->edge()->from () && n0 == setB->edge()->to ());
        bool reverseA =
          (n3 == setA->edge()->from () && n2 == setA->edge()->to ());

        reverse = reverseB;

        ConfigProjectorPtr_t p = ConfigProjector::create (graph->robot(),
            "intersect_" + setB->edge()->name() + "_" + setA->edge()->name(),
            graph->errorThreshold (), graph->maxIterations ());

        graph->insertNumericalConstraints (p);
        // TODO: Is reverse case different ?
        bool nodeB_Eq_nodeA = (setB->edge()->node() == setA->edge()->node());

        setB->edge()->insertNumericalConstraints (p);
        setB->edge()->node ()->insertNumericalConstraints (p);

        graph->insertLockedJoints (p);
        setB->edge()->insertLockedJoints (p);
        setB->edge()->node ()->insertLockedJoints (p);

        vector_t rhsB = p->rightHandSideFromConfig (before->initial ());

        setA->edge()->insertNumericalConstraints (p);
        if (nodeB_Eq_nodeA)
          setA->edge()->node()->insertNumericalConstraints (p);
        setA->edge()->insertLockedJoints (p);
        if (nodeB_Eq_nodeA)
          setA->edge()->node()->insertLockedJoints (p);

        p->rightHandSideFromConfig (before->end ());
        return p;
      }
    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp
