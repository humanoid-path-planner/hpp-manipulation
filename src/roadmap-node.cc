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

#include "hpp/manipulation/roadmap-node.hh"

#include <hpp/manipulation/connected-component.hh>

namespace hpp {
  namespace manipulation {
    RoadmapNode::RoadmapNode (const ConfigurationPtr_t& configuration,
        ConnectedComponentPtr_t cc) :
      core::Node (configuration, cc),
      state_ ()
    {}
  } // namespace manipulation
} // namespace hpp
