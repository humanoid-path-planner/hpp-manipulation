//
// Copyright (c) 2015 CNRS
// Authors: Anna Seppala (seppala@laas.fr)
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_CONNECTED_COMPONENT_HH
#define HPP_MANIPULATION_CONNECTED_COMPONENT_HH

#include <hpp/core/connected-component.hh>

# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    /// \addtogroup connected-component
    /// \{
    /// Extension of hpp::core::connected-component. Adds a list of roadmap nodes for
    /// every contraint graph node within the connected component. Thus every roadmap
    /// node is assigned to a grahp node, which minimises computation time.
class HPP_MANIPULATION_DLLAPI ConnectedComponent : public core::ConnectedComponent 
{ 
  public:
      // Map of graph nodes within the connected component
      typedef std::map <char, graph::NodePtr_t> graphNodes_t;
  protected:
  private:
	std:
    }; // class ConnectedComponent
    /// \}
  } //   namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_CONNECTED_COMPONENT_HH
