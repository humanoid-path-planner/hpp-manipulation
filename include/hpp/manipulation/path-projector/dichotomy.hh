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

#ifndef HPP_MANIPULATION_PATHPROJECTOR_DICHOTOMY_HH
# define HPP_MANIPULATION_PATHPROJECTOR_DICHOTOMY_HH

# include "hpp/manipulation/path-projector.hh"
namespace hpp {
  namespace manipulation {
    namespace pathProjector {
      class HPP_MANIPULATION_DLLAPI Dichotomy : public PathProjector
      {
        public:
          Dichotomy (const core::DistancePtr_t distance, value_type maxPathLength);

        protected:
          bool impl_apply (const StraightPathPtr_t path, PathPtr_t& projection) const;

        private:
          value_type maxPathLength_;
      };
    } // namespace pathProjector
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_PATHPROJECTOR_DICHOTOMY_HH
