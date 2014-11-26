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

#ifndef HPP_MANIPULATION_PATHPROJECTOR_HH
# define HPP_MANIPULATION_PATHPROJECTOR_HH

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/fwd.hh"

# include <hpp/core/path.hh>
# include <hpp/core/straight-path.hh>

namespace hpp {
  namespace manipulation {
    /// This class projects a path using constraints.
    class HPP_MANIPULATION_DLLAPI PathProjector
    {
      public:
        typedef hpp::core::Path Path;
        typedef hpp::core::PathPtr_t PathPtr_t;
        typedef hpp::core::StraightPath StraightPath;
        typedef hpp::core::StraightPathPtr_t StraightPathPtr_t;
        typedef hpp::core::PathVector PathVector;
        typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;

        /// Destructor
        virtual ~PathProjector ();

        /// Apply the constraints to the path.
        /// \param[in] the input path,
        /// \param[out] the output path.
        /// \return True if projection succeded
        bool apply (const PathPtr_t path, PathPtr_t& projection) const;

      protected:
        /// Constructor
        PathProjector (const core::DistancePtr_t distance);

        /// Method to be reimplemented by inherited class.
        virtual bool impl_apply (const StraightPathPtr_t path, PathPtr_t& projection) const = 0;

        value_type d (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

      private:
        core::DistancePtr_t distance_;
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATHPROJECTOR_HH
