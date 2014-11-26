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

#include <hpp/util/pointer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/distance.hh>

#include "hpp/manipulation/path-projector.hh"

namespace hpp {
  namespace manipulation {
    PathProjector::PathProjector (const core::DistancePtr_t distance) :
      distance_ (distance)
    {}

    PathProjector::~PathProjector ()
    {}

    value_type PathProjector::d (ConfigurationIn_t q1, ConfigurationIn_t q2) const
    {
      assert (distance_ != NULL);
      return (*distance_) (q1, q2);
    }

    bool PathProjector::apply (const PathPtr_t path, PathPtr_t& proj) const
    {
      assert (path);
      PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (PathVector, path);
      if (!pv) {
        StraightPathPtr_t sp = HPP_DYNAMIC_PTR_CAST (StraightPath, path);
        if (!sp) throw std::invalid_argument ("Unknow inherited class of Path");
        return impl_apply (sp, proj);
      }
      else {
        PathVectorPtr_t res = PathVector::create (pv->outputSize());
        PathPtr_t part;
        for (size_t i = 0; i < pv->numberPaths (); i++) {
          if (!apply (pv->pathAtRank (i), part)) {
            if (part && part->length () > 0) res->appendPath (part);
            return false;
          }
          res->appendPath (part);
        }
        proj = res;
      }
      assert (proj);
      return true;
    }
  } // namespace manipulation
} // namespace hpp
