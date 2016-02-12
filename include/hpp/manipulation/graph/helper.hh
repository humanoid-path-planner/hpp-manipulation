// Copyright (c) 2016, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_GRAPH_HELPER_HH
# define HPP_MANIPULATION_GRAPH_HELPER_HH

# include <string>
# include <algorithm>

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// \addtogroup constraint_graph
      /// \{

      struct NumericalConstraintsAndPassiveDofs {
        NumericalConstraints_t nc;
        IntervalsContainer_t pdof;
        NumericalConstraintsAndPassiveDofs merge
          (const NumericalConstraintsAndPassiveDofs& other) {
            NumericalConstraintsAndPassiveDofs ret;
            // ret.nc.reserve (nc.size() + other.nc.size());
            ret.pdof.reserve (pdof.size() + other.pdof.size());

            std::copy (nc.begin(), nc.end(), ret.nc.begin());
            std::copy (other.nc.begin(), other.nc.end(), ret.nc.begin());

            std::copy (pdof.begin(), pdof.end(), ret.pdof.begin());
            std::copy (other.pdof.begin(), other.pdof.end(), ret.pdof.begin());
            return ret;
          }

        template <bool forPath> void addToComp (GraphComponentPtr_t comp) const;

        template <bool param> void specifyFoliation (LevelSetEdgePtr_t lse) const;
      };

      struct FoliatedManifold {
        // Manifold definition
        NumericalConstraintsAndPassiveDofs nc;
        LockedJoints_t lj;
        NumericalConstraintsAndPassiveDofs nc_path;
        // Foliation definition
        NumericalConstraintsAndPassiveDofs nc_fol;
        LockedJoints_t lj_fol;

        FoliatedManifold merge (const FoliatedManifold& other) {
          FoliatedManifold both;
          both.nc = nc.merge (other.nc);
          both.nc_path = nc_path.merge (other.nc_path);

          std::copy (lj.begin (), lj.end (), both.lj.end ());
          std::copy (other.lj.begin (), other.lj.end (), both.lj.end ());
          return both;
        }

        void addToNode (NodePtr_t comp) const;
        void addToEdge (EdgePtr_t comp) const;
        void specifyFoliation (LevelSetEdgePtr_t lse) const;

        bool isFoliated () const {
          return lj_fol.empty () && nc_fol.nc.empty ();
        }
      };

      class HPP_MANIPULATION_DLLAPI Helper
      {
        public:
          typedef std::pair <WaypointEdgePtr_t, WaypointEdgePtr_t> WaypointEdgePair_t;

          WaypointEdgePair_t createWaypoints (
              const std::string& forwName,   const std::string& backName,
              const NodePtr_t& from,         const NodePtr_t& to,
              const size_type& wForw,        const size_type& wBack,
              const FoliatedManifold& grasp, const FoliatedManifold& pregrasp,
              const FoliatedManifold& place, const FoliatedManifold& preplace,
              const bool levelSetPlace,      const bool levelSetGrasp,
              const FoliatedManifold& submanifoldDef = FoliatedManifold ()
              );
      };
      /// \}
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_HELPER_HH
