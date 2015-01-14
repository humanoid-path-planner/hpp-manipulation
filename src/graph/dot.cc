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

#include "hpp/manipulation/graph/dot.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      namespace dot {
        const std::string Tooltip::tooltipendl = "&#10;";

        std::ostream& operator<< (std::ostream& os, const DrawingAttributes& da)
        {
          if (da.attr.empty ()) return os;
          os << da.openSection;
          size_t i = da.attr.size ();
          for (DrawingAttributes::Map::const_iterator it = da.attr.begin ();
              it != da.attr.end (); ++it) {
            os << it->first << "=" << it->second; 
            i--;
            if (i > 0) os << da.separator;
          }
          return os << da.closeSection;
        }

        std::ostream& insertComments (std::ostream& os, const std::string& c)
        {
          return os << "/*" << std::endl << c << std::endl << "*/";
        }
      } // namespace dot
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
