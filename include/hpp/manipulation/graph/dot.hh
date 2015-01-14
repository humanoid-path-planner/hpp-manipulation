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

#ifndef HPP_MANIPULATION_GRAPH_DOT_HH
# define HPP_MANIPULATION_GRAPH_DOT_HH

# include <ostream>
# include <sstream>
# include <map>
# include <list>

namespace hpp {
  namespace manipulation {
    namespace graph {
      namespace dot {

        struct DrawingAttributes {
          typedef std::pair <std::string, std::string> Pair;
          typedef std::map <std::string, std::string> Map;

          std::string separator, openSection, closeSection;
          Map attr;

          inline void insertWithQuote (const std::string& K, const std::string& V) {
            attr.insert (Pair (K, "\"" + V + "\""));
          }
          inline void insert (const std::string& K, const std::string& V) {
            attr.insert (Pair (K, V));
          }
          std::string& operator [] (const std::string& K) {
            return attr [K];
          }
          DrawingAttributes () :
            separator (", "), openSection ("["), closeSection ("]"),
            attr () {};
        };

        struct Tooltip {
          static const std::string tooltipendl;
          typedef std::list <std::string> TooltipLineVector;
          TooltipLineVector v;

          Tooltip () : v() {};
          inline std::string toStr () const {
            std::stringstream ss;
            size_t i = v.size ();
            for (TooltipLineVector::const_iterator
                it = v.begin (); it != v.end (); ++it ) {
              ss << *it;
              i--;
              if (i > 0) ss << tooltipendl;
            }
            return ss.str ();
          }
          inline void addLine (const std::string& l) {
            v.push_back (l);
          }
        };

        std::ostream& insertComments (std::ostream& os, const std::string& c);

        std::ostream& operator<< (std::ostream& os, const DrawingAttributes& da);
      } // namespace dot
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_DOT_HH
