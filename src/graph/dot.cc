// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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
