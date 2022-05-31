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

#ifndef HPP_MANIPULATION_GRAPH_DOT_HH
#define HPP_MANIPULATION_GRAPH_DOT_HH

#include <list>
#include <map>
#include <ostream>
#include <sstream>

namespace hpp {
namespace manipulation {
namespace graph {
namespace dot {

struct DrawingAttributes {
  typedef std::pair<std::string, std::string> Pair;
  typedef std::map<std::string, std::string> Map;

  std::string separator, openSection, closeSection;
  Map attr;

  inline void insertWithQuote(const std::string& K, const std::string& V) {
    attr.insert(Pair(K, "\"" + V + "\""));
  }
  inline void insert(const std::string& K, const std::string& V) {
    attr.insert(Pair(K, V));
  }
  std::string& operator[](const std::string& K) { return attr[K]; }
  DrawingAttributes()
      : separator(", "), openSection("["), closeSection("]"), attr(){};
};

struct Tooltip {
  static const std::string tooltipendl;
  typedef std::list<std::string> TooltipLineVector;
  TooltipLineVector v;

  Tooltip() : v(){};
  inline std::string toStr() const {
    std::stringstream ss;
    size_t i = v.size();
    for (TooltipLineVector::const_iterator it = v.begin(); it != v.end();
         ++it) {
      ss << *it;
      i--;
      if (i > 0) ss << tooltipendl;
    }
    return ss.str();
  }
  inline void addLine(const std::string& l) { v.push_back(l); }
};

std::ostream& insertComments(std::ostream& os, const std::string& c);

std::ostream& operator<<(std::ostream& os, const DrawingAttributes& da);
}  // namespace dot
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_DOT_HH
