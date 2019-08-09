// Copyright (c) 2019, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_GRAPH_VALIDATION_REPORT_HH
# define HPP_MANIPULATION_GRAPH_VALIDATION_REPORT_HH

# include <string>
# include <vector>
# include <hpp/core/validation-report.hh>

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// \addtogroup constraint_graph
      /// \{

      /// Check that graph components are valid.
      ///
      /// A stringified validation report can be obtained via
      /// Validation::print or operator<< (std::ostream&, const Validation&).
      class HPP_MANIPULATION_DLLAPI Validation
      {
        public:
          Validation(const core::ProblemPtr_t& problem)
            : problem_ (problem) {}

          void clear ()
          {
            warnings_.clear();
            errors_.clear();
          }

          bool hasWarnings () const { return !warnings_.empty(); }

          bool hasErrors () const { return !errors_.empty(); }

          virtual std::ostream& print (std::ostream& os) const;

          /// Validate a graph component.
          /// It dynamically casts in order to call the right function among
          /// the validation method below.
          ///
          /// \return true if the component could not be proven infeasible.
          /// \note Even if true is returned, the report can contain warnings.
          bool validate (const GraphComponentPtr_t& comp);

          /// Validate a state
          /// \return true if the state could not be proven infeasible.
          /// \note Even if true is returned, the report can contain warnings.
          bool validateState (const StatePtr_t& state);

          /// Validate an edge
          /// \return true if the edge could not be proven infeasible.
          /// \note Even if true is returned, the report can contain warnings.
          bool validateEdge  (const EdgePtr_t & edge);

          /// Validate an graph
          /// \return true if no component of the graph could not be proven infeasible.
          /// \note Even if true is returned, the report can contain warnings.
          bool validateGraph (const GraphPtr_t& graph);


        private:
          void addWarning (const GraphComponentPtr_t& c, const std::string& w)
          {
            warnings_.push_back (Message (c, w));
          }

          void addError (const GraphComponentPtr_t& c, const std::string& w)
          {
            errors_.push_back (Message (c, w));
          }

          typedef std::pair<GraphComponentPtr_t, std::string> Message;
          std::vector<Message> warnings_, errors_;

          core::ProblemPtr_t problem_;
      };

      inline std::ostream& operator<< (std::ostream& os, const Validation& v)
      {
        return v.print(os);
      }

      /// \}
    } // namespace graph
  } // namespace manipulation

} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_VALIDATION_REPORT_HH
