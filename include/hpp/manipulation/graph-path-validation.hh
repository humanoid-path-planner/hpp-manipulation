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


#ifndef HPP_MANIPULATION_GRAPHPATHVALIDATOR_HH
# define HPP_MANIPULATION_GRAPHPATHVALIDATOR_HH

# include <hpp/core/path-validation.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    using hpp::core::PathValidation;
    using hpp::core::PathValidationPtr_t;
    using hpp::core::Path;
    using hpp::core::PathPtr_t;
    using hpp::core::PathVector;
    using hpp::core::PathVectorPtr_t;

    /// \addtogroup validation
    /// \{

    /// Path validation for a constraint graph
    ///
    /// This class encapsulates another path validation class.
    /// The encapsulated path validation is responsible for collision
    /// checking, whereas this class checks if a path is valid regarding
    /// the constraint graph.
    class HPP_MANIPULATION_DLLAPI GraphPathValidation : public PathValidation
    {
      public:
	/// Check that path is valid regarding the constraint graph.
	///
	/// \param path the path to check for validity,
	/// \param reverse if true check from the end,
	/// \retval the extracted valid part of the path, pointer to path if
	///         path is valid,
	/// \retval report information about the validation process. unused in
	///         this case,
	/// \return whether the whole path is valid.
        ///
        /// \notice Call the encapsulated PathValidation::validate.
	virtual bool validate (const PathPtr_t& path, bool reverse,
			       PathPtr_t& validPart,
			       PathValidationReportPtr_t& report);

        /// Set the encapsulated path validator.
        void innerValidation (const PathValidationPtr_t& pathValidation)
        {
          pathValidation_ = pathValidation;
        }

        /// Get the encapsulated path validator.
        const PathValidationPtr_t& innerValidation ()
        {
          return pathValidation_;
        }

        /// Set the graph of constraints
        void constraintGraph (const graph::GraphPtr_t& graph)
        {
          constraintGraph_ = graph;
        }

        /// Get the graph of constraints
        graph::GraphPtr_t constraintGraph () const
        {
          return constraintGraph_;
        }

        /// Create a new instance of this class.
        /// \param pathValidation a PathValidation that is responsible for collision
        //         checking.
        //  \param graph A pointer to the constraint graph.
        static GraphPathValidationPtr_t create (
            const PathValidationPtr_t& pathValidation);

        template <typename T>
          static GraphPathValidationPtr_t create (
              const pinocchio::DevicePtr_t& robot, const value_type& stepSize);

        void addObstacle (const hpp::core::CollisionObjectPtr_t&);

        /// Remove a collision pair between a joint and an obstacle
        /// \param joint the joint that holds the inner objects,
        /// \param obstacle the obstacle to remove.
        /// \notice collision configuration validation needs to know about
        /// obstacles. This virtual method does nothing for configuration
        /// validation methods that do not care about obstacles.
        virtual void removeObstacleFromJoint (const JointPtr_t& joint,
            const pinocchio::CollisionObjectPtr_t& obstacle)
        {
          assert (pathValidation_);
          pathValidation_->removeObstacleFromJoint (joint, obstacle);
        }

      protected:
        /// Constructor
        GraphPathValidation (const PathValidationPtr_t& pathValidation);

      private:
        /// Do validation regarding the constraint graph for PathVector
        bool impl_validate (const PathVectorPtr_t& path, bool reverse,
            PathPtr_t& validPart, PathValidationReportPtr_t& report);
        /// Do validation regarding the constraint graph for Path 
        bool impl_validate (const PathPtr_t& path, bool reverse,
            PathPtr_t& validPart, PathValidationReportPtr_t& report);
        /// The encapsulated PathValidation.
        PathValidationPtr_t pathValidation_;
        /// Pointer to the constraint graph.
        graph::GraphPtr_t constraintGraph_;
    };

    template <typename T>
      GraphPathValidationPtr_t GraphPathValidation::create
      (const pinocchio::DevicePtr_t& robot, const value_type& stepSize)
    {
      return GraphPathValidation::create (T::create (robot, stepSize));
    }
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPHPATHVALIDATOR_HH
