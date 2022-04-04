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


#ifndef HPP_MANIPULATION_GRAPHPATHVALIDATOR_HH
# define HPP_MANIPULATION_GRAPHPATHVALIDATOR_HH

# include <hpp/core/path-validation.hh>
# include <hpp/core/obstacle-user.hh>

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
    class HPP_MANIPULATION_DLLAPI GraphPathValidation :
      public PathValidation,
      public core::ObstacleUserInterface
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

        /// Add obstacle in the environment
        ///
        /// Dynamic cast inner path validation into
        /// hpp::core::ObstacleUserInterface and calls
        /// hpp::core::ObstacleUserInterface::addObstacle in case of success.
        void addObstacle (const hpp::core::CollisionObjectConstPtr_t& object)
        {
          shared_ptr<core::ObstacleUserInterface> oui =
            HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pathValidation_);
          if (oui) oui->addObstacle (object);
        }

        /// Remove a collision pair between a joint and an obstacle
        ///
        /// Dynamic cast inner path validation into
        /// hpp::core::ObstacleUserInterface and calls
        /// hpp::core::ObstacleUserInterface::removeObstacleFromJoint in case
        /// of success.
        void removeObstacleFromJoint (const JointPtr_t& joint,
            const pinocchio::CollisionObjectConstPtr_t& obstacle)
        {
          shared_ptr<core::ObstacleUserInterface> oui =
            HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pathValidation_);
          if (oui) oui->removeObstacleFromJoint (joint, obstacle);
        }

        /// \brief Filter collision pairs.
        ///
        /// Dynamic cast inner path validation into
        /// hpp::core::ObstacleUserInterface and calls
        /// hpp::core::ObstacleUserInterface::filterCollisionPairs in case of
        /// success.
        void filterCollisionPairs (const core::RelativeMotion::matrix_type& relMotion)
        {
          shared_ptr<core::ObstacleUserInterface> oui =
            HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pathValidation_);
          if (oui) oui->filterCollisionPairs (relMotion);
        }

        /// Set different security margins for collision pairs
        ///
        /// Dynamic cast inner path validation into
        /// hpp::core::ObstacleUserInterface and calls
        /// hpp::core::ObstacleUserInterface::setSecurityMargins in case of
        /// success.
        void setSecurityMargins(const matrix_t& securityMargins)
        {
          shared_ptr<core::ObstacleUserInterface> oui =
            HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pathValidation_);
          if (oui) oui->setSecurityMargins(securityMargins);
        }

        /// \copydoc hpp::core::ObstacleUserInterface::setSecurityMarginBetweenBodies
        ///
        /// Dynamic cast inner path validation into
        /// hpp::core::ObstacleUserInterface and calls
        /// hpp::core::ObstacleUserInterface::setSecurityMargins in case of
        /// success.
        void setSecurityMarginBetweenBodies(const std::string& body_a,
                                            const std::string& body_b,
                                            const value_type& margin)
        {
          shared_ptr<core::ObstacleUserInterface> oui =
            HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pathValidation_);
          if (oui) oui->setSecurityMarginBetweenBodies(body_a, body_b, margin);
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
