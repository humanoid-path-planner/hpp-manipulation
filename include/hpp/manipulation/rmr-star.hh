// Copyright 2018 (c) CNRS
// Authors: Margaux Sebal, Florent Lamiraux

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

# ifndef HPP_MANIPULATION_RMR_STAR_HH
# define HPP_MANIPULATION_RMR_STAR_HH


#include <hpp/core/path-planner.hh>
#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/fwd.hh"

namespace hpp {
  namespace manipulation {

    /// Implementation of RMR star
    ///
    /// This class implements algorithm RMR* as described in
    /// "Optimal Sampling-Based Manipulation Planner" by P. Schmitt, W. Neubauer,
    /// K. Wurm, G. Wichert, and W. Burgard.
    class HPP_MANIPULATION_DLLAPI RMRStar :
      public core::PathPlanner
    {
      public:

        /// Create an instance and return a shared pointer to the instance
        /// \param problem reference to the problem to be solved,
        /// \param roadmap roadmap to be expanded.
        static RMRStarPtr_t create (const core::Problem& problem,
            const core::RoadmapPtr_t& roadmap);

	 /// One step of extension.
        ///
        /// A set of constraints is chosen using the graph of constraints.
        /// A constraint extension is done using a chosen set.
        ///
        virtual void oneStep ();

	typedef std::multimap <constraints::ImplicitPtr_t,constraints::vector_t> RhsMap_t;
	RhsMap_t RhsMap_;

	struct ContactState {
	public:
	  ContactState () : state_ (), rightHandSide_ (), loopEdgeConstraint_ (), config_ (), rhsMap_()
	  {
	  }
	  ContactState (const graph::StatePtr_t& state,
			ConfigurationIn_t config,
			const core::ConstraintSetPtr_t& constraints) :
	    state_ (state), rightHandSide_ (),
	    loopEdgeConstraint_ (constraints), config_(config),rhsMap_()
	  {

	    assert (loopEdgeConstraint_);
	    assert (loopEdgeConstraint_->configProjector ());
	    rightHandSide_ = loopEdgeConstraint_->configProjector ()->
	      rightHandSideFromConfig (config);
	    core::NumericalConstraints_t num =constraints->configProjector ()->solver().numericalConstraints();
	      constraints::solver::BySubstitution solver
	    ( constraints->configProjector ()-> solver ());
	    for (std::size_t i=0 ; i<num.size() ; i++)
	      {
		constraints::ImplicitPtr_t function = num[i];
		constraints::vectorOut_t rhs= function->nonConstRightHandSide();
		 solver.getRightHandSide(num[i],rhs);

		rhsMap_.insert
		  (std::pair<constraints::ImplicitPtr_t,constraints::vectorIn_t>
		   (function,rhs));
		 }

	  }
	  const graph::StatePtr_t& state () const
	  {
	    assert (state_);
	    return state_;
	  }
	  const constraints::vector_t& rightHandSide () const
	  {
	    assert (state_);
	    return rightHandSide_;
	  }
	  const core::ConstraintSetPtr_t& constraints () const
	  {
	    assert (state_);
	    return loopEdgeConstraint_;
	  }
	  const Configuration_t& config () const
	  {
	    assert (state_);
	    return config_;
	  }

	  const RhsMap_t& rhsMap () const
	  {
	    assert (state_);

	    return rhsMap_;
	  }

	private:
	  graph::StatePtr_t state_;
	  constraints::vector_t rightHandSide_;
	  core::ConstraintSetPtr_t loopEdgeConstraint_;
	  Configuration_t config_;
	  RhsMap_t rhsMap_;
	};

	bool smaller (const RMRStar::ContactState& a,
		      const RMRStar::ContactState& b);

     protected:
        /// Protected constructor
        RMRStar (const core::Problem& problem,
		 const core::RoadmapPtr_t& roadmap);

     private:


	typedef std::map <graph::StatePtr_t, graph::EdgePtr_t> TransitionMap_t;
	TransitionMap_t transition_;

	typedef std::pair<core::Problem,core::RoadmapPtr_t>ProblemAndRoadmap_t;

	typedef std::multimap <ContactState , ProblemAndRoadmap_t> AssociationMap_t;
	AssociationMap_t association_;



	/// Pointer to the problem
        const Problem& pb_;
	/// Pointer to the roadmap
        const RoadmapPtr_t roadmap_;



	///////////////////////////////////////////////////////////////////
	///Functions declaration
	//////////////////////////////////////////////////////////////////

	ContactState sampleContact ();

	void computeTransitionMap ();

	std::vector<graph::StatePtr_t> extract_keys(TransitionMap_t input_map);


	void startSolve ();

	void buildRoadmap ();

	void copyRoadmap ();

	void connectRoadmap ();

	void associationmap ();

	void storeRhs ();

	bool rhsNull (vector_t rhs);

	pinocchio::value_type biggerThreshold
	  ( constraints::solver::BySubstitution solver1,constraints::solver::BySubstitution solver2);


	//pointer to the kPrmStar method
	core::pathPlanner::kPrmStarPtr_t kPrm_;

	//Pointer to the copy of the loop-edge constraints
	core::ConstraintSetPtr_t copyEdgeConstraints_;

	//Pointer to the edge steering method
	core::SteeringMethodPtr_t edgeSteeringMethod_;

	// intermediary roadmap constructor
	core::RoadmapPtr_t interRoadmap_;

	//Random config used as q_init
	mutable ConfigurationPtr_t q_rand_;

	//current contactState
	RMRStar::ContactState contactState_;

	//Pointer to the graph problem
	graph::GraphPtr_t graph_;


	enum STEP {
	  BUILD_ROADMAP,
	  CONNECT_ROADMAPS,
	  QUERY
	};

	STEP step_;

    }; // class RMRStar
    bool operator< (const RMRStar::ContactState& c1 ,
		    const RMRStar::ContactState& c2);

  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_RMR_Star_HH
