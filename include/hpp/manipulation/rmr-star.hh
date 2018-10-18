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

#include <hpp/statistics/success-bin.hh>

#include "hpp/manipulation/graph/statistics.hh"

#include "hpp/manipulation/config.hh"
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

	struct ContactState {
	public:
	  ContactState () : state_ (), rightHandSide_ (), loopEdgeConstraint_ ()
	  {
	  }
	  ContactState (const graph::StatePtr_t& state,
			ConfigurationIn_t config,
			const core::ConstraintSetConstPtr_t& constraints) :
	    state_ (state), rightHandSide_ (),
	    loopEdgeConstraint_ ()
	  {
	    loopEdgeConstraint_ = HPP_DYNAMIC_PTR_CAST(core::ConstraintSet,
						       constraints->copy());
	    assert (loopEdgeConstraint_);
	    assert (loopEdgeConstraint_->configProjector ());
	    rightHandSide_ =loopEdgeConstraint_->configProjector ()->
	      rightHandSideFromConfig (config);
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
	  const core::ConstraintSetPtr_t constraints () const
	  {
	    assert (state_);
	    return loopEdgeConstraint_;
	  }
	private:
	  graph::StatePtr_t state_;
	  constraints::vector_t rightHandSide_;
	  core::ConstraintSetPtr_t loopEdgeConstraint_;
	};
	bool smaller (const RMRStar::ContactState& a,
		      const RMRStar::ContactState& b);

     protected:
        /// Protected constructor
        RMRStar (const core::Problem& problem,
		 const core::RoadmapPtr_t& roadmap);

     private:

	void computeTransitionMap ();

	typedef std::map <graph::StatePtr_t, graph::EdgePtr_t> TransitionMap_t;
	TransitionMap_t transition_;

	typedef std::pair<core::NodePtr_t,core::NodePtr_t>NodeInOut_t;
	typedef std::map <NodeInOut_t, core::PathPtr_t> NodeMap_t;
	NodeMap_t nodeMap_;


	NodeMap_t connectionmap_;

	typedef std::pair<core::Problem,core::RoadmapPtr_t>ProblemAndRoadmap_t;

	typedef std::multimap <ContactState , ProblemAndRoadmap_t> AssociationMap_t;
	AssociationMap_t association_;

	//Return the vector of the transition map keys
	std::vector<graph::StatePtr_t> extract_keys(TransitionMap_t input_map);

	/// Pointer to the problem
        const Problem& pb_;
	/// Pointer to the roadmap
        const RoadmapPtr_t roadmap_;


	ContactState sampleContact ();

	void startSolve ();

	void buildRoadmap ();

	void copyRoadmap ();

	void connectLeaves ();

	void connectRoadmap ();


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
