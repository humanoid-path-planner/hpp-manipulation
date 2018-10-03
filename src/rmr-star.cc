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


#include "hpp/manipulation/rmr-star.hh"

#include <boost/tuple/tuple.hpp>
#include <boost/assign/list_of.hpp>

#include <hpp/util/pointer.hh>
#include <hpp/util/timer.hh>
#include <hpp/util/assertion.hh>

#include <hpp/pinocchio/configuration.hh>

#include <hpp/core/path-validation.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/roadmap.hh>


#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/constraint-set.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/connected-component.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/roadmap-node.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/state-selector.hh"


namespace hpp {
  namespace manipulation {
    namespace {
      HPP_DEFINE_TIMECOUNTER(oneStep);
    }///end namespace
    ////////////////////////////////////////////////////////////////////////////
    RMRStarPtr_t RMRStar::create (const core::Problem& problem,
				  const core::RoadmapPtr_t& roadmap)
    {
      RMRStar* ptr;
      core::RoadmapPtr_t r2 = roadmap;
      RoadmapPtr_t r;
      try {
	const Problem& p = dynamic_cast < const Problem& > (problem);
	RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST (Roadmap, r2);
	ptr = new RMRStar (p, r);
      } catch (std::exception&) {
	if (!r)
	  throw std::invalid_argument ("The roadmap must be of type hpp::manipulation::Roadmap.");
	else
	  throw std::invalid_argument ("The problem must be of type hpp::manipulation::Problem.");
      }
      RMRStarPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }
    ////////////////////////////////////////////////////////////////////////////

    //creation of the ContactState class operator

    bool pluspetit (RMRStar::ContactState a, RMRStar::ContactState b){
      if (a.state<b.state){
      	return true;
      }
      else {
	return false;
      }
    }
    ////////////////////////////////////////////////////////////////////////////

    void RMRStar::computeTransitionMap ()
    {
      hppDout (info, "the algorithm enter compute Transition map ");

      // Access to the constraint graph
      graph::GraphPtr_t cg =pb_.constraintGraph ();

      const std::size_t nbComp = cg->nbComponents();

      //create an indexed table with a node as key and the loop edge as value
      for (std::size_t i=0; i<nbComp; ++i)
	{

	  const graph::GraphComponentPtr_t graphComp (cg->get(i));

	  graph::EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST(graph::Edge, graphComp));

	  if(edge)
	    {
	      if (edge->from()==edge->to())

		{
		  transition_[edge->from()]=edge;
		}
	    }
	}
    }

    ////////////////////////////////////////////////////////////////////////////

    std::vector<graph::StatePtr_t> RMRStar::extract_keys(RMRStar::TransitionMap_t input_map) {

      //create a vector with all the input_map keys

      std::vector<graph::StatePtr_t> retval;

      for (TransitionMap_t:: iterator i=input_map.begin() ; i!= input_map.end() ; ++i)  {
	retval.push_back(i->first);
      }
      return retval;
    }

    ////////////////////////////////////////////////////////////////////////////

    RMRStar::ContactState RMRStar::sampleContact ()
    {
      hppDout (info, "the algorithm enter sample contact ");

      graph::GraphPtr_t cg = pb_.constraintGraph ();
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
      std::size_t nbStates (transition_.size ());

      // Shoot random number
      std::size_t i_rand=rand() % nbStates;

      // Grap random state of the graph
      const graph::StatePtr_t s_rand = RMRStar::extract_keys(transition_)[i_rand];
      bool constraintApplied =false;

      int i=0;
      while (constraintApplied==false && i<=1000)
	{
	  //Shoot random configuration
	  q_rand_= shooter->shoot();

	  // Get state constraint of s_rand (configConstraint)
	  ConstraintSetPtr_t stateConfig = cg->configConstraint(s_rand);
	  constraintApplied = stateConfig->apply(*q_rand_);
	  i++;
	}

      if (constraintApplied ==false){
	std::ostringstream os;
	os << "fail to find a random configuration in state " << s_rand->name ();
	throw std::runtime_error (os.str ().c_str ());
      }

      //Get loop_edge constraints
      graph::EdgePtr_t loop_edge = transition_[s_rand];
      core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();

      copyEdgeConstraints_ = HPP_DYNAMIC_PTR_CAST(core::ConstraintSet, edgeConstraints->copy());

      // recovery of the edge steering method
      edgeSteeringMethod_ = loop_edge -> steeringMethod();

      // get right hand side of constraint from q_rand
      vector_t rhs =copyEdgeConstraints_->configProjector ()->rightHandSideFromConfig (*q_rand_);

      contactState_.state = s_rand;
      contactState_.rightHandSide = rhs;
      contactState_.loopEdgeConstraint=edgeConstraints;

      return contactState_;
    }

    ////////////////////////////////////////////////////////////////////////////

    void RMRStar::buildRoadmap ()
    {
      int counter =0;
      sampleContact();

      //copy the problem and pass the edge contraints
      core::Problem p (problem ());
      const unsigned long int maxIteration=100;

      interRoadmap_ = core::Roadmap::create(pb_.distance(),pb_.robot());
      assert (interRoadmap_);

      kPrm_=core::pathPlanner::kPrmStar::createWithRoadmap(p,interRoadmap_);
      kPrm_->core::pathPlanner::kPrmStar::maxIterations(maxIteration);

      RMRStar::ProblemAndRoadmap_t pbRoadmap (p,interRoadmap_) ;
      association_.insert(std::pair<RMRStar::ContactState ,RMRStar::ProblemAndRoadmap_t> (contactState_,pbRoadmap));

    while (counter<=1000)

      {
	hppDout (info, "enter in the while loop");
	kPrm_->oneStep();
	counter++;
    }
      p.initConfig(q_rand_);
      p.steeringMethod (edgeSteeringMethod_);
      p.constraints(copyEdgeConstraints_);


     }
      ////////////////////////////////////////////////////////////////////////////

    void  RMRStar::copyRoadmap ()
    {
      const core::Edges_t edges = interRoadmap_->edges ();

      // std ::vector<core::PathPtr_t> pathList;
      //construct a map with the starting and ending node of each edge
      //of the map as key and the path as value.

      for  (core::Edges_t::const_iterator itedge = edges.begin();
	    itedge != edges.end(); itedge++){

	RMRStar::NodeInOut_t pairNode((*itedge)->from(),(*itedge)->to());
	nodeMap_ [pairNode] =(*itedge)->path();
      }

      //copy the nodes and the edges in the global roadmap


      for  (RMRStar::NodeMap_t::const_iterator itnode = nodeMap_.begin();
	    itnode != nodeMap_.end(); itnode++){

	core::NodePtr_t node1 =roadmap()->addNode(itnode->first.first->configuration());
	core::NodePtr_t node2 =roadmap()->addNode(itnode->first.second->configuration());
	roadmap()->addEdge (node1, node2,itnode->second);

      }
    }

    ////////////////////////////////////////////////////////////////////////////

    void RMRStar::connectRoadmap () {

      PathProjectorPtr_t pathProjector (problem().pathProjector ());
      core::PathPtr_t projpath;
      //core::PathPtr_t projpath2;
      graph::GraphPtr_t cg =pb_.constraintGraph ();
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
      core::ConfigurationPtr_t q_rand;
      int i= 0;
      int k= 5;    //the number of closest nodes that we try to connect to the
                   //"intersection node
      int flag =0;
      int flag2 =0;

      //Flow through the maps stocked in the association_ map

      for  (RMRStar::AssociationMap_t::const_iterator itstate = association_.begin(); itstate != association_.end(); itstate++)
	{
	  graph::StatePtr_t state = (itstate->first).state;
	  core::ConstraintSetPtr_t constraintEdge = (itstate->first).loopEdgeConstraint;
	  bool constraintApplied = false;

	  // check if the states are different and are neighboors
	  graph::Edges_t connectedEdges = cg->getEdges(state, contactState_.state);
	  bool vectorEmpty = connectedEdges.empty ();

	  if (contactState_.state != state && vectorEmpty)
	    {
	      while (constraintApplied==false &&  i<=1000)
		{
		  //Shoot random configuration
		  q_rand= shooter->shoot();

		  // Get constraints of inter state and merge them
		  ConstraintSetPtr_t stateTransitConfig = cg->configConstraint(contactState_.state);
		  ConstraintSetPtr_t stateConfig = cg->configConstraint(state);
		  stateConfig -> addConstraint (stateTransitConfig);
		  stateConfig -> addConstraint (constraintEdge);
		  stateConfig -> addConstraint (contactState_.loopEdgeConstraint);
		  constraintApplied = stateConfig->apply(*q_rand);
		  i++;
		}

	      //test constraints have been applied to q_rand
	      if (!constraintApplied) {assert ("fail apply constraint to q_rand");}

	      //connect the interRoadmap to the nodeInter
	      core::NodePtr_t nodeInter =interRoadmap_->addNode (q_rand);
	      core::Nodes_t nearNodes = interRoadmap_->nearestNodes(q_rand,k);

	      //Travel through the k nearest neighboors of q_rand in interRoadmap
	      for (core::Nodes_t :: const_iterator itnode =nearNodes.begin(); itnode !=nearNodes.end(); itnode++ )
		{
		  core::ConfigurationPtr_t nodeConfig =(*itnode)->configuration();

		  //use the steering method of interRoadmap_ to find a path to the nodeInter
		  core::PathPtr_t path =(*edgeSteeringMethod_)(*q_rand,*nodeConfig);

		  if (path==NULL){
		    flag++;
		  }
		  else {
		    if (pathProjector->apply(path,projpath)){

		      NodeInOut_t connectionNodeInter (nodeInter,*itnode);
		      connectionmap_[connectionNodeInter]=projpath;
		    }
		    else {flag++;}
		  }
		  if (flag==k){assert ("not any path has been found between the nodeInter and the interRoadmap");}
		}

	      //connect the roadmaps selected in the associationmap to the nodeInter

	      core::RoadmapPtr_t selectRoadmap =itstate ->second.second;
	      core::Nodes_t nearestNodes = selectRoadmap->nearestNodes(q_rand,k);

	      for (core::Nodes_t :: const_iterator itnode =nearestNodes.begin();
		   itnode !=nearestNodes.end(); itnode++ )
		{
		  core::ConfigurationPtr_t nodeConfig =(*itnode)->configuration();
		  RMRStar::ProblemAndRoadmap_t PbRm =itstate-> second;
		  core::Problem pb = PbRm.first;
		  core::SteeringMethodPtr_t  sm =pb.steeringMethod();

		  //use the steering method of the selectRoadmap to find a path to the nodeInter
		  core::PathPtr_t path =(*sm)(*q_rand,*nodeConfig);

		  if (path==NULL){
		    flag2++;
		  }
		  else {
		    core::PathPtr_t projpath;
		    if (pathProjector->apply(path,projpath)){

		      NodeInOut_t connectionNodeInter (nodeInter,*itnode);
		      connectionmap_[connectionNodeInter]=projpath;
		    }
		    else { flag2++;}
		  }
		  if (flag2==k){assert ("not any path has been found between the nodeInter and the roadmap");}
		}
	    }
	}

    }
    ////////////////////////////////////////////////////////////////////////////

    void RMRStar::connectLeaves ()
    {
 for  (RMRStar::NodeMap_t::const_iterator itnode = nodeMap_.begin();
	    itnode != nodeMap_.end(); itnode++){

	core::NodePtr_t node1 =roadmap()->addNode(itnode->first.first->configuration());
	core::NodePtr_t node2 =roadmap()->addNode(itnode->first.second->configuration());
	roadmap()->addEdge (node1, node2,itnode->second);

      }

    }
    ////////////////////////////////////////////////////////////////////////////

    void RMRStar::startSolve ()
    {
      hppDout (info, "the algorithm reaches startsolve");
      computeTransitionMap ();

    step_= BUILD_ROADMAP;
    PathPlanner::startSolve ();
    }
    ////////////////////////////////////////////////////////////////////////////
    void RMRStar::oneStep ()
    {
      	int i=0;
      hppDout (info, "the algorithm reaches oneStep");
      // RMRStar::ContactState contactState;

      switch (step_)
	{
	case BUILD_ROADMAP:

	  buildRoadmap();
	  copyRoadmap ();
	if (association_.size()<1)
	  {
	    step_=BUILD_ROADMAP;
	  }
	else{
	  step_=CONNECT_ROADMAPS;
	}
	hppDout (info, "the algorithm reaches sample contact");
	break;

      case CONNECT_ROADMAPS:

	hppDout (info, "the algorithm reaches connect road map");
	connectRoadmap();
	connectLeaves();
	step_= BUILD_ROADMAP;
	if (i>=100) {step_=QUERY;}
	break;
      case QUERY:
	hppDout (info, "the algorithm reached the end");
	interrupt ();
	break;
      }

  }
    ////////////////////////////////////////////////////////////////////////////

    RMRStar::RMRStar (const core::Problem& problem,
		      const core::RoadmapPtr_t& roadmap) :
    core::PathPlanner (problem, roadmap),
    transition_ (), pb_ (static_cast <const manipulation::Problem& > (problem)),
    roadmap_ (HPP_STATIC_PTR_CAST (manipulation::Roadmap, roadmap))
  {
#ifndef NDEBUG
    dynamic_cast <const manipulation::Problem& > (problem);
    (HPP_DYNAMIC_PTR_CAST (manipulation::Roadmap, roadmap));

#endif
  }
    ////////////////////////////////////////////////////////////////////////////
    bool operator<(RMRStar:: ContactState c1 , RMRStar::ContactState c2)
    {
      return pluspetit (c1,c2);
    }

}//end namespace manipulation
}//end namespace hpp
