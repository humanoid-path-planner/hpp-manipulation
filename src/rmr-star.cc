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

#define HPP_DEBUG

#include "hpp/manipulation/rmr-star.hh"

#include <hpp/pinocchio/configuration.hh>

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/roadmap.hh>


#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap.hh"


namespace hpp {
  namespace manipulation {

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

    // ContactState class operator

    bool smaller (RMRStar::ContactState a, RMRStar::ContactState b){
      if (a.state () < b.state ()){
      	return true;
      }
      else{return false; }

      if (a.rightHandSide ().size () < b.rightHandSide ().size ()){
	return true;
      }
      else {
	return false;
      }
      for (int i=0; i < a.rightHandSide ().size(); ++i)	{
	if (a.rightHandSide ()[i]<b.rightHandSide ()[i]){
	  return true;
	}
	else {
	  return false;
	}
      }
    }
    ////////////////////////////////////////////////////////////////////////////
    //Create a map which associate the graph's states and their edge loop

    void RMRStar::computeTransitionMap ()
    {
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

    //Create a vector with the graph's states

    std::vector<graph::StatePtr_t> RMRStar::extract_keys
    (RMRStar::TransitionMap_t input_map) {

      //create a vector with all the input_map keys

      std::vector<graph::StatePtr_t> retval;

      for (TransitionMap_t:: iterator i=input_map.begin() ;
	   i!= input_map.end() ; ++i)  {

	retval.push_back(i->first);
      }
      return retval;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Shot a random config in a random state and create a ContactState
    //with the state, the configuration and the constraints applied

    RMRStar::ContactState RMRStar::sampleContact ()
    {
      graph::GraphPtr_t cg = pb_.constraintGraph ();
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
      std::size_t nbStates (transition_.size ());

      // Shoot random number
      std::size_t i_rand=rand() % nbStates;

      // Grap random state of the graph
      const graph::StatePtr_t s_rand =
	RMRStar::extract_keys(transition_)[i_rand];

      bool constraintApplied =false;
      bool valid =false;
      core:: ValidationReportPtr_t validationReport;
      core::ConfigValidationsPtr_t configValidations
	(problem ().configValidations ());
      int i=0;

      while (valid==false && i<=1000)
	{
	  //Shoot random configuration
	  q_rand_= shooter->shoot();

	  // Get state constraint of s_rand (configConstraint)
	  ConstraintSetPtr_t stateConfig = cg->configConstraint(s_rand);
	  constraintApplied = stateConfig->apply(*q_rand_);
	  if (constraintApplied!=true){
	    valid=false;
	  i++;
	  continue;
	  }
	  valid = configValidations->validate (*q_rand_, validationReport);
	  i++;
	    }

      if (constraintApplied ==false){
	hppDout (info,"fail to find a random configuration in state "
		 << s_rand->name ());
      }

      //Get loop_edge constraints
      graph::EdgePtr_t loop_edge = transition_[s_rand];
      core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();

      // recovery of the edge steering method
      edgeSteeringMethod_ = loop_edge -> steeringMethod();

      // get right hand side of constraint from q_rand
      ContactState contactState (s_rand, *q_rand_, edgeConstraints);
      return contactState;
    }

    ////////////////////////////////////////////////////////////////////////////

        //build a roadmap on the ContactState leaf


    void RMRStar::buildRoadmap ()

    {
      using core::pathPlanner::kPrmStar;
      copyEdgeConstraints_=contactState_.constraints();
              hppDout (info,"connect states "<<contactState_.state()->name());
	      hppDout (info,"RHS="<<contactState_.rightHandSide());
      //copy the problem and pass the edge contraints
      core::Problem p (problem ());
      p.initConfig(q_rand_);
      p.steeringMethod (edgeSteeringMethod_);
      p.constraints(copyEdgeConstraints_);
      p.resetGoalConfigs();

      kPrm_=kPrmStar::createWithRoadmap(p,interRoadmap_);
      interRoadmap_->clear();

      kPrm_->startSolve();


      kPrmStar::STATE kPrmState;

      do {
        kPrmState= kPrm_->getComputationState();
         kPrm_->oneStep();

      }	while (kPrmState != kPrmStar::CONNECT_INIT_GOAL);

    }
    ////////////////////////////////////////////////////////////////////////////
    //Copy the interRoadmap (current map) on the main roadmap

    void RMRStar::copyRoadmap ()
    {
      const core::Edges_t& edges = interRoadmap_->edges ();
      core::NodePtr_t node1;
      core::NodePtr_t node2;

      //construct a map with the starting and ending node of each edge
      //of the map as key and the path as value.

      for  (core::Edges_t::const_iterator itedge = edges.begin();
	    itedge != edges.end(); itedge++){

	node1=roadmap_->addNode((*itedge)->from()->configuration());
	node2=roadmap_->addNode((*itedge)->to()->configuration());

	roadmap()->addEdge (node1, node2,(*itedge)->path());

      }

    }

    ////////////////////////////////////////////////////////////////////////////
    //find an internode and connect the interRoadmaps to it

    void RMRStar::connectRoadmap () {
      hppDout(info,"enter in connectRoadMap");

      using constraints::solver::HierarchicalIterative;

      PathProjectorPtr_t pathProjector (pb_.pathProjector ());
      core::PathPtr_t projpath;
      graph::GraphPtr_t cg =pb_.constraintGraph ();
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
      core::ConfigurationPtr_t q_inter;
      core:: ValidationReportPtr_t validationReport;
      int i= 0;
      int k= 5;    //the number of closest nodes that we try to connect to the
                   //intersection node
      int flag =0;
      int max_iter=100;
      core::PathPtr_t path;

      // Iterate through the maps stocked in the association_ map

      for  (RMRStar::AssociationMap_t::const_iterator itstate =
	      association_.begin(); itstate != association_.end(); itstate++)
	{
	  graph::StatePtr_t state = (itstate->first).state ();
	  core::ConstraintSetPtr_t constraintEdge =
	    (itstate->first).constraints ();
	  constraints::vector_t rhs=
	    (itstate->first).rightHandSide ();
	  Configuration_t config = (itstate->first).config ();

	  // check if the states are different and neighboors or are similar
	  graph::Edges_t connectedEdges =
	    cg->getEdges(state,contactState_.state ());
	  bool vectorEmpty = connectedEdges.empty ();
	  bool valid=false;

	  //Apply constraint
	  HierarchicalIterative::Status constraintApplied=
	    HierarchicalIterative::INFEASIBLE;

	  core::ConfigValidationsPtr_t configValidations
	    (pb_.configValidations ());



	  if ((contactState_.state () !=state && !vectorEmpty)||
	      (contactState_.state () ==state &&
	       contactState_.rightHandSide()==rhs) )
	    {

	      // Get constraints of inter state
	      const core::ConstraintSetPtr_t& constraintTransitConfig =
		contactState_.constraints ();

	      //get the solver of the stateConfig
	      assert (constraintTransitConfig);

	      constraints::solver::BySubstitution solver
		(constraintTransitConfig->configProjector ()->solver ());

	      //Get the numerical constraints of constraintEdge
	      const constraints::NumericalConstraints_t& constraints =
	      	constraintEdge->configProjector ()->solver().
		numericalConstraints();

	      //Get the numerical constraints of Transitconfig
	      const constraints::NumericalConstraints_t& transitConstraints =
	      	constraintTransitConfig->configProjector ()->solver().
	      	numericalConstraints();

	      //Copy the constraints and the right hand side in the new solver
	      for (std::size_t j=0; j<constraints.size(); j++){
	        solver.add(constraints[j]);
	      }
	      for (std::size_t j=0; j<transitConstraints.size(); j++){
		solver.rightHandSideFromConfig (transitConstraints[j],
						contactState_.config());
	      }
	      for (std::size_t j=0; j<constraints.size(); j++){
		solver.rightHandSideFromConfig (constraints[j], config);
	      }

	      // hppDout (info,"solver"<< solver);
	      while ((valid==false) && i<max_iter)
		{
		  //Shoot random configuration
		  q_inter= shooter->shoot();
		  constraintApplied = solver.solve(*q_inter);
		  if (constraintApplied != HierarchicalIterative::SUCCESS) {
		    valid=false;
		    ++i;
		    continue;
		  }
		  valid = configValidations->validate (*q_inter,
						       validationReport);
		  i++;
		}
	      hppDout (info,"connect states "<<contactState_.state()->name()<<
		       "  state "<<state->name());
	      hppDout (info,"RHS="<<contactState_.rightHandSide()<<" rhs  "<<
		       rhs);
	      hppDout (info,"q_inter="<< pinocchio::displayConfig (*q_inter));
	      hppDout (info,"constraintApplied="<<constraintApplied);
	      hppDout (info,"i="<<i);

	      //test constraints have been applied to q_inter
	      if (i==max_iter) {
		hppDout (info,"WARNING i reached max_iter, not any connect node have been found");
	      }
	      else
		{
		  i=0;
		  //connect the interRoadmap to the nodeInter
		  core::Nodes_t nearNodes = interRoadmap_->
		    nearestNodes(q_inter,k);
		  core::NodePtr_t nodeConnect=roadmap_->addNode (q_inter);



		  //Travel through the k nearest neighboors of q_inter in
		  //interRoadmap
		  for (core::Nodes_t :: const_iterator itnode = nearNodes.
			 begin(); itnode !=nearNodes.end(); itnode++ )
		    {
		      core::ConfigurationPtr_t nodeConfig =
			(*itnode)->configuration();

		      hppDout (info, " node1 " <<  pinocchio::displayConfig(*nodeConfig));
		      assert (edgeSteeringMethod_->constraints ()->isSatisfied (*q_inter));
		      assert (edgeSteeringMethod_->constraints ()->isSatisfied (*nodeConfig));
		      //use the steering method of interRoadmap_ to find a path to the nodeInter
		      path =(*edgeSteeringMethod_)(*q_inter,*nodeConfig);


		      if (path==NULL){
			flag++;
			hppDout (info,"path1=NULL");

		      }
		      else {
			if (pathProjector->apply(path,projpath)){
			  core::NodePtr_t node=
			    roadmap_->addNode (nodeConfig);
			  roadmap_->addEdges (nodeConnect,node,projpath);
			  hppDout (info,"edge created");

			}
			else {flag++;}
		      }
		      if (flag==k){hppDout (info,"not any path has been found between the nodeInter and the interRoadmap with state="<<contactState_.state()->name()<<"RHS="<<contactState_.rightHandSide());}
		    }
		  hppDout (info,"f1="<<flag);

		  flag=0;

        //connect the roadmaps selected in the associationmap to the nodeInter
		  //Get select roadmap steering method
		  core::RoadmapPtr_t selectRoadmap =itstate -> second.second;
		  core::Nodes_t nearestNodes = selectRoadmap->
		    nearestNodes(q_inter,k);

		  RMRStar::ProblemAndRoadmap_t PbRm =itstate-> second;
		  core::Problem pb = PbRm.first;
		  core::SteeringMethodPtr_t  sm =pb.steeringMethod();


		  for (core::Nodes_t :: const_iterator itnode =nearestNodes.
			 begin(); itnode !=nearestNodes.end(); itnode++ )
		    {
		      core::ConfigurationPtr_t nodeConfig =
			(*itnode)->configuration();


		      //use the steering method of the selectRoadmap to find a path to the nodeInter
		      hppDout (info, " node "<<  pinocchio::displayConfig(*nodeConfig));
		      // assert (sm->constraints ()->isSatisfied (*q_inter));
		      //assert (sm->constraints ()->isSatisfied (*nodeConfig));
		      path =(*sm)(*q_inter,*nodeConfig);


		      if (path==NULL){
			flag++;
			hppDout (info,"path2=NULL");

		      }
		      else {
			//core::PathPtr_t projpath;
			if (pathProjector->apply(path,projpath)){
			  core::NodePtr_t node=roadmap_->addNode(nodeConfig);
			  roadmap_->addEdges (nodeConnect,node,projpath);
			  hppDout (info,"edge created");

			}
			else { flag++;}
		      }
		      if (flag==k){hppDout (info,"not any path has been found between the nodeInter and the roadmap with state="<<state->name()<<"RHS="<<rhs);}
		    }
		  hppDout (info,"f2="<<flag);

		  flag=0;

		}
	    }
	}
      associationmap();
    }


    //////////////////////////////////////////////////////////////////////////
    void RMRStar::associationmap ()
    {

       //copy the problem and pass the edge contraints
      core::Problem p (problem ());
      p.initConfig(q_rand_);
      p.steeringMethod (edgeSteeringMethod_);
      p.constraints(copyEdgeConstraints_);
      core::RoadmapPtr_t r = core::Roadmap::create(p.distance(),p.robot());
      r->clear();

      for (core::Nodes_t :: const_iterator itnode =interRoadmap_->nodes().begin(); itnode !=interRoadmap_->nodes().end(); itnode++ )
	{
	  r->addNode((*itnode)->configuration());
	}

      RMRStar::ProblemAndRoadmap_t pbRoadmap (p,r) ;

      //complete the map with the association ContactState/ProblemAndRoadmap
       association_.insert
	(std::pair<RMRStar::ContactState,RMRStar::ProblemAndRoadmap_t>
	(contactState_,pbRoadmap));

       hppDout(info,"association_ size = " <<association_.size());

    }

    ////////////////////////////////////////////////////////////////////////////


    void RMRStar::startSolve ()
    {
      hppDout (info,"Start Solve");
      PathPlanner::startSolve ();
      computeTransitionMap ();

      //Set parameter and create interRoadmap
      interRoadmap_ = core::Roadmap::create(pb_.distance(),pb_.robot());
      interRoadmap_->clear();
      q_rand_=roadmap()->initNode ()->configuration();

      //Set ContactState
      graph::GraphPtr_t graph =pb_.constraintGraph ();
      graph::StatePtr_t stateInit=
	graph->getState(*(q_rand_));
      graph::EdgePtr_t loop_edge = transition_[stateInit];
      core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();
      RMRStar::ContactState contactStateInit
	(stateInit, *q_rand_, edgeConstraints);
      edgeSteeringMethod_ = loop_edge -> steeringMethod();
      contactState_=contactStateInit;

      buildRoadmap();
      copyRoadmap ();
      associationmap();

      hppDout (info,"first association Init"<<association_.size());

      for (core::NodeVector_t::const_iterator itn
	     (roadmap_->goalNodes ().begin ());
	   itn != roadmap_->goalNodes ().end (); ++itn) {


	interRoadmap_ -> clear();

	//Set ContactState_
	q_rand_=(*itn)->configuration();
	graph::StatePtr_t stateGoal=graph->getState(*q_rand_);
	loop_edge = transition_[stateGoal];
	edgeConstraints =loop_edge->pathConstraint();
	RMRStar::ContactState contactStateGoal
	  (stateGoal,*q_rand_, edgeConstraints);
	contactState_=contactStateGoal;

	buildRoadmap();
	copyRoadmap ();
	hppDout(info,"First connect");
	connectRoadmap();
      hppDout (info,"association Goal"<<association_.size());


      }
            step_=BUILD_ROADMAP;


    }
    ////////////////////////////////////////////////////////////////////////////
    void RMRStar::oneStep ()
    {
      int i=0;

      switch (step_)
	{

	case BUILD_ROADMAP:
	  contactState_ = sampleContact();
	  buildRoadmap();
	  copyRoadmap ();
	  step_=CONNECT_ROADMAPS;

	  break;

	case CONNECT_ROADMAPS:

	  connectRoadmap();

	  step_= BUILD_ROADMAP;
	  if (i>=10) {step_=QUERY;}
	  i++;
	  break;
	case QUERY:
	  hppDout (info, "Max iteration reached");
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
    bool operator< (const RMRStar::ContactState& c1 ,
		    const RMRStar::ContactState& c2)
    {
      return smaller (c1,c2);
    }

}//end namespace manipulation
}//end namespace hpp
