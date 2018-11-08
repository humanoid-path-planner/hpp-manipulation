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

    //creation of the ContactState class operator

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

    std::vector<graph::StatePtr_t> RMRStar::extract_keys
    (RMRStar::TransitionMap_t input_map) {

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
      graph::GraphPtr_t cg = pb_.constraintGraph ();
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
      std::size_t nbStates (transition_.size ());

      // Shoot random number
      std::size_t i_rand=rand() % nbStates;
      hppDout(info, "Sampled contact " << i_rand);

      // Grap random state of the graph
      const graph::StatePtr_t s_rand = RMRStar::extract_keys(transition_)[i_rand];

      bool constraintApplied =false;
      bool valid =false;
      core:: ValidationReportPtr_t validationReport;
      core::ConfigValidationsPtr_t configValidations (problem ().configValidations ());
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
	std::ostringstream os;
	os << "fail to find a random configuration in state " << s_rand->name ();
	throw std::runtime_error (os.str ().c_str ());
      }
      hppDout (info,"q_rand="<< pinocchio::displayConfig (*q_rand_));

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

    void RMRStar::buildRoadmap ()
    {
      using core::pathPlanner::kPrmStar;
      contactState_ = sampleContact();
      copyEdgeConstraints_=contactState_.constraints();
      interRoadmap_->clear();
      hppDout (info,"state build="<<contactState_.state()->name());
      hppDout (info,"RHS="<<contactState_.rightHandSide());
      
      //copy the problem and pass the edge contraints
      core::Problem p (problem ());
      p.initConfig(q_rand_);
      p.steeringMethod (edgeSteeringMethod_);
      p.constraints(copyEdgeConstraints_);

      kPrm_=kPrmStar::createWithRoadmap(p,interRoadmap_);
      
      kPrm_->startSolve();
      
      kPrmStar::STATE kPrmState;

      do {
        kPrmState= kPrm_->getComputationState();
        kPrm_->oneStep();

      }	while (kPrmState != kPrmStar::CONNECT_INIT_GOAL);
    }
    ////////////////////////////////////////////////////////////////////////////

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
      //hppDout (info,"nodes roadmap="<<roadmap_->nodes().size());

    }

    ////////////////////////////////////////////////////////////////////////////
    /////find an internode and connect the interRoadmaps to it
    ///////////////////////////////////////////////////////////////////////////

    void RMRStar::connectRoadmap () {
      hppDout(info,"enter in connectRoadMap");
      
      using constraints::solver::HierarchicalIterative;

      PathProjectorPtr_t pathProjector (pb_.pathProjector ());
      core::PathPtr_t projpath;
      graph::GraphPtr_t cg =pb_.constraintGraph ();
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
      core::ConfigurationPtr_t q_rand;
      core:: ValidationReportPtr_t validationReport;
      int i= 0;
      int k= 5;    //the number of closest nodes that we try to connect to the
                   //intersection node
      int flag =0;
      int max_iter=100;
      core::PathPtr_t path;

      // Iterate through the maps stocked in the association_ map

      for  (RMRStar::AssociationMap_t::const_iterator itstate = association_.begin(); itstate != association_.end(); itstate++)
	{
	  graph::StatePtr_t state = (itstate->first).state ();
	  core::ConstraintSetPtr_t constraintEdge =
	    (itstate->first).constraints ();
	  constraints::vector_t rhs=
	    (itstate->first).rightHandSide ();
	  Configuration_t config = (itstate->first).config ();

	  // check if the states are different and are neighboors or are similar
	  graph::Edges_t connectedEdges = cg->getEdges(state,
						       contactState_.state ());
	  bool vectorEmpty = connectedEdges.empty ();
	  bool valid=false;
	  HierarchicalIterative::Status constraintApplied=
	    HierarchicalIterative::INFEASIBLE;
	  core::ConfigValidationsPtr_t configValidations (pb_.
							  configValidations ());
	  hppDout (info,"connect states "<<contactState_.state()->name()<<"  state "<<state->name());
	  
	  hppDout (info,"RHS="<<contactState_.rightHandSide()<<" rhs  "<<rhs);
	  if ((contactState_.state () !=state && !vectorEmpty)||
	      (contactState_.state () ==state && contactState_.rightHandSide()==rhs) )
	    {
		      
	      // Get constraints of inter state
	      const core::ConstraintSetPtr_t& constraintTransitConfig = contactState_.constraints ();

	      //get the solver of the stateConfig
	      assert (constraintTransitConfig);

	      constraints::solver::BySubstitution solver
		(constraintTransitConfig->configProjector ()->solver ());
	      	     
	      //Get the numerical constraints of the constraintEdge
	      const constraints::NumericalConstraints_t& constraints =
	      	constraintEdge->configProjector ()->solver().numericalConstraints();

	      //Get the numerical constraints of the Transit config constraintSet
	      const constraints::NumericalConstraints_t& transitConstraints =
	      	constraintTransitConfig->configProjector ()->solver().
	      	numericalConstraints();

	      //Copy the constraints and the right hand side in the new solver 
	      for (std::size_t j=0; j<constraints.size(); j++){
	        solver.add(constraints[j]);
	      }
	      for (std::size_t j=0; j<transitConstraints.size(); j++){
		solver.rightHandSideFromConfig (transitConstraints[j], contactState_.config());
	      }
	      for (std::size_t j=0; j<constraints.size(); j++){
		solver.rightHandSideFromConfig (constraints[j], config);
	      }
	    
	      hppDout (info,"solver"<< solver);
	      while ((valid==false) && i<max_iter)
		{
		  //Shoot random configuration
		  q_rand= shooter->shoot();
		  constraintApplied = solver.solve(*q_rand);
		  if (constraintApplied != HierarchicalIterative::SUCCESS) {
		    valid=false;
		    ++i;
		    continue;
		  }
		  valid = configValidations->validate (*q_rand, validationReport);
		  i++;
		}
	      hppDout (info,"q_rand="<< pinocchio::displayConfig (*q_rand));
	      hppDout (info,"constraintApplied="<<constraintApplied);
	      hppDout (info,"i="<<i);
	      
	      //test constraints have been applied to q_rand
	      if (i==max_iter) {
		hppDout (info,"WARNING i reached max_iter, not any connect node have been found");
	      }
	      else
		{
		  i=0;
		  //connect the interRoadmap to the nodeInter
		  core::Nodes_t nearNodes = interRoadmap_->nearestNodes(q_rand,k);
		  core::NodePtr_t nodeConnect=roadmap_->addNode (q_rand);
		  hppDout (info,"connect Roadmap creation internodes");
		  
		  
		  //Travel through the k nearest neighboors of q_rand in interRoadmap
		  for (core::Nodes_t :: const_iterator itnode =nearNodes.begin(); itnode !=nearNodes.end(); itnode++ )
		    {
		      core::ConfigurationPtr_t nodeConfig =(*itnode)->configuration();
		      
		      //use the steering method of interRoadmap_ to find a path to the nodeInter
		      path =(*edgeSteeringMethod_)(*q_rand,*nodeConfig);
		     
		      
		      if (path==NULL){
			flag++;
		      }
		      else {
			if (pathProjector->apply(path,projpath)){
			  core::NodePtr_t node= roadmap_->addNode (nodeConfig);
			  roadmap_->addEdges (nodeConnect,node,projpath);
				  
			}
			else {flag++;}
		      }
		      if (flag==k){hppDout (info,"not any path has been found between the nodeInter and the interRoadmap");}
		    }
		  flag=0;
		  
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
		      path =(*sm)(*q_rand,*nodeConfig);
		      nodeConfig =(*itnode)->configuration();
		      
		      if (path==NULL){
			flag++;
		      }
		      else {
			//core::PathPtr_t projpath;
			if (pathProjector->apply(path,projpath)){
			  core::NodePtr_t node= roadmap_->addNode (nodeConfig);
			  roadmap_->addEdges (nodeConnect,node,projpath);
			 
			}
			else { flag++;}
		      }
		      if (flag==k){hppDout (info,"not any path has been found between the nodeInter and the roadmap");}
		    }
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

      RMRStar::ProblemAndRoadmap_t pbRoadmap (p,interRoadmap_) ;
      
      //complete the map with the association ContactState/ProblemAndRoadmap
       association_.insert
	(std::pair<RMRStar::ContactState,RMRStar::ProblemAndRoadmap_t>
	(contactState_,pbRoadmap));

       hppDout(info,"association_ size = " <<association_.size());

    }

    ////////////////////////////////////////////////////////////////////////////

    /*void RMRStar:: tryToConnectInitAndGoal ()
    {
      hppDout (info,"Enter in tryConnectInitAndGoal");
      PathProjectorPtr_t pathProjector (problem().pathProjector ());
      int k=5;
      core:: NodePtr_t initNode (roadmap ()->initNode ());
      core::Nodes_t nearNodes = interRoadmap_->nearestNodes((initNode->configuration()),k);
      core::PathPtr_t path;
      core::PathPtr_t projpath;

      int flag=0;
      hppDout (info,"near nodes size init"<<nearNodes.size());

      //Travel through the k nearest neighboors of q_rand in interRoadmap
      for (core::Nodes_t :: const_iterator itnode =nearNodes.begin(); itnode !=nearNodes.end(); itnode++ )
	{
	  core::ConfigurationPtr_t nodeConfig =(*itnode)->configuration();
	  

	  //path =(*edgeSteeringMethod_)(*(initNode->configuration()),*nodeConfig);
	  core::SteeringMethodPtr_t sm=problem().steeringMethod();
	  path =(*sm)(*(initNode->configuration()),*nodeConfig);
	  hppDout(info,"initconfig"<<*(initNode->configuration()));
	  hppDout(info,"otherconfig"<<*nodeConfig);

	  if (path==NULL){
	    flag++;
	    hppDout(info,"pathNull");
	  }
	  else {
	    if (pathProjector->apply(path,projpath)){
	      core::NodePtr_t node= roadmap()->addNode (nodeConfig);
	       hppDout (info,"init add edge");
	      roadmap()->addEdges (initNode,node,projpath);
	    }
	    else {flag++;}
	  }
	  if (flag==k){hppDout (info,"not any path has been found between the initNode and the interRoadmap");}
	}
      flag=0;
      hppDout (info,"goalnodes size "<<roadmap()->goalNodes().size());
      for (core::NodeVector_t::const_iterator itn
	     (roadmap ()->goalNodes ().begin ());
	   itn != roadmap ()->goalNodes ().end (); ++itn) {
	
	nearNodes = interRoadmap_ -> nearestNodes((*itn)->configuration(),k);

	hppDout (info,"near nodes size goal"<<nearNodes.size());

	for (core::Nodes_t :: const_iterator itnode =nearNodes.begin();
	     itnode !=nearNodes.end(); itnode++ )
	  {
	    core::ConfigurationPtr_t nodeConfig =(*itnode)->configuration();
	    
	    //use the steering method of interRoadmap_ to find a path to the nodeInter
	    path =(*edgeSteeringMethod_)(*((*itn)->configuration()),*nodeConfig);
	    
	    if (path==NULL){
	      flag++;
	    }
	    else {
	      if (pathProjector->apply(path,projpath)){
		core::NodePtr_t node= roadmap()->addNode (nodeConfig);
		 hppDout (info,"goal add edge");
		roadmap()->addEdges (*itn,node,projpath);
	      }
	      else {flag++;}
	    }
	    if (flag==k){hppDout (info,"not any path has been found between the GoalNode and the interRoadmap");}
	  }
	flag=0;
	
      }
      }*/
    ////////////////////////////////////////////////////////////////////////////
    
    void RMRStar::startSolve ()
    {
      hppDout (info,"Start Solve");
      PathPlanner::startSolve ();

      computeTransitionMap ();
      step_=CONNECT_ROADMAPS;
      graph::GraphPtr_t graph =pb_.constraintGraph ();
      interRoadmap_ = core::Roadmap::create(pb_.distance(),pb_.robot());
      interRoadmap_->clear();
      core::NodePtr_t initNode=roadmap()->initNode ();
      graph::StatePtr_t stateInit=
       graph->getState(*(initNode->configuration()));
      interRoadmap_->addNode(initNode->configuration());
      
       //Get loop_edge constraints
      graph::EdgePtr_t loop_edge = transition_[stateInit];
      core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();
      RMRStar::ContactState contactStateInit (stateInit, *(initNode->configuration()), edgeConstraints);

      RMRStar::ProblemAndRoadmap_t pbRoadmapInit (pb_,interRoadmap_) ;
      
      //complete the map with the association ContactState/ProblemAndRoadmap
       association_.insert
	(std::pair<RMRStar::ContactState,RMRStar::ProblemAndRoadmap_t>
	(contactStateInit,pbRoadmapInit));
            hppDout (info,"first association Init"<<association_.size());

       for (core::NodeVector_t::const_iterator itn
	     (roadmap_->goalNodes ().begin ());
	   itn != roadmap_->goalNodes ().end (); ++itn) {
	 
	 graph::StatePtr_t stateGoal=graph->getState(*(*itn)->configuration());
	 
	 interRoadmap_ -> clear();
	 interRoadmap_ -> addNode((*itn)->configuration());
	 //Get loop_edge constraints
	 loop_edge = transition_[stateGoal];
	 edgeConstraints =loop_edge->pathConstraint();
	 RMRStar::ContactState contactStateGoal (stateGoal,( *(*itn)->configuration()), edgeConstraints);
	 
	 /* RMRStar::ProblemAndRoadmap_t pbRoadmapGoal (pb_,interRoadmap_) ;
	 
	 //complete the map with the association ContactState/ProblemAndRoadmap
	  association_.insert
	   (std::pair<RMRStar::ContactState,RMRStar::ProblemAndRoadmap_t>
	   (contactStateGoal,pbRoadmapGoal));*/
	 
	 contactState_=contactStateGoal;
	 edgeSteeringMethod_=problem().steeringMethod();

       }

    }
    ////////////////////////////////////////////////////////////////////////////
    void RMRStar::oneStep ()
    {
      int i=0;

      switch (step_)
	{
	case BUILD_ROADMAP:
	  buildRoadmap();
	  copyRoadmap ();
	  // tryToConnectInitAndGoal();
	    step_=CONNECT_ROADMAPS;
	  
	  break;

	case CONNECT_ROADMAPS:

	   connectRoadmap();
	   // tryToConnectInitAndGoal();

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
