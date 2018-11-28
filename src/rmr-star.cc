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
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
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

      const std::size_t nbComp = graph_->nbComponents();

      //create an indexed table with a node as key and the loop edge as value
      for (std::size_t i=0; i<nbComp; ++i)
	{
	  const graph::GraphComponentPtr_t graphComp (graph_->get(i));

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

      for (TransitionMap_t::const_iterator i=input_map.begin() ;
	   i!= input_map.end() ; ++i)  {

	retval.push_back(i->first);
      }
      return retval;
    }
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    // Shot a random config in a random state and create a ContactState
    //with the state, the configuration and the constraints applied

    RMRStar::ContactState RMRStar::sampleContact ()
    {

      using constraints::solver::HierarchicalIterative;

	bool valid =false;
	ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
	std::size_t nbStates (transition_.size ());
	// Shoot random number
	std::size_t i_rand=rand() % nbStates;

	// Grap random state of the graph
	const graph::StatePtr_t s_rand =
	  RMRStar::extract_keys(transition_)[i_rand];

	bool stateValid = false;

      while (!stateValid){
	core:: ValidationReportPtr_t validationReport;
	core::ConfigValidationsPtr_t configValidations
	  (problem ().configValidations ());
	int i=0;
	std::vector <constraints::vector_t> rhsOfTheFunctionAlreadyVisited;

	HierarchicalIterative::Status constraintApplied=
	  HierarchicalIterative::INFEASIBLE;

	//ConstraintSetPtr_t stateConfigstate = graph_->configConstraint(s_rand);
	ConstraintSetPtr_t stateConfig = graph_->configConstraint(transition_[s_rand]);
	constraints::NumericalConstraints_t numConstraints = stateConfig->configProjector () -> numericalConstraints();
	constraints::solver::BySubstitution solver
	  (stateConfig->configProjector ()->solver ());

	//Try to detect the function already visited and get their Right Hand side to set it
	//to the new state
	hppDout(info, "s_rand"<<s_rand->name());



	for (std::size_t i=0; i<numConstraints.size() ;i++)
	  {
	    for (RhsMap_t:: const_iterator it=RhsMap_.begin() ; it!= RhsMap_.end() ; ++it)
	      {
		if (it->first->functionPtr()->name()==numConstraints[i]->functionPtr()->name()){

		  rhsOfTheFunctionAlreadyVisited.push_back(it->second);

		}
	      }
	      hppDout(info,"rhsOfTheState"<<rhsOfTheFunctionAlreadyVisited.size());

	    if (!rhsOfTheFunctionAlreadyVisited.empty()){

	      std::size_t indice_rand=rand() % rhsOfTheFunctionAlreadyVisited.size();
	      constraints::vector_t Rhs_rand=rhsOfTheFunctionAlreadyVisited[indice_rand];
	      hppDout(info,"Rhs_rand"<<Rhs_rand);
	      bool success=solver.rightHandSide(numConstraints[i],Rhs_rand);

	      if (!success) { hppDout(info,"Fail to set Rhs for the function =" <<numConstraints[i]);
	      }
	      rhsOfTheFunctionAlreadyVisited.clear();
	    }

	  }

	while (valid==false && i<=100)
	  {
	    //Shoot random configuration
	    q_rand_= shooter->shoot();
	    constraintApplied =solver.solve(*q_rand_);
	    hppDout(info,"solverRhs"<<solver.rightHandSide());
	    hppDout (info,"RHSfrom =config"<<solver.rightHandSideFromConfig(*q_rand_));
	    // Get state constraint of s_rand (configConstraint)
	    /* constraintApplied = stateConfig->apply(*q_rand_);
	       if (constraintApplied!=true){*/
	    if (constraintApplied != HierarchicalIterative::SUCCESS) {
	      valid=false;
	      i++;
	      continue;
	    }
	    valid = configValidations->validate (*q_rand_, validationReport);
	    i++;
	    hppDout(info,"q_rand sampleContact"<<pinocchio::displayConfig(*q_rand_));
	  }
	hppDout (info,"i= "<<i);

	if (i==101){
	  hppDout (info,"fail to find a random configuration in state "
		   << s_rand->name ());

	  stateValid=false;
	  i=0;
	}

	else {stateValid=true;}

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
              hppDout (info,"build state "<<contactState_.state()->name());
	      hppDout	(info, "solver_build"<<contactState_.constraints()->configProjector ()->solver ());

	      //hppDout (info,"RHS="<<contactState_.rightHandSide());
      //copy the problem and pass the edge contraints
      core::Problem p (problem ());
      p.initConfig(q_rand_);
      p.steeringMethod (edgeSteeringMethod_);
      p.constraints(copyEdgeConstraints_);
      p.resetGoalConfigs();

      kPrm_=kPrmStar::createWithRoadmap(p,interRoadmap_);
      interRoadmap_->clear();
      hppDout (info,"kPRM start Solve ");

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
      hppDout (info,"copy Roadmap ");

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

      using constraints::solver::HierarchicalIterative;

      PathProjectorPtr_t pathProjector (pb_.pathProjector ());
      core::PathPtr_t projpath;
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());
      core::ConfigurationPtr_t q_inter;
      core:: ValidationReportPtr_t validationReport;
      int i= 0;
      int k= 5;    //the number of closest nodes that we try to connect to the
                   //intersection node
      int flag =0;
      int max_iter=100;
      core::PathPtr_t path;
      core::PathPtr_t validPath;


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
	  RhsMap_t rhsMap= (itstate->first).rhsMap ();

	  // check if the states are different and neighboors or are similar
	  graph::Edges_t connectedEdges =
	    graph_->getEdges(state,contactState_.state ());
	  bool vectorEmpty = connectedEdges.empty ();
	  bool valid=false;

	  //Apply constraint
	  HierarchicalIterative::Status constraintApplied=
	    HierarchicalIterative::INFEASIBLE;

	  core::ConfigValidationsPtr_t configValidations
	    (pb_.configValidations ());

	  // Get constraints of inter state
	  const core::ConstraintSetPtr_t& constraintTransitConfig =
	    contactState_.constraints ();

	  //Get the inter state solver
	  constraints::solver::BySubstitution solverTransit
	    ( constraintTransitConfig->configProjector ()-> solver ());

	  //Get the solver of constraint Edge
	  constraints::solver::BySubstitution solverEdge
	    ( constraintEdge->configProjector ()-> solver ());

	  //Get the numerical constraints of constraintEdge
	  const constraints::NumericalConstraints_t& constraints =
	    constraintEdge->configProjector ()->solver().
	    numericalConstraints();

	  //Get the numerical constraints of Transitconfig
	  const constraints::NumericalConstraints_t& transitConstraints =
	    constraintTransitConfig->configProjector ()->solver().
	    numericalConstraints();

	  hppDout(info, "association size" << association_.size());
	  hppDout(info, "rhs contactstate_" <<contactState_.rightHandSide());
	  hppDout (info, "rhs current" <<rhs);


	  bool rhsEqual=true;
	  pinocchio::value_type errorThreshold =
	    biggerThreshold(solverTransit,solverEdge);

	  // for (std::size_t l=0; l<transitConstraints.size() ;l++)
	  for (RhsMap_t:: const_iterator it=contactState_.rhsMap().begin() ; it!=contactState_.rhsMap().end() ; ++it)
	    {

		 for (RhsMap_t:: const_iterator i=rhsMap.begin() ; i!=rhsMap.end() ; ++i)
		{

		  hppDout(info, "les fonctions : "<<it->first->functionPtr ()->name()<< "et " << i->first->functionPtr ()->name()<<" sont comparées. Leur RHS sont: "<<it->second << "et  " << i->second);

		  if (it->first==i->first &&
		      it->second.isApprox( i->second,errorThreshold) ){
		     hppDout(info, "tout est egal OK");
		      break;}
		  if (it->first==i->first &&
		     it->second!=i->second )

		    {hppDout(info, "les fonctions sont egale mais pas les rhs");

		      rhsEqual=false;
		      break;
		      }

		}
		  if (rhsEqual==false){break;}
	    }



	  if ((contactState_.state () !=state && !vectorEmpty && rhsEqual)||
	      (contactState_.state () ==state &&
	       contactState_.rightHandSide().isApprox(rhs,errorThreshold) ))
	    {

	      constraints::solver::BySubstitution solver
		(constraintTransitConfig->configProjector ()->solver ());

	      //get the solver of the stateConfig
	      assert (constraintTransitConfig);

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

	      hppDout(info,"solver q_inter"<<solver);
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

			// Retrieve the path validation algorithm associated to the problem

			if (pathProjector->apply(path,projpath))
			  {

			    PathValidationPtr_t pathValidation ( transition_[contactState_.state()]->pathValidation());
			    PathValidationReportPtr_t report;
			    bool valid = pathValidation->validate (projpath, false, validPath, report);


			    hppDout(info,"valid= "<<valid);

			    if (valid){
			      core::NodePtr_t node=
				roadmap_->addNode (nodeConfig);
			      roadmap_->addEdges (nodeConnect,node,projpath);
			      hppDout (info,"edge created");
			    }
			  }
			else {flag++;}
		      }
		      if (flag==k)
			{hppDout (info,"not any path has been found between the nodeInter and the interRoadmap with state="<<contactState_.state()->name()<<"RHS="<<contactState_.rightHandSide());}
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
		  // sm->constraints ()->configProjector()->rightHandSideFromConfig(*q_inter);

		  for (core::Nodes_t::const_iterator itnode =nearestNodes.
			 begin(); itnode !=nearestNodes.end(); itnode++ )
		    {
		      core::ConfigurationPtr_t nodeConfig =
			(*itnode)->configuration();


		      //use the steering method of the selectRoadmap to find a path to the nodeInter

		      constraints::vector_t error;

		      constraints::vector_t errorbis;


		       assert (pb.constraints ()->isSatisfied (*q_inter));
		      hppDout (info, " node "<<  pinocchio::displayConfig(*nodeConfig));

		      assert (pb.constraints ()->isSatisfied (*nodeConfig));



		      path =(*sm)(*q_inter,*nodeConfig);


		      if (path==NULL){
			flag++;
			hppDout (info,"path2=NULL");

		      }
		      else {


			if (pathProjector->apply(path,projpath)){
			  PathValidationPtr_t pathValidation (pb.pathValidation ());
			  PathValidationReportPtr_t report;
			  bool valid = pathValidation->validate (projpath, false, validPath, report);

			  hppDout(info,"valid= "<<valid);

			  if (valid){

			    core::NodePtr_t node=roadmap_->addNode(nodeConfig);
			    roadmap_->addEdges (nodeConnect,node,projpath);
			    hppDout (info,"edge created");
			  }
			}
			else { flag++;}
		      }
		      if (flag==k)
			{hppDout (info,"not any path has been found between the nodeInter and the roadmap with state="<<state->name()<<"RHS="<<rhs);}
		    }
		  hppDout (info,"f2="<<flag);

		  flag=0;

		}
	    }
	}
      associationmap();
    }
    ////////////////////////////////////////////////////////////////////////


    void RMRStar::storeRhs ()
    {
      constraints::solver::BySubstitution solver
		(contactState_.constraints()->configProjector ()->solver ());
      core::NumericalConstraints_t constraints =solver. numericalConstraints();

      for (std::size_t i=0 ; i<constraints.size() ; i++)
	{
	  constraints::ImplicitPtr_t function = constraints[i];
	  constraints::vectorOut_t rhs= function->nonConstRightHandSide();
	  bool success = false;
	  success=solver.getRightHandSide(function,rhs);

	   assert (success);

	  bool null=rhsNull(rhs);
	  bool alreadyInMap=false;

	  if (!null){

	    // hppDout (info, "enter in the if loop RHS non null");
	    for (RhsMap_t:: const_iterator it=RhsMap_.begin() ; it!= RhsMap_.end() ; ++it)
	      {
		if (function==(it->first) && rhs.isApprox(it->second,solver.errorThreshold()))
		  {
		    alreadyInMap=true;
		  }

	      }

	    if(alreadyInMap==false){
	       hppDout(info, "increment RhsMap_");
	      RhsMap_.insert
		(std::pair<constraints::ImplicitPtr_t,constraints::vectorIn_t>
		 (function,rhs));
	    }
	  }


	}
      /* for (RhsMap_t:: const_iterator num=RhsMap_.begin() ; num!= RhsMap_.end() ; ++num)
	{
	  hppDout (info, "function = "<<num->first->functionPtr()->name() <<" et RHs = " <<num->second);
	}
	hppDout(info,"RhsMap_"<<RhsMap_.size());*/
    }
    //////////////////////////////////////////////////////////////////////////
    bool RMRStar::rhsNull (vector_t rhs)
    {
      bool null = true;

      for (size_type i=0; i<rhs.size(); ++i )
	{
	  if (rhs[i]!=0)
	    {
	      null=false;
	    }
	}
      return null;
    }

      //////////////////////////////////////////////////////////////////////////
    void RMRStar::associationmap ()
    {

       //copy the problem and pass the edge contraints
      core::Problem p (problem ());
      p.initConfig(q_rand_);
      p.steeringMethod (edgeSteeringMethod_->copy ());
      p.constraints(core::ConstraintSet::createCopy(copyEdgeConstraints_));
      p.pathValidation(transition_[contactState_.state()]->pathValidation());
      core::RoadmapPtr_t r = core::Roadmap::create(p.distance(),p.robot());
      r->clear();


      //Create a copy of the interRoadmap that we stock in the inter Roadmap
      for (core::Nodes_t :: const_iterator itnode =interRoadmap_->nodes().begin(); itnode !=interRoadmap_->nodes().end(); itnode++ )
	{
	  r->addNode((*itnode)->configuration());
	  assert (edgeSteeringMethod_->constraints ()->isSatisfied( *(*itnode)->configuration()));

	}

      RMRStar::ProblemAndRoadmap_t pbRoadmap (p,r) ;

      //complete the map with the association ContactState/ProblemAndRoadmap
       association_.insert
	(std::pair<RMRStar::ContactState,RMRStar::ProblemAndRoadmap_t>
	(contactState_,pbRoadmap));

     for (AssociationMap_t:: const_iterator it=association_.begin() ; it!= association_.end() ; ++it)
	      {
		hppDout(info,"association_ size = " <<association_.size());



		hppDout(info, "Right Hand Side"<<it->first.rightHandSide());

		const RhsMap_t rhs = it->first.rhsMap() ;
		for (RhsMap_t::const_iterator i=rhs.begin();i!= rhs.end() ; ++i)
		  {
		    hppDout(info, "function"<<i->first->functionPtr()->name()<< " rhs=  "<<i->second);

		  }
	      }
    }
    ///////////////////////////////////////////////////////////////////////////
    pinocchio::value_type RMRStar::biggerThreshold
    ( constraints::solver::BySubstitution solver1,constraints::solver::BySubstitution solver2)
    {
      pinocchio::value_type threshold1 = solver1.errorThreshold();
      pinocchio::value_type threshold2 = solver2.errorThreshold();
      if (threshold1 < threshold2)
	{
	  return threshold2;
	  }

      if (threshold2 < threshold1)
	{
	  return threshold1;
	}
      else
	{
	  return threshold1;
	}
    }
    ////////////////////////////////////////////////////////////////////////////


    void RMRStar::startSolve ()
    {
      hppDout (info,"Start Solve");
      PathPlanner::startSolve ();
      graph_ =pb_.constraintGraph ();
      computeTransitionMap ();

      //Set parameter and create interRoadmap
      interRoadmap_ = core::Roadmap::create(pb_.distance(),pb_.robot());
      interRoadmap_->clear();
      q_rand_=roadmap()->initNode ()->configuration();

      //Set ContactState
      graph::StatePtr_t stateInit=
	graph_->getState(*(q_rand_));
      graph::EdgePtr_t loop_edge = transition_[stateInit];
      core::ConstraintSetPtr_t edgeConstraints =loop_edge->pathConstraint();
      RMRStar::ContactState contactStateInit
	(stateInit, *q_rand_, edgeConstraints);
      edgeSteeringMethod_ = loop_edge -> steeringMethod();
      contactState_=contactStateInit;

      buildRoadmap();
      copyRoadmap ();
      associationmap();
      storeRhs();


      for (core::NodeVector_t::const_iterator itn
	     (roadmap_->goalNodes ().begin ());
	   itn != roadmap_->goalNodes ().end (); ++itn) {


	interRoadmap_ -> clear();

	//Set ContactState_
	q_rand_=(*itn)->configuration();

	graph::StatePtr_t stateGoal=graph_->getState(*q_rand_);
	loop_edge = transition_[stateGoal];
	edgeConstraints =loop_edge->pathConstraint();

	RMRStar::ContactState contactStateGoal
	  (stateGoal,*q_rand_, edgeConstraints);

	contactState_=contactStateGoal;

	buildRoadmap();
	copyRoadmap ();
	connectRoadmap();
	storeRhs();


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
	  storeRhs();

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
