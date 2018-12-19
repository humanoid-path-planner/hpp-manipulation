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

	ConstraintSetPtr_t stateConfig =
	  graph_->configConstraint(transition_[s_rand]);

	constraints::NumericalConstraints_t numConstraints =
	  stateConfig->configProjector () -> numericalConstraints();

	constraints::solver::BySubstitution solver
	  (stateConfig->configProjector ()->solver ());

	//Try to detect the function already visited and get their
	//Right Hand side to set it to the new state
	hppDout(info,"solver"<<solver);
	hppDout (info,"counter = "<<counter_);

	//	if (counter_!=setRhsFreq_)
	// {
	for (std::size_t i=0; i<numConstraints.size() ;i++)
	  {
	    for (RhsMap_t:: const_iterator it=RhsMap_.begin() ;
		 it!= RhsMap_.end() ; ++it)
	      {
		if (it->first->functionPtr()->name()==
		    numConstraints[i]->functionPtr()->name()){

		  rhsOfTheFunctionAlreadyVisited.push_back(it->second);
		}
	      }

	    if (!rhsOfTheFunctionAlreadyVisited.empty()){

	      std::size_t indice_rand=rand() % rhsOfTheFunctionAlreadyVisited.size();
	      constraints::vector_t Rhs_rand=rhsOfTheFunctionAlreadyVisited[indice_rand];
	      bool success=solver.rightHandSide(numConstraints[i],Rhs_rand);
	      assert(success);
	      rhsOfTheFunctionAlreadyVisited.clear();
	    }
	  }
	// }
	hppDout(info, "i = "<<i);
	hppDout(info, "valid ="<<valid);
	while (valid==false && i<=100)
	  {
	    //Shoot random configuration
	    q_rand_= shooter->shoot();
	    constraintApplied =solver.solve(*q_rand_);

	    if (constraintApplied != HierarchicalIterative::SUCCESS) {
	      valid=false;
	      i++;
	      continue;
	    }
	    valid = configValidations->validate (*q_rand_, validationReport);
	    i++;
	    hppDout(info,"q_rand sampleContact"<<pinocchio::displayConfig(*q_rand_));
	  }

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

    void RMRStar::copyRoadmap ()
    {
      const core::Edges_t& edges = interRoadmap_->edges ();
      core::NodePtr_t node1;
      core::NodePtr_t node2;

      for  (core::Edges_t::const_iterator itedge = edges.begin();
	    itedge != edges.end(); itedge++){
	node1=roadmap_->addNode(core::ConfigurationPtr_t
                                (new Configuration_t(*(*itedge)->from()->configuration())));
        node2=roadmap_->addNode(core::ConfigurationPtr_t
                                (new Configuration_t(*(*itedge)->to()->configuration())));
	

	roadmap()->addEdge (node1, node2,(*itedge)->path());

      }
    }

    ////////////////////////////////////////////////////////////////////////////

    void RMRStar::connectRoadmap () {

      using constraints::solver::HierarchicalIterative;

      PathProjectorPtr_t pathProjector (pb_.pathProjector ());
      core::PathPtr_t projpath;
      ConfigurationShooterPtr_t shooter (pb_.configurationShooter ());

      core:: ValidationReportPtr_t validationReport;
      size_type k=problem().getParameter("RMR*/numberOfConnectNodes").intValue(); 

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
	  RhsMap_t rhsMap= (itstate->first).rhsMap ();
	  core::RoadmapPtr_t selectRoadmap =itstate -> second.second;
	  core::Nodes_t nodes= selectRoadmap->nodes ();

	  // check if the states are different and neighboors or are similar
	  graph::Edges_t connectedEdges =
	    graph_->getEdges(contactState_.state (),state);

	  assert (connectedEdges.size()<=1);

	  bool vectorEmpty = connectedEdges.empty ();
	  bool valid=false;

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

	  bool rhsEqual=true;
	  pinocchio::value_type errorThreshold =
	    biggestThreshold(solverTransit,solverEdge);

	  for (RhsMap_t:: const_iterator it=contactState_.rhsMap().begin() ; it!=contactState_.rhsMap().end() ; ++it)
	    {

	      for (RhsMap_t:: const_iterator i=rhsMap.begin() ; i!=rhsMap.end() ; ++i)
		{
		  if (it->first==i->first &&
		      it->second!=i->second )
		    {
		      //les fonctions sont egale mais pas les rhs
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

	      graph::WaypointEdgePtr_t waypointEdge
		(HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge,connectedEdges[0]));

	      if (waypointEdge){
		assert (contactState_.state ()!=state);
		hppDout(info, "Problème avec Waypoints");

		connectStatesByWaypoints
		  (waypointEdge,constraints, transitConstraints,
		   rhsMap,max_iter,k, configValidations,
		   validationReport,valid,constraintApplied, projpath,
		   pathProjector,shooter,selectRoadmap, state,config);
	      }
	      else
		{
		hppDout(info, "Problème sans Waypoints");

		connectDirectStates
		  (constraintTransitConfig,constraints, transitConstraints,
		   configValidations, validationReport,max_iter,k,shooter,
		   constraintApplied,path,projpath, pathProjector,config,
		   valid,state,selectRoadmap);
	      }
	    }
	}
      associationmap();
    }


    /////////////////////////////////////////////////////////////////////////////////////////////

    void RMRStar::connectStatesByWaypoints
    ( graph::WaypointEdgePtr_t waypointEdge,
      const constraints::NumericalConstraints_t& constraints,
      const constraints::NumericalConstraints_t& transitConstraints,
      RhsMap_t rhsMap, int max_iter,size_type k,
      core::ConfigValidationsPtr_t configValidations,
      core:: ValidationReportPtr_t validationReport, bool valid,
      constraints::solver::HierarchicalIterative::Status constraintApplied,
      core::PathPtr_t projpath, PathProjectorPtr_t pathProjector,
      ConfigurationShooterPtr_t shooter,core::RoadmapPtr_t roadmap,
      graph::StatePtr_t state, Configuration_t config)
    {
      using constraints::solver::HierarchicalIterative;

      //get waypoints constraints
      std::size_t nbOfWaypoints = waypointEdge -> nbWaypoints();
      core::ConfigurationPtr_t q_waypointInter;
      core::PathPtr_t validPath;
      core::PathPtr_t path;
      int i=0;
      hppDout(info,"nbOfWaypoints = "<<nbOfWaypoints);
      bool succeed=false;

      /////////////////////////////////////////////
      //Find intersec waypoint
      ///////////////////////////////////////////
      for (std::size_t Waypoint=0; Waypoint<nbOfWaypoints ; Waypoint++)
	{
	  const char *intersec ="intersec";
	  const char *waypointName=
	    (waypointEdge->waypoint(Waypoint)->from()->name()).c_str();
	  const char * waypointIntersec= std::strstr (waypointName,intersec);

	  if (waypointIntersec)
	    {

	      while (succeed ==false)
		{
		  const core::ConstraintSetPtr_t& constraintTransitConfig =
		    contactState_.constraints ();

		  valid=false;
		  q_waypointInter = createInterStateNode
		    (constraintTransitConfig,constraints, transitConstraints,
		     configValidations, validationReport,max_iter,shooter,
		     constraintApplied,config,
		     valid,state);

		  //Copy of  waypointInter
		  core::Configuration_t* ptr =
		    new Configuration_t(*q_waypointInter);

		  core::ConfigurationPtr_t q_waypoint (ptr);

		  core::Configuration_t* ptr2 =
		    new Configuration_t(*q_waypointInter);

		  core::ConfigurationPtr_t q_nextWaypoint (ptr2);

		  /////////////////////////////////////////////////////////////
		  ///Connect the waypoints from intersec to the first one
		  ////////////////////////////////////////////////////////////

		  for (int w=Waypoint-1; w>=1; --w)
		    {
		      hppDout(info,"w= "<<w);
		      graph::EdgePtr_t forwardEdge = waypointEdge->waypoint(w);
		      graph::Edges_t edges =
			graph_->getEdges(state, contactState_.state());
		      assert(edges.size()==1);

		      graph::WaypointEdgePtr_t backwardWaypointEdge =
			HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, edges[0]);
		      assert(backwardWaypointEdge);

		      graph::EdgePtr_t Edge =
			backwardWaypointEdge->waypoint(nbOfWaypoints-w);
		      core::ConstraintSetPtr_t constraintEdge=
			Edge->configConstraint();
		      constraints::solver::BySubstitution solver
			(constraintEdge->configProjector ()->solver ());

		      for (std::size_t j=0; j<solver.numericalConstraints().size(); j++)
			{
			  for (RhsMap_t:: const_iterator it=contactState_.rhsMap().begin() ; it!=contactState_.rhsMap().end() ; ++it)
			    {
			      if (solver.numericalConstraints()[j]==it->first)
				{
				  solver.rightHandSide
				    (solver.numericalConstraints()[j],it->second);
				}
			    }
			  for (RhsMap_t::const_iterator i=rhsMap.begin(); i!=rhsMap.end(); ++i)
			    {
			      if (solver.numericalConstraints()[j]==i->first)
				{
				  solver.rightHandSide
				    (solver.numericalConstraints()[j],i->second);
				}
			    }
			}
		      i=0;
		      valid=false;

		      while ((valid==false) && i<max_iter)
			{
			  constraintApplied = solver.solve(*q_nextWaypoint);

			  if (constraintApplied !=
			      HierarchicalIterative::SUCCESS)
			    {
			    valid=false;
			    ++i;
			    continue;
			  }
			  valid = configValidations->validate
			    (*q_nextWaypoint,validationReport);
			  i++;
			}

		      if (i==max_iter) {
			hppDout (info,"i reached max_iter, not any connect node have been found");
		      hppDout(info, "i = "<<i);
		      hppDout(info, "success = "<<succeed);
		      }
		      else
			{
			  succeed=true;

			  connectConfigToNode(forwardEdge,path,pathProjector,projpath,q_nextWaypoint, q_waypoint);

			  *q_waypoint=*q_nextWaypoint;

			}
		    }

		  /////////////////////////////////////////////////////////////
		  ///Connect first Waypoint to the interRoadmap
		  /////////////////////////////////////////////////////////////

		  if (i!=max_iter){
		    core::Nodes_t nearNodes =
		      interRoadmap_->nearestNodes(q_waypoint,k);

		    for (core::Nodes_t :: const_iterator itnode = nearNodes.
			   begin(); itnode !=nearNodes.end(); itnode++ )
		      {
			core::ConfigurationPtr_t nodeConfig =
			  (*itnode)->configuration();

			connectConfigToNode ( waypointEdge->waypoint(0),path,pathProjector,projpath,nodeConfig,q_waypoint);
		      }
		    }
		  /////////////////////////////////////////////////////////////
		  //Connect the waypoints from intersec to the last one
		  /////////////////////////////////////////////////////////////
		  *q_waypoint=*q_waypointInter;
		  *q_nextWaypoint=*q_waypointInter;

		  for (std::size_t w=Waypoint; w<nbOfWaypoints; w++)
		    {
		      hppDout(info,"w= "<<w);
		      graph::EdgePtr_t Edge = waypointEdge->waypoint(w);
		      core::ConstraintSetPtr_t constraintEdge=Edge->configConstraint();
		      constraints::solver::BySubstitution solver
			(constraintEdge->configProjector ()->solver ());

		      for (std::size_t j=0; j<solver.numericalConstraints().size(); j++)
			{
			  for (RhsMap_t:: const_iterator it=contactState_.rhsMap().begin() ; it!=contactState_.rhsMap().end() ; ++it)
			    {
			      if (solver.numericalConstraints()[j]==it->first)
				{
				  solver.rightHandSide
				    (solver.numericalConstraints()[j],it->second);
				}
			    }
			  for (RhsMap_t::const_iterator i=rhsMap.begin(); i!=rhsMap.end(); ++i)
			    {
			      if (solver.numericalConstraints()[j]==i->first)
				{
				  solver.rightHandSide
				    (solver.numericalConstraints()[j],i->second);
				}
			    }
			}

		      i=0;
		      valid=false;

		      while ((valid==false) && i<max_iter)
			{
			  constraintApplied = solver.solve(*q_nextWaypoint);

			  if (constraintApplied != HierarchicalIterative::SUCCESS) {
			    valid=false;
			    ++i;
			    continue;
			  }
			  valid = configValidations->validate
			    (*q_nextWaypoint,validationReport);
			  i++;
			}

		      if (i==max_iter||!succeed) {
			hppDout (info,"i reached max_iter, not any connect node have been found");
			hppDout(info, "i = "<<i);
			hppDout(info, "succeed = "<<succeed);

		      }
		      else
			{
			  connectConfigToNode(Edge,path,pathProjector,projpath,q_waypoint, q_nextWaypoint);

			  *q_waypoint=*q_nextWaypoint;
			}
		    }

		  /////////////////////////////////////////////////////////////
		  ///Connect the last waypoint to the Roadmap in the table
		  /////////////////////////////////////////////////////////////

		  if (i!=max_iter && succeed){
		    core::Nodes_t nearestNodes =
		      roadmap->nearestNodes(q_waypoint,k);

		    for (core::Nodes_t :: const_iterator itnode = nearestNodes.
			   begin(); itnode !=nearestNodes.end(); itnode++ )
		      {
			core::ConfigurationPtr_t nodeConfig =
			  (*itnode)->configuration();

			connectConfigToNode ( waypointEdge->waypoint(nbOfWaypoints),path,pathProjector,projpath,q_waypoint,nodeConfig);
		      }
		  }
		}
	      
	      break;
	    }
	}
    }
    
    ////////////////////////////////////////////////////////////////////////

    void RMRStar::connectDirectStates
    ( const core::ConstraintSetPtr_t& constraintTransitConfig,
      const constraints::NumericalConstraints_t& constraints,
      const constraints::NumericalConstraints_t& transitConstraints,
      core::ConfigValidationsPtr_t configValidations,
      core:: ValidationReportPtr_t validationReport,
      int max_iter , size_type k,
      ConfigurationShooterPtr_t shooter,
      constraints::solver::HierarchicalIterative::Status constraintApplied,
      core::PathPtr_t path,
      core::PathPtr_t projpath, core::PathProjectorPtr_t pathProjector,
      core:: Configuration_t config, bool valid, graph::StatePtr_t state,
      core::RoadmapPtr_t roadmap) {

      core::ConfigurationPtr_t q_inter = createInterStateNode
	(constraintTransitConfig,constraints, transitConstraints,
	 configValidations, validationReport,max_iter,shooter,
	 constraintApplied,config,
	 valid, state);
      graph::EdgePtr_t edgeTransit =transition_[contactState_.state()];
      core::Nodes_t nearNodes = interRoadmap_->nearestNodes(q_inter,k);
      core::ConstraintSetPtr_t constraintTRansit =
	edgeTransit->configConstraint();

      for (core::Nodes_t :: const_iterator itnode = nearNodes.
	     begin(); itnode !=nearNodes.end(); itnode++ )
	{
	  core::ConfigurationPtr_t nodeConfig =
	    (*itnode)->configuration();

	  connectConfigToNode (edgeTransit,path,pathProjector,projpath,q_inter,nodeConfig);

	}
      graph::EdgePtr_t edge =transition_[state];
      core::Nodes_t nearestNodes = roadmap->nearestNodes(q_inter,k);
      core::ConstraintSetPtr_t constraint =  edge->configConstraint();

      for (core::Nodes_t :: const_iterator it = nearestNodes.
	     begin(); it !=nearestNodes.end(); it++ )
	{
	  core::ConfigurationPtr_t nodeConfig =
	    (*it)->configuration();

	  connectConfigToNode (edge,path,pathProjector,projpath,q_inter,nodeConfig);
	}
    }
    ///////////////////////////////////////////////////////////////////////

    core::ConfigurationPtr_t RMRStar::createInterStateNode
    ( const core::ConstraintSetPtr_t& constraintTransitConfig,
      const constraints::NumericalConstraints_t& constraints,
      const constraints::NumericalConstraints_t& transitConstraints,
      core::ConfigValidationsPtr_t configValidations,
      core:: ValidationReportPtr_t validationReport,
      int max_iter ,
      ConfigurationShooterPtr_t shooter,
      constraints::solver::HierarchicalIterative::Status constraintApplied,
      core:: Configuration_t config, bool valid, graph::StatePtr_t state)
    {
      using constraints::solver::HierarchicalIterative;

      core::ConfigurationPtr_t q_inter;
      int i= 0;

      constraints::solver::BySubstitution solver
	(constraintTransitConfig->configProjector ()->solver ());

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

      while ((valid==false) && i<max_iter)
	{
	  //Shoot random configuration
	  q_inter= shooter->shoot();
	  hppDout (info,"q_inter="<< pinocchio::displayConfig (*q_inter));

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
      hppDout (info,"i="<<i);
      hppDout (info,"connect states "<<contactState_.state()->name()<<
	       "  state "<<state->name());
      hppDout (info,"q_inter="<< pinocchio::displayConfig (*q_inter));
      hppDout (info,"constraintApplied="<<constraintApplied);

      //test constraints have been applied to q_inter
      if (i==max_iter) {
	hppDout (info,"i reached max_iter, not any connect node have been found");
	assert (i!=max_iter);
      }
      return q_inter;
    }
    ////////////////////////////////////////////////////////////////////////

    void RMRStar::connectConfigToNode
    ( graph::EdgePtr_t edge,core::PathPtr_t  path,
      core::PathProjectorPtr_t pathProjector, core::PathPtr_t projpath,
      core::ConfigurationPtr_t q1,
      core::ConfigurationPtr_t configuration)
    {

      //connect the interRoadmap to the nodeInter
      core::PathPtr_t validPath;
      core::NodePtr_t nodeConnect=
	roadmap_->addNode (core::ConfigurationPtr_t (new Configuration_t(*q1)));

      hppDout (info, " node1 " << pinocchio::displayConfig(*configuration));
      hppDout (info, " node2 " <<  pinocchio::displayConfig(*q1));

      core::SteeringMethodPtr_t sm= edge->steeringMethod();

      assert(edge->from()->contains(*q1));
      assert(edge->state()->contains(*q1));
      assert(edge->to()->contains(*configuration));
      assert(edge->state()->contains(*configuration));

      path =(*sm)(*q1,*configuration);

      if (path==NULL){
	hppDout (info,"path=NULL");
      }
      else {

	if (pathProjector->apply(path,projpath))
	  {

	    PathValidationPtr_t pathValidation ( edge->pathValidation());
	    PathValidationReportPtr_t report;
	    bool valid =
	      pathValidation->validate (projpath, false, validPath, report);

	    if (valid){
	      core::NodePtr_t node=
		roadmap_->addNode(core::ConfigurationPtr_t
				  (new Configuration_t(*configuration)));

	      roadmap_->addEdges (nodeConnect,node,projpath);
	      assert (projpath->initial () == *(nodeConnect->configuration()));
	      assert (projpath->end () == *(node->configuration()));
	      hppDout (info,"edge created ");
	    }
	    else {hppDout(info, "path not valid");}
	  }
	else {
	  hppDout(info, "path not OK");}
      }
    }
    ///////////////////////////////////////////////////////////////////////////

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
	    for (RhsMap_t:: const_iterator it=RhsMap_.begin() ; it!= RhsMap_.end() ; ++it)
	      {
		if (function==(it->first) &&
		    rhs.isApprox(it->second,solver.errorThreshold()))
		  {
		    alreadyInMap=true;
		  }
	      }

	    if(alreadyInMap==false){
	      RhsMap_.insert
		(std::pair<constraints::ImplicitPtr_t,constraints::vectorIn_t>
		 (function,rhs));
	    }
	  }
	}
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

      //Create a copy of the interRoadmap that we stock in the associationmap
      for (core::Nodes_t :: const_iterator itnode =interRoadmap_->nodes().begin(); itnode !=interRoadmap_->nodes().end(); itnode++ )
	{
	  r->addNode(core::ConfigurationPtr_t
                     (new Configuration_t(*(*itnode)->configuration())));	}

      RMRStar::ProblemAndRoadmap_t pbRoadmap (p,r) ;

      //complete the map with the association ContactState/ProblemAndRoadmap
      association_.insert
	(std::pair<RMRStar::ContactState,RMRStar::ProblemAndRoadmap_t>
	 (contactState_,pbRoadmap));
    }
    ///////////////////////////////////////////////////////////////////////////

    pinocchio::value_type RMRStar::biggestThreshold
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
    ///////////////////////////////////////////////////////////////////////////

    void RMRStar::startSolve ()
    {
      PathPlanner::startSolve ();
      graph_ =pb_.constraintGraph ();
      computeTransitionMap ();
      setRhsFreq_=problem().getParameter("RMR*/SetRhsFreq").intValue();
      counter_=0;
      //Set parameter and create interRoadmap
      interRoadmap_ = core::Roadmap::create(pb_.distance(),pb_.robot());
      interRoadmap_->clear();
      q_rand_=roadmap()->initNode ()->configuration();

      //Set init ContactState
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

	//Set final ContactState
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
      switch (step_)
	{

	case BUILD_ROADMAP:

	  contactState_ = sampleContact();
	  buildRoadmap();
	  copyRoadmap ();
	  storeRhs();

	  step_=CONNECT_ROADMAPS;
	  if (counter_==setRhsFreq_)
	    {
	      counter_=0;
	    }
	  counter_++;
	  break;

	case CONNECT_ROADMAPS:

	  connectRoadmap();
	  step_= BUILD_ROADMAP;
	 
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
    // ----------- Declare parameters ------------------------------------- //
    using core::Parameter;
    using core::ParameterDescription;
    HPP_START_PARAMETER_DECLARATION(RMRStar)
    core::Problem::declareParameter(ParameterDescription (Parameter::INT,
							  "RMR*/numberOfConnectNodes",
							  "The desired number of the nodes we try to connect between the intersection node and the roadmaps.",
							  Parameter((size_type)3)));
    
    core::Problem::declareParameter(ParameterDescription (Parameter::INT,
							  "RMR*/SetRhsFreq",
							  "The desired number of the frequency of set configuration map build.If it's 0 it's never mannually set, if it's 100 it's 99% of time mannually set",
							  Parameter((size_type)100)));
    HPP_END_PARAMETER_DECLARATION(RMRStar) 
  }//end namespace manipulation
}//end namespace hpp
