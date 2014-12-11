namespace hpp {
  namespace manipulation {
    namespace graph {
/**
 \mainpage

 \section sec_intro_hpp_manipulation Introduction

 This package implements a solver for manipulation planning problems. A manipulation planning problem
 is defined by:
 \li Robot Build a \LModel{Device} from several \LModel{Device} and Object,
 \li Object A \LModel{Device} with Handle,
 \li Graph A graph of constraints defining the rules of a manipulation problem,
 \li ManipulationPlanner Implements a RRT-based algorithm to solve manipulation planning problems.

 \section sec_graph_hpp_manipulation Constraint graph
 The graph of constraint, also referred to as constraint graph, represents the rules of a manipulation problem.
 The component of the graph are:
 \li Node represents a state of the Robot with constraints,
 \li Edge represents a transition between two Node with parametric constraints.

 Node contains a set of \LHPP{core,Constraint} that a configuration of the Robot should satisfy
 to be in the represented state. To ensure that a configuration is in only one state, the Node are
 ordered in a NodeSelector. The method NodeSelector::getNode(ConfigurationIn_t) const returns a
 pointer to the first Node for which Node::contains(ConfigurationIn_t) const returns true.
 For optimization only, another set of \LHPP{core,Constraint} is used for \LHPP{core,StraightPath} lying in this Node.

 Edge has methods Edge::isInNodeFrom, to tell if a corresponding path lyes in Edge::from() or Edge::to(),
 and Edge::node(), to retrive this Node.
 Edge also contains two sets of \LHPP{core,Constraint}:
 \li Edge::configConstraint() returns a \LHPP{core,ConstraintSet} used to generate a configuration lying in Edge::to()
     and respecting the offset (previsously set using hpp::core::Constraint::offsetFromConfig),
 \li Edge::pathConstraint() returns a \LHPP{core,ConstraintSet} to be inserted in \LHPP{core,Path} represented
     by this Edge.
 
 \note
   For implementation details, see graph::Graph.
   For more information about parametric and non-parametric constraints, see \LHPP{core,DifferentiableFunction}
   and \LHPP{core,ConfigProjector}

 \section sec_solver_hpp_manipulation Manipulation planner

 ManipulationPlanner class implements an algorithm based on RRT. See
 <a href="ObjectManipulation_MasterThesis_JosephMirabel.pdf">this master thesis</a> for details about the algorithm.
 **/
    }
  }
}

