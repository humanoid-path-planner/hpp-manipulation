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

#include "hpp/manipulation/graph/edge.hh"

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/obstacle-user.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/pointer.hh>
#include <sstream>

#include "hpp/manipulation/constraint-set.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/steering-method/graph.hh"

namespace hpp {
namespace manipulation {
namespace graph {
Edge::Edge(const std::string& name)
    : GraphComponent(name),
      isShort_(false),
      pathConstraints_(),
      targetConstraints_(),
      steeringMethod_(),
      securityMargins_(),
      pathValidation_() {}

Edge::~Edge() {}

StatePtr_t Edge::stateTo() const { return to_.lock(); }

StatePtr_t Edge::stateFrom() const { return from_.lock(); }

void Edge::relativeMotion(const RelativeMotion::matrix_type& m) {
  if (!isInit_)
    throw std::logic_error(
        "The graph must be initialized before changing the relative motion "
        "matrix.");
  shared_ptr<core::ObstacleUserInterface> oui =
      HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pathValidation_);
  if (oui) oui->filterCollisionPairs(m);
  relMotion_ = m;
}

bool Edge::direction(const core::PathPtr_t& path) const {
  Configuration_t q0 = path->initial(), q1 = path->end();
  const bool src_contains_q0 = stateFrom()->contains(q0);
  const bool dst_contains_q0 = stateTo()->contains(q0);
  const bool src_contains_q1 = stateFrom()->contains(q1);
  const bool dst_contains_q1 = stateTo()->contains(q1);
  // Karnaugh table:
  // 1 = forward, 0 = reverse, ? = I don't know, * = 0 or 1
  // s0s1 \ d0d1 | 00 | 01 | 11 | 10
  // 00          |  ? |  ? |  ? |  ?
  // 01          |  ? |  ? |  0 |  0
  // 11          |  ? |  1 |  * |  0
  // 10          |  ? |  1 |  1 |  1
  //
  /// true if reverse
  if ((!src_contains_q0 && !src_contains_q1) ||
      (!dst_contains_q0 && !dst_contains_q1) ||
      (!src_contains_q0 && !dst_contains_q0))
    HPP_THROW(std::runtime_error,
              "Edge " << name() << " does not seem to have generated path from"
                      << pinocchio::displayConfig(q0) << " to "
                      << pinocchio::displayConfig(q1));
  return !(src_contains_q0 && (!src_contains_q1 || dst_contains_q1));
}

void Edge::securityMarginForPair(const size_type& row, const size_type& col,
                                 const value_type& margin) {
  if ((row < 0) || (row >= securityMargins_.rows())) {
    std::ostringstream os;
    os << "Row index should be between 0 and " << securityMargins_.rows() + 1
       << ", got " << row << ".";
    throw std::runtime_error(os.str().c_str());
  }
  if ((col < 0) || (col >= securityMargins_.cols())) {
    std::ostringstream os;
    os << "Column index should be between 0 and " << securityMargins_.cols() + 1
       << ", got " << col << ".";
    throw std::runtime_error(os.str().c_str());
  }
  if (securityMargins_(row, col) != margin) {
    securityMargins_(row, col) = margin;
    securityMargins_(col, row) = margin;
    invalidate();
  }
}

bool Edge::intersectionConstraint(const EdgePtr_t& other,
                                  ConfigProjectorPtr_t proj) const {
  GraphPtr_t g = graph_.lock();

  g->insertNumericalConstraints(proj);
  insertNumericalConstraints(proj);
  state()->insertNumericalConstraints(proj);

  if (wkPtr_.lock() == other)  // No intersection to be computed.
    return false;

  bool stateB_Eq_stateA = (state() == other->state());

  other->insertNumericalConstraints(proj);
  if (!stateB_Eq_stateA) other->state()->insertNumericalConstraints(proj);
  return true;
}

EdgePtr_t Edge::create(const std::string& name, const GraphWkPtr_t& graph,
                       const StateWkPtr_t& from, const StateWkPtr_t& to) {
  Edge* ptr = new Edge(name);
  EdgePtr_t shPtr(ptr);
  ptr->init(shPtr, graph, from, to);
  return shPtr;
}

void Edge::init(const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph,
                const StateWkPtr_t& from, const StateWkPtr_t& to) {
  GraphComponent::init(weak);
  parentGraph(graph);
  wkPtr_ = weak;
  from_ = from;
  to_ = to;
  state_ = to;
  // add 1 joint for the environment
  size_type n(graph.lock()->robot()->nbJoints() + 1);
  securityMargins_.resize(n, n);
  securityMargins_.setZero();
}

void Edge::initialize() {
  if (!isInit_) {
    targetConstraints_ = buildTargetConstraint();
    pathConstraints_ = buildPathConstraint();
  }
  isInit_ = true;
}

std::ostream& Edge::print(std::ostream& os) const {
  os << "|   |   |-- ";
  GraphComponent::print(os) << " --> " << to_.lock()->name();
  return os;
}

std::ostream& Edge::dotPrint(std::ostream& os,
                             dot::DrawingAttributes da) const {
  da.insertWithQuote("label", name());
  da.insert("shape", "onormal");
  dot::Tooltip tp;
  tp.addLine("Edge constains:");
  populateTooltip(tp);
  da.insertWithQuote("tooltip", tp.toStr());
  da.insertWithQuote("labeltooltip", tp.toStr());
  os << stateFrom()->id() << " -> " << stateTo()->id() << " " << da << ";";
  return os;
}

ConstraintSetPtr_t Edge::targetConstraint() const {
  throwIfNotInitialized();
  return targetConstraints_;
}

// Merge constraints of several graph components into a config projector
// Replace constraints and complement by combination of both when
// necessary.
static void mergeConstraintsIntoConfigProjector(
    const ConfigProjectorPtr_t& proj,
    const std::vector<GraphComponentPtr_t>& components,
    const GraphPtr_t& graph) {
  NumericalConstraints_t nc;
  for (const auto& gc : components)
    nc.insert(nc.end(), gc->numericalConstraints().begin(),
              gc->numericalConstraints().end());

  // Remove duplicate constraints
  auto end = nc.end();
  for (auto it = nc.begin(); it != end; ++it)
    end = std::remove(std::next(it), end, *it);
  nc.erase(end, nc.end());

  NumericalConstraints_t::iterator itnc1, itnc2;

  // Look for complement
  for (itnc1 = nc.begin(); itnc1 != nc.end(); ++itnc1) {
    const auto& c1 = *itnc1;
    itnc2 = std::next(itnc1);
    constraints::ImplicitPtr_t combination;
    itnc2 = std::find_if(std::next(itnc1), nc.end(),
                         [&c1, &combination, &graph](const auto& c2) -> bool {
                           assert(c1 != c2);
                           return graph->isComplement(c1, c2, combination) ||
                                  graph->isComplement(c2, c1, combination);
                         });
    if (itnc2 != nc.end()) {
      assert(*itnc1 != *itnc2);
      *itnc1 = combination;
      nc.erase(itnc2);
    }
  }

  for (const auto& _nc : nc) proj->add(_nc);
  // Insert numerical costs
  nc.clear();
  for (const auto& gc : components)
    nc.insert(nc.end(), gc->numericalCosts().begin(),
              gc->numericalCosts().end());
  for (const auto& _nc : nc) proj->add(_nc, 1);
}

ConstraintSetPtr_t Edge::buildTargetConstraint() {
  std::string n = "(" + name() + ")";
  GraphPtr_t g = graph_.lock();

  ConstraintSetPtr_t constraint = ConstraintSet::create(g->robot(), "Set " + n);

  ConfigProjectorPtr_t proj = ConfigProjector::create(
      g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
  proj->solver().solveLevelByLevel(this->solveLevelByLevel());
  std::vector<GraphComponentPtr_t> components;
  components.push_back(g);
  components.push_back(wkPtr_.lock());
  components.push_back(stateTo());
  if (state() != stateTo()) {
    components.push_back(state());
  }
  // Copy constraints from
  // - graph,
  // - this edge,
  // - the destination state,
  // - the state in which the transition lies if different
  mergeConstraintsIntoConfigProjector(proj, components, parentGraph());

  constraint->addConstraint(proj);
  constraint->edge(wkPtr_.lock());
  return constraint;
}

ConstraintSetPtr_t Edge::pathConstraint() const {
  throwIfNotInitialized();
  return pathConstraints_;
}

ConstraintSetPtr_t Edge::buildPathConstraint() {
  std::string n = "(" + name() + ")";
  GraphPtr_t g = graph_.lock();

  ConstraintSetPtr_t constraint = ConstraintSet::create(g->robot(), "Set " + n);

  ConfigProjectorPtr_t proj = ConfigProjector::create(
      g->robot(), "proj_" + n, .5 * g->errorThreshold(), g->maxIterations());
  proj->solver().solveLevelByLevel(this->solveLevelByLevel());
  std::vector<GraphComponentPtr_t> components;
  components.push_back(g);
  components.push_back(wkPtr_.lock());
  components.push_back(state());
  mergeConstraintsIntoConfigProjector(proj, components, parentGraph());

  constraint->addConstraint(proj);
  constraint->edge(wkPtr_.lock());

  // Build steering method
  const ProblemPtr_t& problem(g->problem());
  steeringMethod_ =
      problem->manipulationSteeringMethod()->innerSteeringMethod()->copy();
  steeringMethod_->constraints(constraint);
  // Build path validation and relative motion matrix
  // TODO this path validation will not contain obstacles added after
  // its creation.
  pathValidation_ = problem->pathValidationFactory();
  shared_ptr<core::ObstacleUserInterface> oui =
      HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pathValidation_);
  if (oui) {
    relMotion_ = RelativeMotion::matrix(g->robot());
    RelativeMotion::fromConstraint(relMotion_, g->robot(), constraint);
    oui->filterCollisionPairs(relMotion_);
    oui->setSecurityMargins(securityMargins_);
  }
  return constraint;
}

bool Edge::canConnect(ConfigurationIn_t q1, ConfigurationIn_t q2) const {
  ConstraintSetPtr_t constraints = pathConstraint();
  constraints->configProjector()->rightHandSideFromConfig(q1);
  if (!constraints->isSatisfied(q1) || !constraints->isSatisfied(q2)) {
    return false;
  }
  return true;
}

bool Edge::build(core::PathPtr_t& path, ConfigurationIn_t q1,
                 ConfigurationIn_t q2) const {
  using pinocchio::displayConfig;
  if (!steeringMethod_) {
    std::ostringstream oss;
    oss << "No steering method set in edge " << name() << ".";
    throw std::runtime_error(oss.str().c_str());
  }
  ConstraintSetPtr_t constraints = pathConstraint();
  constraints->configProjector()->rightHandSideFromConfig(q1);
  if (constraints->isSatisfied(q1)) {
    if (constraints->isSatisfied(q2)) {
      path = (*steeringMethod_)(q1, q2);
      return (bool)path;
    } else {
      hppDout(info, "q2 = " << displayConfig(q2)
                            << " does not satisfy the constraints of edge "
                            << name());
      hppDout(info, "q1 = " << displayConfig(q1));
      return false;
    }
  } else {
    value_type th(constraints->configProjector()->errorThreshold());
    if (!constraints->configProjector()->isSatisfied(q1, 2 * th)) {
      std::ostringstream oss;
      oss << "The initial configuration " << displayConfig(q1)
          << " does not satisfy the constraints of"
             " edge "
          << name() << "." << std::endl;
      oss << "The graph is probably malformed";
      throw std::runtime_error(oss.str().c_str());
    }
    // q1 may slightly violate the edge constraint eventhough the graph
    // is well constructed.
    return false;
  }
}

bool Edge::generateTargetConfig(core::NodePtr_t nStart,
                                ConfigurationOut_t q) const {
  return generateTargetConfig(*(nStart->configuration()), q);
}

bool Edge::generateTargetConfig(ConfigurationIn_t qStart,
                                ConfigurationOut_t q) const {
  ConstraintSetPtr_t c = targetConstraint();
  ConfigProjectorPtr_t proj = c->configProjector();
  proj->rightHandSideFromConfig(qStart);
  if (isShort_) q = qStart;
  if (c->apply(q)) return true;
  const ::hpp::statistics::SuccessStatistics& ss = proj->statistics();
  if (ss.nbFailure() > ss.nbSuccess()) {
    hppDout(warning, c->name() << " fails often.\n" << ss);
  } else {
    hppDout(warning, c->name() << " succeeds at rate "
                               << (value_type)(ss.nbSuccess()) /
                                      (value_type)ss.numberOfObservations()
                               << ".");
  }
  return false;
}

WaypointEdgePtr_t WaypointEdge::create(const std::string& name,
                                       const GraphWkPtr_t& graph,
                                       const StateWkPtr_t& from,
                                       const StateWkPtr_t& to) {
  WaypointEdge* ptr = new WaypointEdge(name);
  WaypointEdgePtr_t shPtr(ptr);
  ptr->init(shPtr, graph, from, to);
  return shPtr;
}

void WaypointEdge::init(const WaypointEdgeWkPtr_t& weak,
                        const GraphWkPtr_t& graph, const StateWkPtr_t& from,
                        const StateWkPtr_t& to) {
  Edge::init(weak, graph, from, to);
  nbWaypoints(0);
  wkPtr_ = weak;
}

void WaypointEdge::initialize() {
  Edge::initialize();
  // Set error threshold of internal edge to error threshold of
  // waypoint edge divided by number of edges.
  assert(targetConstraint()->configProjector());
  value_type eps(targetConstraint()->configProjector()->errorThreshold() /
                 (value_type)edges_.size());
  for (Edges_t::iterator it(edges_.begin()); it != edges_.end(); ++it) {
    (*it)->initialize();
    assert((*it)->targetConstraint());
    assert((*it)->targetConstraint()->configProjector());
    (*it)->targetConstraint()->configProjector()->errorThreshold(eps);
  }
}

bool WaypointEdge::canConnect(ConfigurationIn_t q1,
                              ConfigurationIn_t q2) const {
  /// TODO: This is not correct
  for (std::size_t i = 0; i < edges_.size(); ++i)
    if (!edges_[i]->canConnect(q1, q2)) return false;
  return true;
}

bool WaypointEdge::build(core::PathPtr_t& path, ConfigurationIn_t q1,
                         ConfigurationIn_t q2) const {
  core::PathPtr_t p;
  core::PathVectorPtr_t pv =
      core::PathVector::create(graph_.lock()->robot()->configSize(),
                               graph_.lock()->robot()->numberDof());
  // Many times, this will be called rigth after
  // WaypointEdge::generateTargetConfig so config_ already satisfies the
  // constraints.
  size_type n = edges_.size();
  assert(configs_.cols() == n + 1);
  bool useCache = lastSucceeded_ && configs_.col(0).isApprox(q1) &&
                  configs_.col(n).isApprox(q2);
  configs_.col(0) = q1;
  configs_.col(n) = q2;

  for (size_type i = 0; i < n; ++i) {
    if (i < (n - 1) && !useCache) configs_.col(i + 1) = q2;
    if (i < (n - 1) && !edges_[i]->generateTargetConfig(configs_.col(i),
                                                        configs_.col(i + 1))) {
      hppDout(info, "Waypoint edge "
                        << name()
                        << ": generateTargetConfig failed at waypoint " << i
                        << "."
                        << "\nUse cache: " << useCache);
      lastSucceeded_ = false;
      return false;
    }
    assert(targetConstraint());
    assert(targetConstraint()->configProjector());
    value_type eps(targetConstraint()->configProjector()->errorThreshold());
    if ((configs_.col(i) - configs_.col(i + 1)).squaredNorm() > eps * eps) {
      if (!edges_[i]->build(p, configs_.col(i), configs_.col(i + 1))) {
        hppDout(info, "Waypoint edge "
                          << name() << ": build failed at waypoint " << i << "."
                          << "\nUse cache: " << useCache);
        lastSucceeded_ = false;
        return false;
      }
      pv->appendPath(p);
    }
  }

  path = pv;
  lastSucceeded_ = true;
  return true;
}

bool WaypointEdge::generateTargetConfig(ConfigurationIn_t qStart,
                                        ConfigurationOut_t q) const {
  assert(configs_.cols() == size_type(edges_.size() + 1));
  configs_.col(0) = qStart;
  for (std::size_t i = 0; i < edges_.size(); ++i) {
    configs_.col(i + 1) = q;
    if (!edges_[i]->generateTargetConfig(configs_.col(i),
                                         configs_.col(i + 1))) {
      q = configs_.col(i + 1);
      lastSucceeded_ = false;
      return false;
    }
  }
  q = configs_.col(edges_.size());
  lastSucceeded_ = true;
  return true;
}

void WaypointEdge::nbWaypoints(const size_type number) {
  edges_.resize(number + 1);
  states_.resize(number + 1);
  states_.back() = stateTo();
  const size_type nbDof = graph_.lock()->robot()->configSize();
  configs_ = matrix_t(nbDof, number + 2);
  invalidate();
}

void WaypointEdge::setWaypoint(const std::size_t index, const EdgePtr_t wEdge,
                               const StatePtr_t wTo) {
  assert(edges_.size() == states_.size());
  assert(index < edges_.size());
  if (index == states_.size() - 1) {
    assert(!wTo || wTo == stateTo());
  } else {
    states_[index] = wTo;
  }
  edges_[index] = wEdge;
}

const EdgePtr_t& WaypointEdge::waypoint(const std::size_t index) const {
  assert(index < edges_.size());
  return edges_[index];
}

std::ostream& WaypointEdge::print(std::ostream& os) const {
  os << "|   |   |-- ";
  GraphComponent::print(os) << " (waypoint) --> " << stateTo()->name();
  return os;
}

std::ostream& WaypointEdge::dotPrint(std::ostream& os,
                                     dot::DrawingAttributes da) const {
  // First print the waypoint node, then the first edge.
  da["style"] = "dashed";
  for (std::size_t i = 0; i < states_.size() - 1; ++i)
    states_[i]->dotPrint(os, da);

  da["style"] = "solid";
  for (std::size_t i = 0; i < edges_.size(); ++i)
    edges_[i]->dotPrint(os, da) << std::endl;

  da["style"] = "dotted";
  da["dir"] = "both";
  da["arrowtail"] = "dot";

  return Edge::dotPrint(os, da);
}

std::ostream& LevelSetEdge::print(std::ostream& os) const {
  os << "|   |   |-- ";
  GraphComponent::print(os) << " (level set) --> " << stateTo()->name();
  return os;
}

std::ostream& LevelSetEdge::dotPrint(std::ostream& os,
                                     dot::DrawingAttributes da) const {
  da.insert("shape", "onormal");
  da.insert("style", "dashed");
  return Edge::dotPrint(os, da);
}

void LevelSetEdge::populateTooltip(dot::Tooltip& tp) const {
  GraphComponent::populateTooltip(tp);
  tp.addLine("");
  tp.addLine("Foliation condition constraints:");
  for (NumericalConstraints_t::const_iterator it =
           condNumericalConstraints_.begin();
       it != condNumericalConstraints_.end(); ++it) {
    tp.addLine("- " + (*it)->function().name());
  }
  tp.addLine("Foliation parametrization constraints:");
  for (NumericalConstraints_t::const_iterator it =
           paramNumericalConstraints_.begin();
       it != paramNumericalConstraints_.end(); ++it) {
    tp.addLine("- " + (*it)->function().name());
  }
}

bool LevelSetEdge::generateTargetConfig(ConfigurationIn_t qStart,
                                        ConfigurationOut_t q) const {
  // First, get an offset from the histogram
  statistics::DiscreteDistribution<RoadmapNodePtr_t> distrib =
      hist_->getDistrib();
  if (distrib.size() == 0) {
    hppDout(warning, "Edge " << name() << ": Distrib is empty");
    return false;
  }
  const Configuration_t& qLeaf = *(distrib()->configuration());

  return generateTargetConfigOnLeaf(qStart, qLeaf, q);
}

bool LevelSetEdge::generateTargetConfig(core::NodePtr_t nStart,
                                        ConfigurationOut_t q) const {
  // First, get an offset from the histogram that is not in the same connected
  // component.
  statistics::DiscreteDistribution<RoadmapNodePtr_t> distrib =
      hist_->getDistribOutOfConnectedComponent(nStart->connectedComponent());
  if (distrib.size() == 0) {
    hppDout(warning, "Edge " << name() << ": Distrib is empty");
    return false;
  }
  const Configuration_t &qLeaf = *(distrib()->configuration()),
                        qStart = *(nStart->configuration());

  return generateTargetConfigOnLeaf(qStart, qLeaf, q);
}

bool LevelSetEdge::generateTargetConfigOnLeaf(ConfigurationIn_t qStart,
                                              ConfigurationIn_t qLeaf,
                                              ConfigurationOut_t q) const {
  // First, set the offset.
  ConstraintSetPtr_t cs = targetConstraint();
  const ConfigProjectorPtr_t cp = cs->configProjector();
  assert(cp);

  // Set right hand side of edge constraints with qStart
  cp->rightHandSideFromConfig(qStart);
  // Set right hand side of constraints parameterizing the target state
  // foliation with qLeaf.
  for (NumericalConstraints_t::const_iterator it =
           paramNumericalConstraints_.begin();
       it != paramNumericalConstraints_.end(); ++it) {
    cp->rightHandSideFromConfig(*it, qLeaf);
  }

  // Eventually, do the projection.
  if (isShort_) q = qStart;
  if (cs->apply(q)) return true;
  ::hpp::statistics::SuccessStatistics& ss = cp->statistics();
  if (ss.nbFailure() > ss.nbSuccess()) {
    hppDout(warning, cs->name() << " fails often." << std::endl << ss);
  } else {
    hppDout(warning, cs->name() << " succeeds at rate "
                                << (value_type)(ss.nbSuccess()) /
                                       (value_type)ss.numberOfObservations()
                                << ".");
  }
  return false;
}

void LevelSetEdge::init(const LevelSetEdgeWkPtr_t& weak,
                        const GraphWkPtr_t& graph, const StateWkPtr_t& from,
                        const StateWkPtr_t& to) {
  Edge::init(weak, graph, from, to);
  wkPtr_ = weak;
}

LevelSetEdgePtr_t LevelSetEdge::create(const std::string& name,
                                       const GraphWkPtr_t& graph,
                                       const StateWkPtr_t& from,
                                       const StateWkPtr_t& to) {
  LevelSetEdge* ptr = new LevelSetEdge(name);
  LevelSetEdgePtr_t shPtr(ptr);
  ptr->init(shPtr, graph, from, to);
  return shPtr;
}

LeafHistogramPtr_t LevelSetEdge::histogram() const { return hist_; }

void LevelSetEdge::buildHistogram() {
  Foliation f;

  /// Build the constraint set.
  std::string n = "(" + name() + ")";
  GraphPtr_t g = graph_.lock();

  // The parametrizer
  ConstraintSetPtr_t param = ConstraintSet::create(g->robot(), "Set " + n);

  ConfigProjectorPtr_t proj = ConfigProjector::create(
      g->robot(), "projParam_" + n, g->errorThreshold(), g->maxIterations());

  for (const auto& nc : paramNumericalConstraints_) proj->add(nc);

  param->addConstraint(proj);
  param->edge(wkPtr_.lock());

  f.parametrizer(param);

  // The codition
  // TODO: We assumed that this part of the code can only be reached by
  // configurations that are valid.
  // It would be wiser to make sure configurations are valid, for instance
  // by considering only configurations in the destination state of this
  // edge.
  ConstraintSetPtr_t cond = ConstraintSet::create(g->robot(), "Set " + n);
  proj = ConfigProjector::create(g->robot(), "projCond_" + n,
                                 g->errorThreshold(), g->maxIterations());

  for (const auto& nc : condNumericalConstraints_) proj->add(nc);

  f.condition(cond);
  cond->addConstraint(proj);

  hppDout(info, "Build histogram of LevelSetEdge " << name()
                                                   << "\nParametrizer:\n"
                                                   << *param << "\nCondition:\n"
                                                   << *cond);

  // TODO: If hist_ is not NULL, remove the previous Histogram.
  // It should not be of any use and it slows down node insertion in the
  // roadmap.
  hist_ = LeafHistogram::create(f);
  g->insertHistogram(hist_);
}

void LevelSetEdge::initialize() {
  if (!isInit_) {
    Edge::initialize();
    buildHistogram();
  }
}

ConstraintSetPtr_t LevelSetEdge::buildTargetConstraint() {
  std::string n = "(" + name() + ")";
  GraphPtr_t g = graph_.lock();

  ConstraintSetPtr_t constraint = ConstraintSet::create(g->robot(), "Set " + n);

  ConfigProjectorPtr_t proj = ConfigProjector::create(
      g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());

  // Copy constraints from
  // - graph,
  // - param numerical constraints
  // - this edge,
  // - the destination state,
  // - the state in which the transition lies if different

  g->insertNumericalConstraints(proj);
  for (const auto& nc : paramNumericalConstraints_) proj->add(nc);

  insertNumericalConstraints(proj);
  stateTo()->insertNumericalConstraints(proj);
  if (state() != stateTo()) {
    state()->insertNumericalConstraints(proj);
  }
  constraint->addConstraint(proj);

  constraint->edge(wkPtr_.lock());
  return constraint;
}

void LevelSetEdge::insertParamConstraint(const constraints::ImplicitPtr_t& nm) {
  invalidate();
  paramNumericalConstraints_.push_back(nm);
}

const NumericalConstraints_t& LevelSetEdge::paramConstraints() const {
  return paramNumericalConstraints_;
}

void LevelSetEdge::insertConditionConstraint(
    const constraints::ImplicitPtr_t& nm) {
  invalidate();
  condNumericalConstraints_.push_back(nm);
}

const NumericalConstraints_t& LevelSetEdge::conditionConstraints() const {
  return condNumericalConstraints_;
}

LevelSetEdge::LevelSetEdge(const std::string& name) : Edge(name) {}

LevelSetEdge::~LevelSetEdge() {}
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp
