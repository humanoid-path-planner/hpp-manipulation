// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/numerical-constraint.hh>

#define ARM_LENGTH 1
#define FOREARM_LENGTH 1
#define STEP_PATH (value_type)0.01

#include <math.h>

#include <boost/assign/list_of.hpp>

#include <hpp/core/weighed-distance.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/problem.hh>

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/object-factory.hh>

#include <hpp/constraints/generic-transformation.hh>

#define REQUIRE_MESSAGE(b,m) do {\
  if (!b) {\
    std::cout << m << std::endl;\
    exit(1);\
  }}\
  while (0)

using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;
using hpp::model::BodyPtr_t;
using hpp::model::Configuration_t;
using hpp::model::value_type;

using hpp::constraints::Position;
using hpp::constraints::PositionPtr_t;
using hpp::constraints::matrix3_t;
using hpp::constraints::vector3_t;

using hpp::core::StraightPath;
using hpp::core::StraightPathPtr_t;
using hpp::core::Path;
using hpp::core::PathPtr_t;
using hpp::core::WeighedDistance;
using hpp::core::WeighedDistancePtr_t;
using hpp::core::SteeringMethodStraight;
using hpp::core::SteeringMethodPtr_t;
using hpp::core::ConstraintSet;
using hpp::core::ConstraintSetPtr_t;
using hpp::core::ConfigProjector;
using hpp::core::ConfigProjectorPtr_t;
using hpp::core::NumericalConstraint;

using boost::assign::list_of;

using hpp::core::pathProjector::ProgressivePtr_t;
using hpp::core::pathProjector::Progressive;

using hpp::core::Problem;
using hpp::core::ProblemPtr_t;

hpp::model::ObjectFactory objectFactory;

namespace hpp_test {
  DevicePtr_t createRobot ()
  {
    DevicePtr_t robot = Device::create ("test");
    BodyPtr_t body;
    fcl::Transform3f pos;
    fcl::Matrix3f orient;
    orient (0,0) = 1; orient (0,1) = 0; orient (0,2) = 0;
    orient (1,0) = 0; orient (1,1) = 1; orient (1,2) = 0;
    orient (2,0) = 0; orient (2,1) = 0; orient (2,2) = 1;
    pos.setRotation (orient);
    // Arm joint
    pos.setTranslation (fcl::Vec3f (0, 0, 0));
    JointPtr_t arm = objectFactory.createBoundedJointRotation (pos);
    robot->rootJoint (arm);
    arm->name ("ARM");
    body = objectFactory.createBody ();
    body->name ("ARM_BODY");
    arm->setLinkedBody (body);
    // Forearm joint
    pos.setTranslation (fcl::Vec3f (0, ARM_LENGTH, 0));
    JointPtr_t forearm = objectFactory.createBoundedJointRotation (pos);
    forearm->name ("FOREARM");
    arm->addChildJoint (forearm);
    body = objectFactory.createBody ();
    body->name ("FOREARM_BODY");
    forearm->setLinkedBody (body);
    // End effector joint
    pos.setTranslation (fcl::Vec3f (0, FOREARM_LENGTH, 0));
    JointPtr_t ee = objectFactory.createJointAnchor (pos);
    ee->name ("EE");
    body = objectFactory.createBody ();
    body->name ("EE");
    ee->setLinkedBody (body);
    forearm->addChildJoint (ee);
    //body = objectFactory.createBody ();
    //body->name ("RLEG_BODY2");
    //joint->setLinkedBody (body);

    return robot;
  }

  std::ostream& print (std::ostream& os, const Configuration_t& c)
  {
    os << "[ \t";
    for (int i = 0; i < c.size () - 1; i++)
      os << c[i] << ",\t";
    return os << c[c.size() - 1] << "]";
  }

  std::ostream& printpath (std::ostream& os, const Path& p)
  {
    value_type t = p.timeRange ().first;
    bool noWarning;
    while (t < p.timeRange().second) {
      print (os, p (t, noWarning)) << ",";
      t += STEP_PATH;
    }
    return print (os, p (p.timeRange ().second, noWarning));
  }

  namespace pythonscript {
    std::ostream& start (std::ostream& os)
    {
      os << "import numpy as np\n"
        << "import matplotlib.pyplot as plt\n";
      return os;
    }

    std::ostream& pathToVar (std::ostream& os, const Path& p, const std::string& var)
    {
      os << var << " = np.array ([\n";
      return printpath (os, p) << "\n])\n";
    }

    std::ostream& plot (std::ostream& os, const std::string& var)
    {
      os << "fig = plt.figure ()\n"
        << "axes = fig.gca ()\n"
        << "axes.plot (" << var << "[:,0], " << var << "[:,1], '-.', marker='+', markeredgecolor='r', markersize=5)\n"
        << "axes.set_xlabel ('Theta 1')\n"
        << "axes.set_ylabel ('Theta 2')\n"
        << "axes.set_title ('" << var << "')\n"
        << "plt.show ()\n";
      return os;
    }
  }
}

int main (int , char**) {
  DevicePtr_t r = hpp_test::createRobot ();
  JointPtr_t ee = r->getJointByName ("FOREARM");
  vector3_t target (0, (ARM_LENGTH + FOREARM_LENGTH ) / 2, 0),
            origin (0, FOREARM_LENGTH, 0);
  PositionPtr_t c = Position::create ("Pos", r, ee, origin, target,
      list_of (false)(true)(false).convert_to_container<std::vector<bool> >());
  ConstraintSetPtr_t cs = ConstraintSet::create (r, "test-cs");
  ConfigProjectorPtr_t proj = ConfigProjector::create (r, "test", 1e-4, 20);
  proj->add (NumericalConstraint::create (c));
  cs->addConstraint (proj);
  ProblemPtr_t problem (new Problem (r));
  WeighedDistancePtr_t dist = WeighedDistance::create (r, list_of (1)(1));
  problem->distance (dist);
  SteeringMethodPtr_t sm (SteeringMethodStraight::create (problem));
  const WeighedDistance& d = *dist;
  ProgressivePtr_t pp_ptr = Progressive::create (dist, sm, 0.1);
  Progressive pp = *pp_ptr;

  Configuration_t qinit (2), qgoal (2);
  qinit[0] =  M_PI / 3; qinit[1] = -M_PI / 3;
  qgoal[0] = -M_PI / 3; qgoal[1] =  M_PI / 3;
  Configuration_t qinitp = qinit,
                  qgoalp = qgoal;
  REQUIRE_MESSAGE (cs->apply (qinitp), "Could not project " << qinit);
  REQUIRE_MESSAGE (cs->apply (qgoalp), "Could not project " << qgoal);
  value_type l = d (qinitp, qgoalp);
  StraightPathPtr_t sp = StraightPath::create (r, qinitp, qgoalp, l);
  std::ostream& out = std::cout;
  hpp_test::pythonscript::start (out);
  std::string n = "direct_path";
  hpp_test::pythonscript::pathToVar (out, *sp, n);
  hpp_test::pythonscript::plot (out, n);
  sp = HPP_STATIC_PTR_CAST (StraightPath, sp->copy (cs));
  n = "naive_projection";
  hpp_test::pythonscript::pathToVar (out, *sp, n);
  hpp_test::pythonscript::plot (out, n);
  PathPtr_t pProj;
  pp.apply (sp, pProj);
  n = "proj_progressive";
  hpp_test::pythonscript::pathToVar (out, *pProj, n);
  hpp_test::pythonscript::plot (out, n);
}
