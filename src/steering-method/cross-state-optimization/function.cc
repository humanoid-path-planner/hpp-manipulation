// Copyright (c) 2017, Joseph Mirabel
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

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    namespace steeringMethod {
      typedef core::segment_t segment_t;
      namespace {
        std::string toStr (const segment_t& s)
        {
          std::ostringstream os;
          os << "[ " << s.first << ", " << s.first + s.second << " ]";
          return os.str();
        }
      }

      /// Apply the constraint on a subspace of the input space.
      /// i.e.: \f$ f (q_0, ... , q_n) = f_{inner} (q_k) \f$
      class HPP_MANIPULATION_LOCAL StateFunction :
        public constraints::DifferentiableFunction
      {
        public:
          typedef boost::shared_ptr<StateFunction> Ptr_t;
          StateFunction (const DifferentiableFunctionPtr_t& inner,
              const size_type& nArgs, const size_type& nDers,
              const segment_t& inArgs, const segment_t& inDers) :
            DifferentiableFunction (nArgs, nDers,
                inner->outputSpace(), inner->name() + " | " + toStr(inArgs)),
            inner_ (inner), sa_ (inArgs), sd_ (inDers)
          {
            activeParameters_.setConstant(false);
            activeParameters_.segment(sa_.first, sa_.second)
              = inner_->activeParameters();

            activeDerivativeParameters_.setConstant(false);
            activeDerivativeParameters_.segment(sd_.first, sd_.second)
              = inner_->activeDerivativeParameters();
          }

        protected:
          void impl_compute (LiegroupElement& y, vectorIn_t arg) const
          {
            inner_->value(y, arg.segment (sa_.first, sa_.second));
          }

          void impl_jacobian (matrixOut_t J, vectorIn_t arg) const
          {
            inner_->jacobian(J.middleCols (sd_.first, sd_.second),
                arg.segment (sa_.first, sa_.second));
          }

          std::ostream& print (std::ostream& os) const
          {
            constraints::DifferentiableFunction::print(os);
            return os << incindent << iendl << *inner_ << decindent;
          }

          DifferentiableFunctionPtr_t inner_;
          const segment_t sa_, sd_;
      }; // class Function

      /// Compute the difference between the value of the function in two points.
      /// i.e.: \f$ f (q_0, ... , q_n) = f_{inner} (q_{left}) - f_{inner} (q_{right}) \f$
      class HPP_MANIPULATION_LOCAL EdgeFunction :
        public constraints::DifferentiableFunction
      {
        public:
          typedef boost::shared_ptr<EdgeFunction> Ptr_t;
          EdgeFunction (const DifferentiableFunctionPtr_t& inner,
              const size_type& nArgs, const size_type& nDers,
              const segment_t& lInArgs, const segment_t& lInDers,
              const segment_t& rInArgs, const segment_t& rInDers) :
            DifferentiableFunction (nArgs, nDers,
                LiegroupSpace::Rn(inner->outputSpace()->nv()),
                inner->name() + " | " + toStr(lInArgs) + " - " + toStr(rInArgs)),
            inner_ (inner),
            lsa_ (lInArgs), lsd_ (lInDers),
            rsa_ (rInArgs), rsd_ (rInDers),
            l_ (inner->outputSpace()), r_ (inner->outputSpace())
          {
            activeParameters_.setConstant(false);
            activeParameters_.segment(lsa_.first, lsa_.second)
              = inner_->activeParameters();
            activeParameters_.segment(rsa_.first, rsa_.second)
              = inner_->activeParameters();

            activeDerivativeParameters_.setConstant(false);
            activeDerivativeParameters_.segment(lsd_.first, lsd_.second)
              = inner_->activeDerivativeParameters();
            activeDerivativeParameters_.segment(rsd_.first, rsd_.second)
              = inner_->activeDerivativeParameters();
          }

        protected:
          void impl_compute (LiegroupElement& y, vectorIn_t arg) const
          {
            inner_->value(l_, arg.segment (lsa_.first, lsa_.second));
            inner_->value(r_, arg.segment (rsa_.first, rsa_.second));
            y.vector() = l_ - r_;
          }

          void impl_jacobian (matrixOut_t J, vectorIn_t arg) const
          {
            inner_->jacobian(
                J.middleCols (lsd_.first, lsd_.second),
                arg.segment (lsa_.first, lsa_.second));
            inner_->jacobian(
                J.middleCols (rsd_.first, rsd_.second),
                arg.segment (rsa_.first, rsa_.second));
            J.middleCols (rsd_.first, rsd_.second) *= -1;
          }

          std::ostream& print (std::ostream& os) const
          {
            constraints::DifferentiableFunction::print(os);
            return os << incindent << iendl << *inner_ << decindent;
          }

          DifferentiableFunctionPtr_t inner_;
          const segment_t lsa_, lsd_;
          const segment_t rsa_, rsd_;

          mutable LiegroupElement l_, r_;
      }; // class Function
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp
