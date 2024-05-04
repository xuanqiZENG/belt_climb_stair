

#include "legged_interface/constraint/FootPositionConstraint.h"
#include <cmath>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FootPositionConstraint::FootPositionConstraint(const SwitchedModelReferenceManager& referenceManager, const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex, vector_t& alpha)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      alpha_(&alpha) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool FootPositionConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] && referenceManagerPtr_->getContactFlags(time+0.07)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FootPositionConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  vector_t f = vector_t::Zero(1);
  vector_t pos = vector_t::Zero(3);
  vector_t alpha_con = vector_t::Zero(3);
  alpha_con = *alpha_;
  pos = endEffectorKinematicsPtr_->getPosition(state).front().segment<3>(0);
  f(0) = -(alpha_con(0)+alpha_con(1)*pos(0)+alpha_con(2)*pos(1)) + pos(2);
  // f(1) = (alpha_con(0)+alpha_con(1)*pos(0)+alpha_con(2)*pos(1))+0.01 -pos(2);
  std::cout<<f(0)<<std::endl;
  std::cout<<" "<<std::endl;
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FootPositionConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                                 const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
  auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  positionApprox.dfdx = positionApprox.dfdx;
  linearApproximation.f = getValue(time, state, input, preComp);
  linearApproximation.dfdx = positionApprox.dfdx.row(2);
  linearApproximation.dfdu = matrix_t::Zero(1, input.size());
  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
