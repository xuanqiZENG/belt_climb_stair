

#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

class FootPositionConstraint final : public StateInputConstraint {
 public:

  FootPositionConstraint(const SwitchedModelReferenceManager& referenceManager, const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex, vector_t& alpha);
  ~FootPositionConstraint() override = default;
  FootPositionConstraint* clone() const override { return new FootPositionConstraint(*referenceManagerPtr_,*endEffectorKinematicsPtr_, contactPointIndex_, *alpha_); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }
 
 private:
  FootPositionConstraint(const FootPositionConstraint& other) = default;

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const size_t contactPointIndex_;
  vector_t* alpha_;
};

}  // namespace legged_robot
}  // namespace ocs2
