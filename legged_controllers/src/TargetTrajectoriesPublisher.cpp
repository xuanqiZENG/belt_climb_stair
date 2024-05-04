//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
scalar_t Body_height=0.43;
scalar_t Body_height_max=0.45;
scalar_t Body_height_min=0.08;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
scalar_t pos_limit = 0.8;
vector_t command_global_intergation = vector_t::Zero(3);
vector_t rpy_des = vector_t::Zero(2);
float current_state = 0;
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = Body_height;
  currentPose(4) = rpy_des(1);
  currentPose(5) = rpy_des(0);
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = Body_height;
    target(3) = goal(3);
    target(4) = rpy_des(1);
    target(5) = rpy_des(0);
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  Eigen::Matrix<scalar_t, 3, 1> zyx;
  zyx<< currentPose(3),0,0;
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);
  const scalar_t timeToTarget = TIME_TO_TARGET;
  // Body_height = Body_height + cmdVel(2) * timeToTarget;
  // Body_height = 0.1;
  // std::cout<<Body_height<<std::endl;
  Body_height= std::max(Body_height_min, std::min(Body_height, Body_height_max));
  command_global_intergation(0) = command_global_intergation(0)+cmdVelRot(0) * timeToTarget;
  command_global_intergation(1) = command_global_intergation(1)+cmdVelRot(1) * timeToTarget;
  // if (command_global_intergation(0) - currentPose(0) > pos_limit){
  //     command_global_intergation(0) = currentPose(0) + pos_limit;
  // }
  // else if (command_global_intergation(0) - currentPose(0) < -pos_limit){
  //     command_global_intergation(0) = currentPose(0) - pos_limit;
  // }

  // if (command_global_intergation(1) - currentPose(1) > pos_limit){
  //     command_global_intergation(1) = currentPose(1) + pos_limit;
  // }
  // else if (command_global_intergation(1) - currentPose(1) < -pos_limit){
  //     command_global_intergation(1) = currentPose(1) - pos_limit;
  // }
  // std::cout<<"command"<<std::endl;
  // std::cout<<command_global_intergation<<std::endl;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    // target(0) = command_global_intergation(0);
    // target(1) = command_global_intergation(1);
    target(2) = Body_height;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    // target(3) = currentPose(3);
    target(4) = rpy_des(1);
    target(5) = rpy_des(0);
    return target;
  }();
  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

 void groundestCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
   rpy_des(0)= msg->data[0];   //theta_x
   rpy_des(1)= msg->data[1];   //theta_y
 }

void FSMCallback (const std_msgs::Float64& msg) {
  current_state =  msg.data;
}

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);
  ros::Subscriber ground_est_sub=nodeHandle.subscribe("/ground_est",1,groundestCallback);
  ros::Subscriber FSMsub= nodeHandle.subscribe("/FSM", 1, FSMCallback);
  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);

  ros::spin();
  // Successful exit
  return 0;
}
