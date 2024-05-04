/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <string>

#include <ros/init.h>
#include <ros/package.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <functional>
#include <memory>
#include <mutex>
#include <ocs2_msgs/mpc_observation.h>

#include <std_msgs/Float64MultiArray.h>

#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <std_msgs/Float64.h>
using namespace ocs2;

namespace {
scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t comHeight;
vector_t defaultJointState(12);
}  // namespace
scalar_t Body_height=0.48;
scalar_t Body_height_max=0.5;
scalar_t Body_height_min=0.0;
vector_t command = vector_t::Zero(7);
vector_t rpy_des = vector_t::Zero(3);
vector_t command_global_intergation = vector_t::Zero(3);
std::mutex latestObservationMutex_;
SystemObservation latestObservation_;
SystemObservation observation;
float current_state = 0;
auto observationCallback (const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
}

scalar_t estimateTimeToTarget(vector_t desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t& dpitch = desiredBaseDisplacement(4);
  const scalar_t rotationTime_yaw = std::abs(dyaw) / targetRotationVelocity;
  const scalar_t rotationTime_pitch = std::abs(dpitch) / targetRotationVelocity;
  const scalar_t rotationTime = std::max(rotationTime_yaw, rotationTime_pitch);
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity;
  return std::max(rotationTime_yaw, displacementTime);
}


TargetTrajectories getTargetTrajectories() {
  std::lock_guard<std::mutex> lock(latestObservationMutex_);
  observation = latestObservation_;
  vector_t currentPose = vector_t::Zero(6);
  if (latestObservation_.time == 0.0) {
  currentPose = vector_t::Zero(6);
  }
  else {
  currentPose = observation.state.segment<6>(6);
  }
  
  // const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  Eigen::Matrix<scalar_t, 3, 1> zyx;
  zyx << currentPose(3), currentPose(4), currentPose(5);
  vector_t command_global = getRotationMatrixFromZyxEulerAngles(zyx) * command.head(3);
   command_global_intergation = command_global_intergation + command_global;
  // if (command_global_intergation(0) - currentPose(0) > 0.2){
  //     command_global_intergation(0) = currentPose(0) + 0.2;
  // }
  // else if (command_global_intergation(0) - currentPose(0) < -0.2){
  //     command_global_intergation(0) = currentPose(0) - 0.2;
  // }

  // if (command_global_intergation(1) - currentPose(1) > 0.2){
  //     command_global_intergation(1) = currentPose(1) + 0.2;
  // }
  // else if (command_global_intergation(1) - currentPose(1) < -0.2){
  //     command_global_intergation(1) = currentPose(1) - 0.2;
  // }
  // Body_height=Body_height+command(2);
  // Body_height= std::max(Body_height_min, std::min(Body_height, Body_height_max));
  vector_t target(6);
    // base p_x, p_y are relative to current state
  target(0) = currentPose(0) + command_global(0);
  target(1) = currentPose(1) + command_global(1);
  // target(0) = command_global_intergation(0);
  // target(1) = command_global_intergation(1);
    // base z relative to the default height
  // target(2) = Body_height + command_global(2);
  target(2) = Body_height;
    // theta_z relative to current
  target(3) = currentPose(3) + command(3) * M_PI / 180.0;
  // target(3) = command(3) * M_PI / 180.0;
    // theta_y, theta_x
  // target(4) = 0.0;
  // target(5) = 0.0;
  target(4) = rpy_des(1);
  target(5) = rpy_des(0);
  // target(5) = 0;
  // std::cout<<rpy_des(1)<<std::endl;

  // target reaching duration
  scalar_t targetReachingTime = observation.time + estimateTimeToTarget(target - currentPose);

  scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(24));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState;
  stateTrajectory[1] << vector_t::Zero(6), target, defaultJointState;
  // desired input trajectory (just right dimensions, they are not used)
  vector_array_t inputTrajectory(2, vector_t::Zero(24));
  
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}


void rccommandCallback(const sensor_msgs::Joy::ConstPtr& msg){
   command(0)= 0.7*command(0)+0.3*0.6*msg->axes[1];   //x
   command(1)= 0.7*command(1)+0.3*0.3*msg->axes[0];   //y
  //  command(2)=0.002*msg->axes[7];  //body_height
   command(3)=15*msg->axes[3];
  //  command(3)=command(3) + 2*msg->axes[3];
   command(4)=msg->buttons[4];
   command(5)=msg->axes[2];
   command(6)=msg->axes[5];
 }

 void groundestCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
   rpy_des(0)= msg->data[0];   //theta_x
   rpy_des(1)= msg->data[1];   //theta_y
 }

 void FSMCallback (const std_msgs::Float64& msg) {
  current_state =  msg.data;
}

int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  ros::Rate loop_rate(10);
  nodeHandle.getParam("/referenceFile", referenceFile);
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity);
  ros::Subscriber sub=nodeHandle.subscribe("/joy",1,rccommandCallback);
  ros::Subscriber ground_est_sub=nodeHandle.subscribe("/ground_est",1,groundestCallback);
  ros::Subscriber FSMsub= nodeHandle.subscribe("/FSM", 1, FSMCallback);
  ros::Subscriber observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1, observationCallback);
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;
  targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, robotName));

  while (ros::ok() && ros::master::check()) {
    // if (current_state == 0){
    //   Body_height = 0.43;
    // }
    TargetTrajectories trajectories = getTargetTrajectories();
    targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    // std::cout<< current_state <<std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
