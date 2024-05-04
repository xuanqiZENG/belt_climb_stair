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
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <functional>
#include <memory>
#include <mutex>

#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include <std_msgs/Float64.h>

using namespace ocs2;
using namespace legged_robot;
std::string lastGaitCommand_ = "stance";
std::string GaitCommand_;
float current_state = 0;
void GaitCallback(const sensor_msgs::Joy::ConstPtr& msg){
  if (current_state==1) { 
  if (msg->buttons[0] == 1 && msg->buttons[7]==1){
    GaitCommand_ = "static_walk";
  }
  else if (msg->buttons[1] == 1 && msg->buttons[7]==1){
    GaitCommand_ = "stance";
  }
  else if (msg->buttons[2] == 1 && msg->buttons[7]==1){
    GaitCommand_ = "trot";
  }
  else if (msg->buttons[3] == 1 && msg->buttons[7]==1){
    GaitCommand_ = "climb_stair";
  }
  }
  else if (current_state==0){
    GaitCommand_ = "stance";
  }
 }

void FSMCallback (const std_msgs::Float64& msg) {
  current_state =  msg.data;
}

int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ros::NodeHandle nodehandle;
  ros::Rate loop_rate(10);
  // Get node parameters
  std::string gaitCommandFile;
  ros::Publisher gait_pub;
  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;
  nodehandle.getParam("/gaitCommandFile", gaitCommandFile);
  loadData::loadStdVector(gaitCommandFile, "list", gaitList_, false);
  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitCommandFile, gaitName, false)});
  }
  gait_pub = nodehandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true); 
  ros::Subscriber sub=nodehandle.subscribe("/joy",1,GaitCallback);
  ros::Subscriber FSMsub= nodehandle.subscribe("/FSM", 1, FSMCallback);

  while (ros::ok() && ros::master::check()) {
    if (GaitCommand_ == "stance" && lastGaitCommand_ != "stance"){
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at("stance");
      gait_pub.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      lastGaitCommand_ = "stance";
    }
    else if (GaitCommand_ == "trot" && lastGaitCommand_ != "trot"){
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at("trot");
      gait_pub.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      lastGaitCommand_ = "trot";
    }
    else if (GaitCommand_ == "climb_stair" && lastGaitCommand_ != "climb_stair"){
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at("climb_stair");
      gait_pub.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      lastGaitCommand_ = "climb_stair";
    }
    else if (GaitCommand_ == "static_walk" && lastGaitCommand_ != "static_walk"){
      ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at("static_walk");
      gait_pub.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      lastGaitCommand_ = "static_walk";
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
