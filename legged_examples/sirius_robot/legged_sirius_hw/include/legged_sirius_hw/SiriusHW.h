
//
// Created by qiayuan on 1/24/22.
//

#pragma once
#include "ros_lowstate_lcmt.hpp"
#include "ros_lowcmd_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "lcm-cpp.hpp"
#include <legged_hw/LeggedHW.h>
#include "thread"
#include <ros/subscriber.h>
#include <ros/package.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct SiriusMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct SiriusImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class SiriusHW : public LeggedHW {
 public:
  SiriusHW();
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;
  sensor_msgs::Joy joy_command;

 private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  SiriusMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  SiriusImuData imuData_{};
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};
  lcm::LCM cheetah2ROS;
  lcm::LCM ROS2cheetah;
  lcm::LCM gamepad2controller;
  ros_lowcmd_lcmt ros_lowcmd;
  ros_lowstate_lcmt ros_lowstate;
  gamepad_lcmt gamepad;
  std::thread recv_thread;
  std::thread recv_thread_gamepad;
  ros::Subscriber sub;
  ros::Publisher joyPublisher_; 
  ros::Publisher FSMPublisher_; 
  std_msgs::Float64 current_state;
  void handleROS2CheetahLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const ros_lowstate_lcmt* msg); 
  void handlegamepad2controller(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg); 
};

}  // namespace legged
