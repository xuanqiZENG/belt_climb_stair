
//
// Created by qiayuan on 1/24/22.
// Modified by Lz
//

#include "legged_unitree_hw_free/UnitreeHW.h"
#include <sensor_msgs/Joy.h>

namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);


  udp_ = std::make_shared<FDSC::UnitreeConnection>("LOW_WIRED_DEFAULTS");

  udp_->startRecv();

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);

  std::vector<uint8_t> cmd_bytes = lowCmd_.buildCmd(false);
  udp_->send(cmd_bytes);
  joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
  return true;
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  std::vector<std::vector<uint8_t>> dataall;// = conn.getData();
  udp_->getData(dataall);

  if (dataall.size()!=0)
  {
    std::vector<uint8_t> data = dataall.at(dataall.size()-1);
    lowState_.parseData(data);
    for (int i = 0; i < 12; ++i) {
      // ROS_WARN("Read : joint [%d]: q %f",i,lowState_.motorState[i].q);
      jointData_[i].pos_ = lowState_.motorState[i].q;
      jointData_[i].vel_ = lowState_.motorState[i].dq;
      jointData_[i].tau_ = lowState_.motorState[i].tauEst;
    }
    
    
    imuData_.ori_[0] = lowState_.imu_quaternion[1];
    imuData_.ori_[1] = lowState_.imu_quaternion[2];
    imuData_.ori_[2] = lowState_.imu_quaternion[3];
    imuData_.ori_[3] = lowState_.imu_quaternion[0];
    imuData_.angularVel_[0] = lowState_.imu_gyroscope[0];
    imuData_.angularVel_[1] = lowState_.imu_gyroscope[1];
    imuData_.angularVel_[2] = lowState_.imu_gyroscope[2];
    imuData_.linearAcc_[0] = lowState_.imu_accelerometer[0];
    imuData_.linearAcc_[1] = lowState_.imu_accelerometer[1];
    imuData_.linearAcc_[2] = lowState_.imu_accelerometer[2];
    for (int i = 0; i < 4; i++)
    {
      // ROS_WARN("Read : imu [%d]: imuData_.ori_ %f",i,imuData_.ori_[i]);
    }
    
    
    // we do not have in go1 air, will be delete in controller
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
      contactState_[i] = static_cast<float>(lowState_.footForce[i]) > contactThreshold_;
    }

    // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto& name : names) {
      HybridJointHandle handle = hybridJointInterface_.getHandle(name);
      handle.setFeedforward(0.);
      handle.setVelocityDesired(0.);
      handle.setKd(3.);
    }

    updateJoystick(time);
  }
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (int i = 0; i < 12; ++i) {
    mCmdArr_.motors[i].mode = FDSC::MotorModeLow::Servo;
    mCmdArr_.motors[i].q = static_cast<float>(jointData_[i].posDes_);
    mCmdArr_.motors[i].dq = static_cast<float>(jointData_[i].velDes_);
    mCmdArr_.motors[i].Kp = static_cast<float>(jointData_[i].kp_);
    mCmdArr_.motors[i].Kd = static_cast<float>(jointData_[i].kd_);
    // ROS_WARN("joint [%d]: q %f",i,mCmdArr_.motors[i].q);
    // ROS_WARN("joint [%d]: dq %f",i,mCmdArr_.motors[i].dq);
    // ROS_WARN("joint [%d]: kp %f",i,mCmdArr_.motors[i].Kp);
    // ROS_WARN("joint [%d]: kd %f",i,mCmdArr_.motors[i].Kd);
    // ROS_WARN("joint [%d]: tau %f",i,debug_tau_o);
    mCmdArr_.motors[i].tau = static_cast<float>(static_cast<float>(jointData_[i].ff_));
  }

    lowCmd_.motorCmd = mCmdArr_;
    std::vector<uint8_t> cmdBytes = lowCmd_.buildCmd(false);
    udp_->send(cmdBytes); 
}

bool UnitreeHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = static_cast<int>(FDSC::Leg::FR_);
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = static_cast<int>(FDSC::Leg::FL_);
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index =  static_cast<int>(FDSC::Leg::RR_);
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = static_cast<int>(FDSC::Leg::RL_);
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool UnitreeHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

void UnitreeHW::updateJoystick(const ros::Time& time) {
  if ((time - lastPub_).toSec() < 1 / 50.) {
    return;
  }
  lastPub_ = time;
  FDSC::xRockerBtnDataStruct keyData;
  memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
  sensor_msgs::Joy joyMsg;  // Pack as same as Logitech F710  I will change this code use other joystick
  joyMsg.axes.push_back(-keyData.lx);
  joyMsg.axes.push_back(keyData.ly);
  joyMsg.axes.push_back(-keyData.rx);
  joyMsg.axes.push_back(keyData.ry);
  joyMsg.buttons.push_back(keyData.btn.components.X);
  joyMsg.buttons.push_back(keyData.btn.components.A);
  joyMsg.buttons.push_back(keyData.btn.components.B);
  joyMsg.buttons.push_back(keyData.btn.components.Y);
  joyMsg.buttons.push_back(keyData.btn.components.L1);
  joyMsg.buttons.push_back(keyData.btn.components.R1);
  joyMsg.buttons.push_back(keyData.btn.components.L2);
  joyMsg.buttons.push_back(keyData.btn.components.R2);
  joyMsg.buttons.push_back(keyData.btn.components.select);
  joyMsg.buttons.push_back(keyData.btn.components.start);
  joyPublisher_.publish(joyMsg);
}

}  // namespace legged