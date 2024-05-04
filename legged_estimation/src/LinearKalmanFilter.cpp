//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/LinearKalmanFilter.h"

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <visualization_msgs/Marker.h>

namespace legged {

KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics), tfListener_(tfBuffer_), topicUpdated_(false) {
  xHat_.setZero();
  ps_.setZero();
  vs_.setZero();
  a_.setZero();
  a_.block(0, 0, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(3, 3, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(6, 6, 12, 12) = Eigen::Matrix<scalar_t, 12, 12>::Identity();
  b_.setZero();

  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
  c1 << Eigen::Matrix<scalar_t, 3, 3>::Identity(), Eigen::Matrix<scalar_t, 3, 3>::Zero();
  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
  c2 << Eigen::Matrix<scalar_t, 3, 3>::Zero(), Eigen::Matrix<scalar_t, 3, 3>::Identity();
  c_.setZero();
  c_.block(0, 0, 3, 6) = c1;
  c_.block(3, 0, 3, 6) = c1;
  c_.block(6, 0, 3, 6) = c1;
  c_.block(9, 0, 3, 6) = c1;
  c_.block(0, 6, 12, 12) = -Eigen::Matrix<scalar_t, 12, 12>::Identity();
  c_.block(12, 0, 3, 6) = c2;
  c_.block(15, 0, 3, 6) = c2;
  c_.block(18, 0, 3, 6) = c2;
  c_.block(21, 0, 3, 6) = c2;
  c_(27, 17) = 1.0;
  c_(26, 14) = 1.0;
  c_(25, 11) = 1.0;
  c_(24, 8) = 1.0;
  p_.setIdentity();
  p_ = 100. * p_;
  q_.setIdentity();
  r_.setIdentity();
  feetHeights_.setZero(4);
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  //ground estimator
  Wpla.resize(4, 3);
  Wpla.setZero();
  Wpla(0, 0) = 1;
  Wpla(1, 0) = 1;
  Wpla(2, 0) = 1;
  Wpla(3, 0) = 1;
  apla.setZero();
  zfeet.setZero();
  rpy_des.setZero();

  ros::NodeHandle nh;
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  world2odom_.setRotation(tf2::Quaternion::getIdentity());
  sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);
  ground_est = nh.advertise<std_msgs::Float64MultiArray>("/ground_est", 1);
}

vector_t KalmanFilterEstimate::update(const ros::Time& time, const ros::Duration& period) {
  scalar_t dt = period.toSec();
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(6, 6, 12, 12) = dt * Eigen::Matrix<scalar_t, 12, 12>::Identity();

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  size_t actuatedDofNum = info_.actuatedDofNum;

  vector_t qPino(info_.generalizedCoordinatesNum);
  vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
  qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3),
      rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);

  const auto eePos = eeKinematics_->getPosition(vector_t());
  const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

  Eigen::Matrix<scalar_t, 18, 18> q = Eigen::Matrix<scalar_t, 18, 18>::Identity();
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
  q.block(6, 6, 12, 12) = q_.block(6, 6, 12, 12) * footProcessNoisePosition_;

  Eigen::Matrix<scalar_t, 28, 28> r = Eigen::Matrix<scalar_t, 28, 28>::Identity();
  r.block(0, 0, 12, 12) = r_.block(0, 0, 12, 12) * footSensorNoisePosition_;
  r.block(12, 12, 12, 12) = r_.block(12, 12, 12, 12) * footSensorNoiseVelocity_;
  r.block(24, 24, 4, 4) = r_.block(24, 24, 4, 4) * footHeightSensorNoise_;


  //   Eigen::Vector3f temp_norm_vec;
  // float d = 0;
  // if(iterationCounter > 40)
  // {
  //     Eigen::RowVector3f centroid = Foot_Contact_Point.colwise().mean();
  //     Eigen::MatrixXf demean = Foot_Contact_Point;
  //     demean.rowwise() -= centroid;
  //     Eigen::JacobiSVD<Eigen::MatrixXf> svd(demean, Eigen::ComputeThinU | Eigen::ComputeThinV);
  //     Eigen::Matrix3f V = svd.matrixV();

  //     //Eigen::MatrixXf U = svd.matrixU();
  //     //Eigen::Matrix3f S = U.inverse() * demean * V.transpose().inverse();
  //     temp_norm_vec << V(0,2), V(1,2), V(2,2);
  //     norm_vec = 0.8*norm_vec + 0.2 * temp_norm_vec;
  //     d = -norm_vec.transpose() * centroid.transpose();
  //     //std::cout << "norm vector: " << norm_vec.transpose() << "\n";
  //     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic > R_z;
  //     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic > Pin_Rz;
  //     R_z.resize(3,3);
  //     Pin_Rz.resize(3,3);
  //     R_z << cosf(_yaw_des), sinf(_yaw_des), 0, sinf(_yaw_des), -cosf(_yaw_des), 0, 0, 0, 1;
  //     pseudoInverse(R_z, 0.001, Pin_Rz);
  //     Vec3<float> vec_compute_rp = Pin_Rz * norm_vec;
  //     temp_roll_des = asinf(vec_compute_rp(1));
  //     temp_pitch_des = atanf(vec_compute_rp(0) / vec_compute_rp(2));
  //     temp_roll_des = constrain_angle(temp_roll_des);
  //     temp_pitch_des = constrain_angle(temp_pitch_des);
  //     _roll_des = temp_roll_des;
  //     _pitch_des = temp_pitch_des;
  // }
  // rpy_des << temp_roll_des, temp_pitch_des, rbdState_(0);

  for (int foot = 0; foot < 4; foot++)
  {
    if (contactFlag_[foot] == 1) 
    {
      Wpla(foot, 1) = eePos[foot](0) + rbdState_(3);     
      Wpla(foot, 2) = eePos[foot](1) + rbdState_(4);      
      zfeet(foot) = eePos[foot](2) + rbdState_(5);        
    }
  }
  // Wplainv=Wpla.completeOrthogonalDecomposition().pseudoInverse();
  pseudoInverse(Wpla, 0.001, Wplainv);

  apla = (1.0f - 0.5f) * apla + 0.5f * Wplainv * zfeet;
  Eigen::Matrix<float, 3, 1> norZ(-apla(1), -apla(2), 1);  
  norZ.normalize();

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> leftA, pinvA;
  leftA.resize(3, 3);
  pinvA.resize(3, 3);
  leftA << cosf(rbdState_(0)), sinf(rbdState_(0)), 0, sinf(rbdState_(0)), -cosf(rbdState_(0)), 0, 0, 0, 1;
  // pinvA=leftA.completeOrthogonalDecomposition().pseudoInverse();
  pseudoInverse(leftA, 0.001, pinvA);
  Eigen::Matrix<float, 3, 1> x = pinvA*norZ;

  float rollDes = asinf(x(1));
  float pitchDes = atan2f(x(0), x(2));
  
  // if     ( fabs(rollDes/3.14*180) <  5.0 ) rollDes  = 0;
  // else if( fabs(rollDes/3.14*180) > 15.0 ) rollDes  = rollDes;
  // else
  // {
  //   if(rollDes > 0) rollDes = (rollDes - 5.0/180.0*3.14)*1.5;
  //   if(rollDes < 0) rollDes = (rollDes + 5.0/180.0*3.14)*1.5;
  // }
  // if     ( fabs(pitchDes/3.14*180) <  5.0 ) rollDes  = 0;
  // else if( fabs(pitchDes/3.14*180) > 15.0 ) rollDes  = rollDes;
  // else
  // {
  //   if(pitchDes > 0) pitchDes = (pitchDes - 5.0/180.0*3.14)*1.5;
  //   if(pitchDes < 0) pitchDes = (pitchDes + 5.0/180.0*3.14)*1.5;
  // }

  rpy_des << rollDes, pitchDes, rbdState_(0);
  // std::cout<<""<<std::endl;
  // std::cout<<rpy_des<<std::endl;
  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;

    int qIndex = 6 + i1;
    int rIndex1 = i1;
    int rIndex2 = 12 + i1;
    int rIndex3 = 24 + i;
    bool isContact = contactFlag_[i];

    scalar_t high_suspect_number(100);
    q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
    r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
    r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
    r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

    ps_.segment(3 * i, 3) = -eePos[i];
    ps_.segment(3 * i, 3)[2] += footRadius_;
    vs_.segment(3 * i, 3) = -eeVel[i];
  }

  vector3_t g(0, 0, -9.81);
  vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;

  Eigen::Matrix<scalar_t, 28, 1> y;
  y << ps_, vs_, feetHeights_;
  xHat_ = a_ * xHat_ + b_ * accel;
  Eigen::Matrix<scalar_t, 18, 18> at = a_.transpose();
  Eigen::Matrix<scalar_t, 18, 18> pm = a_ * p_ * at + q;
  Eigen::Matrix<scalar_t, 18, 28> cT = c_.transpose();
  Eigen::Matrix<scalar_t, 28, 1> yModel = c_ * xHat_;
  Eigen::Matrix<scalar_t, 28, 1> ey = y - yModel;
  Eigen::Matrix<scalar_t, 28, 28> s = c_ * pm * cT + r;

  Eigen::Matrix<scalar_t, 28, 1> sEy = s.lu().solve(ey);
  xHat_ += pm * cT * sEy;

  Eigen::Matrix<scalar_t, 28, 18> sC = s.lu().solve(c_);
  p_ = (Eigen::Matrix<scalar_t, 18, 18>::Identity() - pm * cT * sC) * pm;

  Eigen::Matrix<scalar_t, 18, 18> pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
    p_.block(0, 2, 2, 16).setZero();
    p_.block(2, 0, 16, 2).setZero();
    p_.block(0, 0, 2, 2) /= 10.;
  }

  if (topicUpdated_) {
    updateFromTopic();
    topicUpdated_ = false;
  }

  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

  auto odom = getOdomMsg();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);
  
  msg.data.clear();
  msg.data.push_back(rpy_des(0));
  msg.data.push_back(rpy_des(1));
  msg.data.push_back(rpy_des(2));
  msg.data.push_back(apla(0));
  msg.data.push_back(apla(1));
  msg.data.push_back(apla(2));
  ground_est.publish(msg);
  
  // vector_t p1_foot = vector_t::Zero(3);
  // vector_t p2_foot = vector_t::Zero(3);
  // vector_t p3_foot = vector_t::Zero(3);
  // vector_t p4_foot = vector_t::Zero(3);
  // vector_t p1_foot_real = vector_t::Zero(3);
  // vector_t p2_foot_real = vector_t::Zero(3);
  // vector_t p3_foot_real = vector_t::Zero(3);
  // vector_t p4_foot_real = vector_t::Zero(3);
  // // rbdState_.segment<3>(3);
  // p1_foot = eePos[0]+rbdState_.segment<3>(3);
  // p2_foot = eePos[1]+rbdState_.segment<3>(3);
  // p3_foot = eePos[2]+rbdState_.segment<3>(3);
  // p4_foot = eePos[3]+rbdState_.segment<3>(3);
  // p1_foot_real = p1_foot;
  // p2_foot_real = p2_foot;
  // p3_foot_real = p3_foot;
  // p4_foot_real = p4_foot;
  // p1_foot_real(2) = apla(0)+apla(1)*p1_foot(0)+apla(2)*p1_foot(1); 
  // p2_foot_real(2) = apla(0)+apla(1)*p2_foot(0)+apla(2)*p2_foot(1); 
  // p3_foot_real(2) = apla(0)+apla(1)*p3_foot(0)+apla(2)*p3_foot(1); 
  // p4_foot_real(2) = apla(0)+apla(1)*p4_foot(0)+apla(2)*p4_foot(1); 

  // float f = 0.0;
  // visualization_msgs::Marker points, line_strip, line_list;
  //   points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "odom";
  //   points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  //   points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  //   points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  //   points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  //   points.id = 0;
  //   line_strip.id = 1;
  //   line_list.id = 2;

  //   points.type = visualization_msgs::Marker::POINTS;
  //   line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  //   line_list.type = visualization_msgs::Marker::LINE_LIST;

  //   // POINTS markers use x and y scale for width/height respectively
  //   points.scale.x = 0.05;
  //   points.scale.y = 0.05;

  //   // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  //   line_strip.scale.x = 0.05;
  //   line_list.scale.x = 0.02;

  //   // Points are green
  //   points.color.g = 0.5f;
  //   points.color.a = 0.5f;

  //   // Line strip is blue
  //   line_strip.color.b = 1.0;
  //   line_strip.color.a = 1.0;

  //   // Line list is red
  //   line_list.color.r = 1.0;
  //   line_list.color.a = 1.0;

  //   // Create the vertices for the points and lines
  //     geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
  //     p1.x = p1_foot(0);
  //     p1.y = p1_foot(1);
  //     p1.z = p1_foot(2);

  //     p2.x = p2_foot(0);
  //     p2.y = p2_foot(1);
  //     p2.z = p2_foot(2);

  //     p3.x = p3_foot(0);
  //     p3.y = p3_foot(1);
  //     p3.z = p3_foot(2);

  //     p4.x = p4_foot(0);
  //     p4.y = p4_foot(1);
  //     p4.z = p4_foot(2);
  //     // p1_foot_real

  //     p5.x = p1_foot_real(0);
  //     p5.y = p1_foot_real(1);
  //     p5.z = p1_foot_real(2);

  //     p6.x = p2_foot_real(0);
  //     p6.y = p2_foot_real(1);
  //     p6.z = p2_foot_real(2);

  //     p7.x = p3_foot_real(0);
  //     p7.y = p3_foot_real(1);
  //     p7.z = p3_foot_real(2);

  //     p8.x = p4_foot_real(0);
  //     p8.y = p4_foot_real(1);
  //     p8.z = p4_foot_real(2);

  //     // points.points.push_back(p1);
  //     // points.points.push_back(p2);
  //     // points.points.push_back(p3);
  //     // points.points.push_back(p4);
  //     points.points.push_back(p5);
  //     points.points.push_back(p6);
  //     points.points.push_back(p7);
  //     points.points.push_back(p8);
  //     marker_pub.publish(points);

  return rbdState_;
}

void KalmanFilterEstimate::updateFromTopic() {
  auto* msg = buffer_.readFromRT();

  tf2::Transform world2sensor;
  world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                           msg->pose.pose.orientation.w));

  if (world2odom_.getRotation() == tf2::Quaternion::getIdentity())  // First received
  {
    tf2::Transform odom2sensor;
    try {
      geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("odom", msg->child_frame_id, msg->header.stamp);
      tf2::fromMsg(tf_msg.transform, odom2sensor);
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    world2odom_ = world2sensor * odom2sensor.inverse();
  }
  tf2::Transform base2sensor;
  try {
    geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("base", msg->child_frame_id, msg->header.stamp);
    tf2::fromMsg(tf_msg.transform, base2sensor);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();
  vector3_t newPos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), odom2base.getOrigin().z());

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  vector_t qPino(info_.generalizedCoordinatesNum);
  qPino.head<3>() = newPos;
  qPino.segment<3>(3) = rbdState_.head<3>();
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  pinocchio::forwardKinematics(model, data, qPino);
  pinocchio::updateFramePlacements(model, data);

  xHat_.segment<3>(0) = newPos;
  for (size_t i = 0; i < 4; ++i) {
    xHat_.segment<3>(6 + i * 3) = eeKinematics_->getPosition(vector_t())[i];
    xHat_(6 + i * 3 + 2) -= footRadius_;
    if (contactFlag_[i]) {
      feetHeights_[i] = xHat_(6 + i * 3 + 2);
    }
  }

  auto odom = getOdomMsg();
  odom.header = msg->header;
  odom.child_frame_id = "base";
  publishMsgs(odom);
}

void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
  topicUpdated_ = true;
}

nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg() {
  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
  odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
  odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
  odom.pose.pose.orientation.x = quat_.x();
  odom.pose.pose.orientation.y = quat_.y();
  odom.pose.pose.orientation.z = quat_.z();
  odom.pose.pose.orientation.w = quat_.w();
  odom.pose.pose.orientation.x = quat_.x();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.pose.covariance[i * 6 + j] = p_(i, j);
      odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
    }
  }
  //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
  vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
  odom.twist.twist.linear.x = twist.x();
  odom.twist.twist.linear.y = twist.y();
  odom.twist.twist.linear.z = twist.z();
  odom.twist.twist.angular.x = angularVelLocal_.x();
  odom.twist.twist.angular.y = angularVelLocal_.y();
  odom.twist.twist.angular.z = angularVelLocal_.z();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
      odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
    }
  }
  return odom;
}

void KalmanFilterEstimate::loadSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "kalmanFilter.";
  if (verbose) {
    std::cerr << "\n #### Kalman Filter Noise:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
}

}  // namespace legged
