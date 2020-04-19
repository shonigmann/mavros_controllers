//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "lqr_controller/lqr_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
StateDependentLqr::StateDependentLqr(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  geometric_controller_(nh, nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &StateDependentLqr::CmdLoopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &StateDependentLqr::StatusLoopCallback, this); // Define timer for constant loop rate



}
StateDependentLqr::~StateDependentLqr() {
  //Destructor
}

void StateDependentLqr::CmdLoopCallback(const ros::TimerEvent& event){
  /// State Dependent Linear Quadratic Regulator

  geometric_controller_.getErrors(pos_error_, vel_error_);
  Eigen::Vector4d w_des;

  // ParseStates();
  // LinearizeDynamics(x);

  CalculateOptimalGain();

  // w_des = K * (x - x_0) + u_0;
  
  geometric_controller_.setFeedthrough(true);
  geometric_controller_.setBodyRateCommand(w_des);

}

void StateDependentLqr::StatusLoopCallback(const ros::TimerEvent& event){

}

void StateDependentLqr::CalculateOptimalGain(){
  K_ = - R_.inverse() * B_.transpose() * P_;

}

void StateDependentLqr::ParseStates(){

}