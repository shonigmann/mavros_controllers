//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef DISTURBANCEOBSERVERCTRL_H
#define DISTURBANCEOBSERVERCTRL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "geometric_controller/geometric_controller.h"

using namespace std;
using namespace Eigen;
class StateDependentLqr
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_;

    Eigen::Vector3d pos_error_, vel_error_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd K_;


    void CmdLoopCallback(const ros::TimerEvent& event);
    void StatusLoopCallback(const ros::TimerEvent& event);
    void CalculateOptimalGain();


    geometricCtrl geometric_controller_;

  public:
    StateDependentLqr(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ StateDependentLqr();
};


#endif
