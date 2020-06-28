//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "geometric_controller/geometric_controller.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  geometricCtrl *geometricController = new geometricCtrl(nh, nh_private);

  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig> srv;
  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig>::CallbackType f;
  f = boost::bind(&geometricCtrl::dynamicReconfigureCallback, geometricController, _1, _2);
  srv.setCallback(f);

  ros::spin();
  return 0;
}

Eigen::Vector4d geometricCtrl::jerkcontroller(Eigen::Vector3d &ref_jerk, Eigen::Vector3d &ref_acc, Eigen::Vector3d &ref_vel, Eigen::Vector4d &curr_att){
  //Feedforward control from Lopez(2016)
  Eigen::Vector4d ratecmd;
  Eigen::Vector3d jerk, jerk_fb, acc_fb, jerk_vector, ratecmd_pre;
  Eigen::Matrix3d R;

  //TODO: calculate jerk_fb from acc_reference
  // jerk_fb = calc(ref_acc, ref_vel, ref_pos);
  jerk = ref_jerk + jerk_fb;
  jerk_vector = jerk / jerk.norm() - ref_acc*ref_acc.dot(jerk) / std::pow(jerk.norm(), 3); //TODO: is ref_acc ?

  R = quat2RotMatrix(curr_att);
  ratecmd_pre = R.transpose() * jerk_vector;

  ratecmd(0) =  (-1.0)* ratecmd_pre(2);
  ratecmd(2) = ratecmd_pre(1);
  ratecmd(3) = 0.0;

  return ratecmd;
}