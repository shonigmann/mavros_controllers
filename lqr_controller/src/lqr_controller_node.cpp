//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "lqr_controller/lqr_controller.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"dob_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  StateDependentLqr lqrcontroller(nh, nh_private);
  ros::spin();
  return 0;
}
