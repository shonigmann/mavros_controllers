#include "geometric_controller/geometric_controller.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(GeometricControllerTest, acc2quaternion) {
  Eigen::Vector3d acceleration;
  double yaw;
  //Expected result;
  Eigen::Vector4d ref_attitude;
  Eigen::Vector4d attitude;

  //Condition
  acceleration << 0.0, 0.0, 1.0;
  yaw = 0.0;
  // Expected outcome
  ref_attitude << 1.0, 0.0, 0.0, 0.0;

  attitude = geometricCtrl::acc2quaternion(acceleration, yaw);  
  
  ASSERT_TRUE(attitude.isApprox(ref_attitude));

  //Condition
  acceleration << 0.0, 0.0, 1.0;
  yaw = 1.5714;
  // Expected outcome
  ref_attitude << std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw);

  attitude = geometricCtrl::acc2quaternion(acceleration, yaw);
  
  ASSERT_TRUE(attitude.isApprox(ref_attitude));
}

TEST(GeometricControllerTest, matrix_hat) {
  Eigen::Vector3d test;
  Eigen::Vector3d result;
  Eigen::Matrix3d matrix;
  test<< 1.0, 2.0, 3.0;

  result = geometricCtrl::matrix_hat_inv(geometricCtrl::matrix_hat(test));

  // ASSERT_TRUE(result.isApprox(test));
  EXPECT_EQ(result(7), test(0));
  EXPECT_EQ(result(2), test(1));
  EXPECT_EQ(result(3), test(2));

}