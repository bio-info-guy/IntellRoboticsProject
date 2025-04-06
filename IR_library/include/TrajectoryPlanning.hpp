#ifndef TRAJECTORYPLANNING_HPP
#define TRAJECTORYPLANNING_HPP

#include <vector>
#include <TypeDefs.hpp>
#include <MathUtils.hpp>
#include <RigidBodyMotion.hpp>

namespace IRlibrary
{
  double CubicTimeScaling(double, double );

  double QuinticTimeScaling(double, double);

  Eigen::MatrixXd JointTrajectory(Eigen::VectorXd , Eigen::VectorXd, double, size_t, double(*scalingFn)(double, double));

  std::vector<SE3Mat> ScrewTrajectory(SE3Mat, SE3Mat, double, size_t, double(*scalingFn)(double, double));

  std::vector<SE3Mat>  CartesianTrajectory(SE3Mat , SE3Mat, double, size_t, double(*scalingFn)(double, double));
  

} /* IRlibrary */ 

#endif /* ifndef JACOBIAN_HPP */
