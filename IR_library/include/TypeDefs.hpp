#ifndef TYPEDEFS_HPP
#define TYPEDEFS_HPP

#include <eigen3/Eigen/Dense>
namespace IRlibrary
{
  typedef Eigen::Matrix <double, 2, 1> Vec2;
  typedef Eigen::Matrix <double, 3, 1> Vec3;
  typedef Eigen::Matrix <double, 4, 1> Vec4;
  typedef Eigen::Matrix <double, 5, 1> Vec5;
  typedef Eigen::Matrix <double, 6, 1> Vec6;
  typedef Eigen::Matrix <double, 2, 2> SO2Mat;
  typedef Eigen::Matrix <double, 3, 3> SO3Mat;
  typedef Eigen::Matrix <double, 3, 3> so3Mat;
  typedef Eigen::Matrix <double, 4, 4> SE3Mat;
  typedef Eigen::Matrix <double, 4, 4> se3Mat;
  typedef Eigen::Matrix <double, 6, 1> Twist;
  typedef Eigen::Matrix <double, 6, 1> ScrewAxis;
  typedef Eigen::Matrix <double, 6, 6> AdjMat;
  typedef Eigen::Matrix <double, 6, Eigen::Dynamic> JacobianMat;
  struct AxisAngle {
    Vec3 omega;
    double theta;
  };
} /* IRlibrary */ 
#endif /* ifndef TYPEDEFS_HPP */
