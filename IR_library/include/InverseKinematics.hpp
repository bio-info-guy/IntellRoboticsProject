#ifndef INVERSEKINEMATICS_HPP
#define INVERSEKINEMATICS_HPP

#include <vector>
#include <cmath>
#include <eigen3/Eigen/QR>
#include <TypeDefs.hpp>
#include <Jacobian.hpp>
#include <RigidBodyMotion.hpp>
#include <ForwardKinematics.hpp>
#include <iostream>
#include <eigen3/Eigen/Eigenvalues>
namespace IRlibrary {

/** This function uses iterative Newton–Raphson to calculate the inverse
 * kinematics given the list of joint screws B_i expressed in the
 * end-effector frame, the end-effector home configuration M,
 * the desired end-effector configuration T,
 * an initial guess at the joint angles init_thetaList,
 * and the tolerances eps_omega and eps_v on the final error.
 * If a solution is not found within a maximum number of iterations, then success is false.
 */
	bool IKinBody(std::vector <ScrewAxis> const &, SE3Mat const &, SE3Mat const &, Eigen::VectorXd const &, Eigen::VectorXd &, double eps_omega = 1e-10, double eps_v = 1e-10, size_t maxIter = 100);

/** This is similar to IKinBody, except that the joint screws S_i are expressed in the
 * space frame and the tolerances are interpreted in the space frame, also.
 */
	bool IKinSpace(std::vector <ScrewAxis> const &, SE3Mat const &, SE3Mat const &, Eigen::VectorXd const &, Eigen::VectorXd &, double eps_omega = 1e-8, double eps_v = 1e-8, size_t maxIter = 100);


} /* IRlibrary */
#endif /* ifndef INVERSEKINEMATICS_HPP */
