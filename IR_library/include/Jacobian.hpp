#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include <vector>
#include <TypeDefs.hpp>
#include <MathUtils.hpp>
#include <RigidBodyMotion.hpp>

namespace IRlibrary
{
  /** Computes the space jacobian given the list of joint screws Slist expressed in the space frame and the list of joint values thetalist **/
  JacobianMat JacobianSpace (std::vector <ScrewAxis>, std::vector <double>);

  /** Computes the body jacobian given the list of joint screws Blist expressed in the body frame and the list of joint values thetalist **/
  JacobianMat JacobianBody (std::vector <ScrewAxis>, std::vector <double>);

} /* IRlibrary */ 

#endif /* ifndef JACOBIAN_HPP */
