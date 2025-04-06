#ifndef FORWARDKINEMATICS_HPP
#define FORWARDKINEMATICS_HPP

#include <vector>
#include <TypeDefs.hpp>
#include <MathUtils.hpp>
#include <RigidBodyMotion.hpp>

namespace IRlibrary
{
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist **/
	SE3Mat FKinBody (SE3Mat, std::vector <ScrewAxis>, std::vector <double>);

	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	SE3Mat FKinSpace (SE3Mat, std::vector <ScrewAxis>, std::vector <double>);

} /* IRlibrary */ 

#endif /* ifndef FORWARDKINEMATICS_HPP */
