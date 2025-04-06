#include <cmath>
#include <iostream>
#include "TypeDefs.hpp"
#include "RigidBodyMotion.hpp"
#include "ForwardKinematics.hpp"
#include "MathUtils.hpp"

using namespace IRlibrary;
int main(int argc, char **argv)
{
	double theta = M_PI/3;
	std::cout << wrapTo2PI(theta) << std::endl;
	/* SO3Mat rotMat; */
	/* rotMat << cos(theta) , -sin(theta), 0, */
	/* 			 sin(theta), cos(theta), 0, */
	/* 			 0, 0, 1; */
	/* std::cout << rotMat << std::endl; */
	/* std::cout << RotInv(rotMat) << std::endl; */
	/* std::cout << rotMat.inverse() << std::endl; */
	/* std::cout << rotMat.transpose() << std::endl; */
	return 0;
}
