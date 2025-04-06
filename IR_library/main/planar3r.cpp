#include <Planar3R.hpp>
#include <iostream>
int main(int argc, char ** argv) {
	IRlibrary::Planar3R obj(3, 2, 2.5);
	IRlibrary::Vec3 q = {M_PI/3., M_PI/6., M_PI/4. };
	std::vector<double> wsRad = obj.getwsRad();
	std::cout << wsRad[0] << "\t"  << wsRad[1] << "\t" << wsRad[2] << "\t" << wsRad[3] << std::endl;
	obj.setConfig(q);
	auto xy1 = obj.getX();
	IRlibrary::Vec3 initial_thetaList = q;
	initial_thetaList[0] -= M_PI/180.;
	initial_thetaList[1] -= M_PI/180.;
	initial_thetaList[2] -= M_PI/180.;
	obj.inverseKinematics_numerical(initial_thetaList, xy1);
	auto q1 = obj.getConfig();
	std::cout << q1[0] << " " << q1[1]  << " " <<q1[2]<< std::endl;
	obj.setX(xy1);
	auto q2 = obj.getConfig();
	std::cout << q2[0] << " " << q2[1]  << " " <<q2[2]<< std::endl;
	return 0;
}

