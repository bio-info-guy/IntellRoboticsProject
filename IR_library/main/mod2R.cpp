#include <Planar2Rmod.hpp>
#include <iostream>
int main(int argc, char ** argv) {
	IRlibrary::Planar2Rmod obj(2, 3);
	IRlibrary::Vec2 q = {3.14, 0.3};
	obj.setConfig(q);
	q = obj.getXY();
	std::cout << q[0] << " " << q[1] << std::endl;
	return 0;
}

