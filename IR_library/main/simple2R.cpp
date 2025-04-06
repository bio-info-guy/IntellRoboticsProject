#include <Planar2R.hpp>
#include <iostream>
int main(int argc, char ** argv) {
	IRlibrary::Planar2R obj(2, 1);
	IRlibrary::Vec2 q = {0, 0};
	obj.setConfig(q);
	q = obj.getXY();
	std::cout << q[0] << " " << q[1] << std::endl;
	return 0;
}

