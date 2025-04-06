#include "Planar2Rmod.hpp"

namespace IRlibrary
{
	void Planar2Rmod::zeroForwardKinematics() {
		//Complete the lines below
	  Vec2 r;
	  r << l[1]*2/3, l[1]/2;
	  xy[0] = base[0]+l[0]*std::cos(q[0])+r[0]*std::cos(q[0]+q[1])-r[1]*std::sin(q[0]+q[1]);
	  xy[1] = base[1]+l[1]*std::sin(q[0])+r[0]*std::sin(q[0]+q[1])+r[1]*std::cos(q[0]+q[1]);
	}
	
} /* IRlibrary */ 


