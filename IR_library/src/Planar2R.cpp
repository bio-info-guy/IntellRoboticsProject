#include "Planar2R.hpp"

namespace IRlibrary
{
	void Planar2R::zeroForwardKinematics() {
		/* Fix this code */
	  xy[0] = base[0] + l[0]*std::cos(q[1]) + l[1]*std::cos(q[1]+q[0]);
	  xy[1] = base[1] + l[0]*std::sin(q[1]) + l[1]*std::sin(q[1]+q[0]);
	}
	
} /* IRlibrary */ 


