#include "Spatial3R.hpp"

namespace IRlibrary
{
	void Spatial3R::zeroForwardKinematics() {
	  Vec3 p;
	  SE3Mat M;
	  M << 1.0, 0.0,  0.0, l[1]+l[2],
	       0.0, 0.0, -1.0,       0.0,
	       0.0, 1.0,  0.0,      l[0],
	       0.0, 0.0,  0.0,       1.0;

	  Vec3 w1, w2, w3, q1, q2, q3;
	  ScrewAxis s1, s2, s3;
	  w1 << 0.0, 0.0, 1.0;
	  q1 << 0.0, 0.0, l[0];
	  s1 << w1, -w1.cross(q1);
	  
	  w2 << 0.0, -1.0, 0.0;
	  q2 << 0.0, 0.0, l[0];
	  s2 << w2, -w2.cross(q2);
	  
	  w3 << 0.0, -1.0, 0.0;
	  q3 << l[1], 0.0, l[0]; 
	  s3 << w3, -w3.cross(q3);
	  
	  std::vector<ScrewAxis> s;
	  s.push_back(s1);
	  s.push_back(s2);
	  s.push_back(s3);
	  
	  std::vector<double> thetas{q[0], q[1], q[2]};
	  SE3Mat T_E = FKinSpace(M, s, thetas);
	  p[0] = T_E(0, 3);
	  p[1] = T_E(1, 3);
	  p[2] = T_E(2, 3);
	  x = base + p;
	  axisAngle = AxisAng3(so3ToVec(MatrixLog3(T_E.block<3,3>(0, 0))));
	}
	
} /* IRlibrary */ 


