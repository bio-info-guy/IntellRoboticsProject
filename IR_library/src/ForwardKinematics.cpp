#include <ForwardKinematics.hpp>
#include <iostream>
namespace IRlibrary
{
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist **/
	SE3Mat FKinBody (SE3Mat M, std::vector <ScrewAxis> Blist, std::vector <double> thetaList){
		SE3Mat T_eef = M;
		for (int i = 0; i < thetaList.size(); ++i)
			T_eef = T_eef * MatrixExp6(VecTose3(Blist[i]*thetaList[i]));
		return T_eef;
	}

	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	SE3Mat FKinSpace (SE3Mat M, std::vector <ScrewAxis> Slist, std::vector <double> thetaList){
		SE3Mat T_eef = M;
		for (int i = (thetaList.size() - 1); i >= 0; --i)
		  {
		    T_eef = MatrixExp6(VecTose3(Slist[i]*thetaList[i])) * T_eef;
		  }
		return T_eef;
	}

} /* IRlibrary */ 
