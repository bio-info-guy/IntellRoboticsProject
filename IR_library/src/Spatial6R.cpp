#include "Spatial6R.hpp"
#include <iostream>
namespace IRlibrary
{
  void Spatial6R::zeroForwardKinematics() {
    std::vector<double> q_list;
    q_list.push_back(q[0]); q_list.push_back(q[1]); q_list.push_back(q[2]); q_list.push_back(q[3]); q_list.push_back(q[4]); q_list.push_back(q[5]); 
    SE3Mat T_E = FKinSpace(M, Slist, q_list);
    X = T_E;
    axisAngle = AxisAng3(so3ToVec(MatrixLog3(T_E.block<3,3>(0, 0))));
  }

  bool Spatial6R::inverseKinematics_numerical(Eigen::VectorXd const &initial_thetaList, SE3Mat const &X_i){
    Eigen::VectorXd thetaList;

    if(IKinSpace(Slist, M, X_i, initial_thetaList, thetaList)) {
      std::cout << "Spatial6R::inverseKinematics_numerical [ERROR] failed to converge or at singularity\n";
      return 1;
    }
    X = X_i;
    q << thetaList;
    return 0;
  }

  bool Spatial6R::inverseKinematics_analytical(SE3Mat const &X_i){
    if(!checkInDexWs(X_i)){return 1;}
    Vec3 p_j, p_i;
    double q1, q2, q3, q4, q5, q6;
    double r, h, D;
    p_j << X_i(0,3), X_i(1,3), X_i(2,3);
    if (p_j[1] == 0 and p_j[0] == 0){q1 = 0.0;}
    else{q1 = std::atan2(p_j[1], p_j[0]);}
    r = std::sqrt(p_j[0]*p_j[0]+p_j[1]*p_j[1]);
    h = p_j[2]-l[0];
    D= (r*r + h*h - l[1]*l[1] -l[2]*l[2])/(2*l[1]*l[2]);
    q3= std::atan2(std::sqrt(1-D*D), D);
    q2 = std::atan2(h, r)-std::atan2(l[2]*std::sin(q3),l[1]+l[2]*std::cos(q3));
    SE3Mat T = MatrixExp6(VecTose3(-Slist[2]*q3))*MatrixExp6(VecTose3(-Slist[1]*q2))*MatrixExp6(VecTose3(-Slist[0]*q1))*X_i*TransInv(M);
    SO3Mat R;
    TransToRp(T, R, p_i);
    if (R(2,0) != 1 and R(2,0) != -1){
      q4 = std::atan2(R(1,0), R(0,0));
      q5 = std::atan2(-R(2,0), std::sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));
      q6 = std::atan2(R(2,1), R(2,2));
    }else if (R(2,0) == 1){
      q4 = 0;
      q5 = M_PI/2;
      q6 = std::atan2(R(0,1), R(1,1));
    }else{
      q4 = 0;
      q5 = -M_PI/2;
      q6 = -std::atan2(R(0,1), R(1,1));
    }
    X = X_i;
    q << q1, q2, q3, q4, q5, q6;
    return 0;
  }
  
  bool Spatial6R::checkInDexWs(SE3Mat const &X_i){
    Vec3 s_center;
    Vec3 p;
    s_center << base[0], base[1], base[2]+l[0];
    p << X_i(0,3), X_i(1,3), X_i(2,3);
    if((s_center - p).norm() > wsRad[1] or (s_center - p).norm() < wsRad[0]){return 0;}else{return 1;}
  };
	
} /* IRlibrary */ 


