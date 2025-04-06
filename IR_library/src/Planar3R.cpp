#include "Planar3R.hpp"

namespace IRlibrary
{
  void Planar3R::zeroForwardKinematics() {
    x[0] = base[0] + l[0]*std::cos(q[0])+l[1]*std::cos(q[0]+q[1])+l[2]*std::cos(q[0]+q[1]+q[2]);
    x[1] = base[1] + l[0]*std::sin(q[0])+l[1]*std::sin(q[0]+q[1])+l[2]*std::sin(q[0]+q[1]+q[2]);
    x[2] = q[0] + q[1] + q[2];
  }

  bool Planar3R::inverseKinematics(Vec3 const &x_in){
    Eigen::VectorXd thetaList;
    Vec2 b;
    double pDist = (x_in.head(2)-base).norm();
    if(pDist > wsRad[3] or pDist < wsRad[0]){
      std::cout << "planar3R::inverseKinematics [ERROR] end effector not reachable" << std::endl;
      return 1;
    }
    x = x_in;
    b << x_in[0]-l[2]*std::cos(x_in[2]), x_in[1]-l[2]*std::sin(x_in[2]);
    if ((b-base).norm() > l[1]+l[2] or (b-base).norm() < std::fabs(l[1]-l[2])){
      std::cout << "planar3R::inverseKinematics [ERROR] end effector orientation not possible" << std::endl;
      return 1;
    }
    double beta; double alpha; double phi;
    beta = std::acos((l[0]*l[0]+l[1]*l[1]-b[0]*b[0]-b[1]*b[1])/(2*l[0]*l[1]));
    alpha = std::acos((l[0]*l[0]+b[0]*b[0]+b[1]*b[1]-l[1]*l[1])/(2*l[0]*std::sqrt(b[0]*b[0]+b[1]*b[1])));
    phi = std::atan2(b[1], b[0]);
    q << phi-alpha, M_PI-beta, x_in[2]-(M_PI-beta)-(phi-alpha);
    return 0;
    
  }
  
  bool Planar3R::inverseKinematics_numerical(Eigen::VectorXd const &initial_thetaList, Vec3 const &x_in) {
    SE3Mat T;
    double ctht = cos(x_in[2]); double stht = sin(x_in[2]);
    T << ctht, -stht, 0, x_in[0],
      stht, ctht, 0, x_in[1],
      0, 0, 1, 0,
      0, 0, 0, 1;

    Eigen::VectorXd thetaList;
    if(IKinSpace(Slist, M, T, initial_thetaList, thetaList)) {
      std::cout << "Planar3R::inverseKinematics_numerical [ERROR] failed to converge or at singularity\n";
      return 1;
    }
    x = x_in;
    q << thetaList;
    return 0;
  }
  bool Planar3R::checkInDexWs(Vec3 x_in) {
    double pDist = (x_in.head(2) - base).norm();    
    if(pDist > wsRad[2] or pDist < wsRad[1] ){
      return false;
    }else{
      return true;
    }
  }
  
} /* IRlibrary */ 


