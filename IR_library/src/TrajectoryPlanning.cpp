#include <TrajectoryPlanning.hpp>

namespace IRlibrary
{
  double CubicTimeScaling(double Tf, double t){
    double s = (3*t*t)/(Tf*Tf)-(2*t*t*t)/(Tf*Tf*Tf);
    return s;
  };

  double QuinticTimeScaling(double Tf, double t){
    double s = (10/(Tf*Tf*Tf))*t*t*t - (15/(Tf*Tf*Tf*Tf))*(t*t*t*t) +(6/(Tf*Tf*Tf*Tf*Tf))*(t*t*t*t*t);
    return s;
  };

  Eigen::MatrixXd JointTrajectory(Eigen::VectorXd thetStart, Eigen::VectorXd thetEnd, double Tf, size_t N, double(*scalingFn)(double, double)){
    Eigen::MatrixXd thetaPath(N, thetStart.size());
    for (int i = 0; i < N; ++i){
      thetaPath.row(i) = thetStart+scalingFn(Tf, i*(Tf/(N-1)))*(thetEnd - thetStart);
    }
    return thetaPath;
  };

  std::vector<SE3Mat> CartesianTrajectory(SE3Mat Xstart, SE3Mat Xend, double Tf, size_t N, double(*scalingFn)(double, double)){
    std::vector<SE3Mat> SE3Matlist;
    SO3Mat Rs, Re;
    so3Mat Rtwist;
    Vec3 ps, pe, pdiff;
    TransToRp(Xstart, Rs, ps);
    TransToRp(Xend, Re, pe);
    Rtwist = MatrixLog3(Rs.transpose()*Re);
    pdiff = pe-ps;
    for (int i = 0; i < N; ++i){
      SO3Mat R_s = Rs*MatrixExp3(Rtwist*scalingFn(Tf, i*(Tf/(N-1))));
      Vec3 p_s = ps+scalingFn(Tf, i*(Tf/(N-1)))*pdiff;
      SE3Matlist.push_back(RpToTrans(R_s, p_s));
    }
    return SE3Matlist;
  };

  std::vector<SE3Mat>  ScrewTrajectory(SE3Mat Xstart, SE3Mat Xend, double Tf, size_t N, double(*scalingFn)(double, double)){
    std::vector<SE3Mat> SE3Matlist;
    se3Mat X_twist = MatrixLog6(TransInv(Xstart)*Xend);
    
    for(int i = 0; i < N; ++i){
      SE3Mat X_s = Xstart*MatrixExp6(scalingFn(Tf, i*(Tf/(N-1)))*(X_twist));
      SE3Matlist.push_back(X_s);
    }
    return SE3Matlist;
  };
  

} /* IRlibrary */ 

