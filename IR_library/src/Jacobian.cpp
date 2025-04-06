#include <Jacobian.hpp>
#include <iostream>
namespace IRlibrary
{
  /** Computes the space jacobian given the list of joint screws Slist expressed in the space frame and the list of joint values thetalist **/
  JacobianMat JacobianSpace (std::vector <ScrewAxis> Slist, std::vector <double> thetaList){
    SE3Mat T_temp = SE3Mat::Identity();
    AdjMat A;
    JacobianMat J(6, Slist.size());
    J.col(0) = Slist[0];
    for (int i = 0; i <= thetaList.size()-2; ++i)
    {
      T_temp = T_temp * MatrixExp6(VecTose3(Slist[i]*thetaList[i]));
      A = Adjoint(T_temp);
      //      if (i == 2){std::cout << A << std::endl;}
      J.col(i+1) = A*Slist[i+1];
    }  
    return J;
  }

  /** Computes the body jacobian given the list of joint screws Blist expressed in the body frame and the list of joint values thetalist **/
  JacobianMat JacobianBody (std::vector <ScrewAxis> Blist, std::vector <double> thetaList){
    SE3Mat T_temp = SE3Mat::Identity();
    AdjMat A;
    JacobianMat J(6, Blist.size());
    J.col(Blist.size()-1) = Blist[Blist.size()-1];
    for (int i = (thetaList.size() - 1); i >= 1; --i)
    {
      T_temp = T_temp*MatrixExp6(VecTose3(-Blist[i]*thetaList[i]));
      A = Adjoint(T_temp);
      J.col(i-1) = A*Blist[i-1];
    }
    return J;
  }

} /* IRlibrary */ 
