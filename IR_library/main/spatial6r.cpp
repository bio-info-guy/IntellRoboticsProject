#include <Spatial6R.hpp>
#include <iostream>
#include <cmath>
#include <TrajectoryPlanning.hpp>
#include <Planar3R.hpp>
#include <MathUtils.hpp>

using namespace IRlibrary;
int main(int argc, char **argv){
  Spatial3R obj3r(0.1, 2.5, 2.5);
  Spatial6R obj6r(0.1, 2.5, 2.5);
  Vec6 q;
  Vec3 q_3;
  q_3 << 0.81, -0.46, 1.08;
  q << 2.81, -0.46, 3.13, M_PI/3, 2, 0.9;
  std::cout << q << std::endl << std::endl;;
  obj6r.setConfig(q);
  obj3r.setConfig(q_3);
  std::cout << obj3r.getX() << std::endl << std::endl;
  std::cout << obj6r.getX() << std::endl << std::endl;
  SE3Mat X_e = obj6r.getX();
  obj6r.setX(X_e, false);
  std::cout << obj6r.getConfig() << std::endl << std::endl;
  q << 0, 0, 0, 0, 0, 0;
  obj6r.setConfig(q);
  double Time = 5.0;
  size_t N = 40;
  Eigen::MatrixXd q_path(N, 6);
  Vec6 q_j, q_i;
  std::cout << X_e << std::endl << std::endl;
  std::cout << obj6r.getM() << std::endl;
  std::vector<SE3Mat>  X_path= CartesianTrajectory(obj6r.getM(), X_e, Time, N, QuinticTimeScaling);
  for(int i = 0; i < N; ++i){
    SO3Mat R; Vec3 p;
    TransToRp(X_path[i], R, p);
    std::cout << p[0] << " " << p[1] << std::endl;
    obj6r.setX(X_path[i], false);
    q_j = obj6r.getConfig();
    q_i << wrapToPI(q_j[0]), wrapToPI(q_j[1]), wrapToPI(q_j[2]),wrapToPI(q_j[3]),wrapToPI(q_j[4]),wrapToPI(q_j[5]);
    q_path.row(i) = q_i;
  }
  //std::cout << q_path << std::endl;
}
