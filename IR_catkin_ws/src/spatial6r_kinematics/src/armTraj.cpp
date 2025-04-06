#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <Spatial6R.hpp>
#include <RigidBodyMotion.hpp>
#include <TrajectoryPlanning.hpp>
#include <MathUtils.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <geometry_msgs/Point.h>

using namespace IRlibrary;


double l1, l2, l3;
size_t N = 1000;

double intersectSphere(Vec3 start, Vec3 vel){
  double x1 = start[0]; double y1 = start[1] ; double z1 = start[2];
  double x_ = vel[0]; double y_ = vel[1]; double z_ = vel[2];
  double a = x_*x_ + y_*y_ + z_*z_;
  double b = -2*((x_*(0-x1)+y_*(0-y1)+z_*(l1-z1)));
  double c = x1*x1 + y1*y1 + (l1-z1)*(l1-z1) - (l2+l3)*(l2+l3);
  double t1, t2, t;
  if(b*b - 4*a*c >= 0){
    t1 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
    t2 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
    t = (t1-t2)*0.95+t2;
  }else{ t = -1000000;}
  return t;
}

IRlibrary::SE3Mat finalPosition( IRlibrary::Vec3 start, IRlibrary::Vec3 vel){
  double x1 = start[0]; double y1 = start[1] ; double z1 = start[2];
  double x_ = vel[0]; double y_ = vel[1]; double z_ = vel[2];
  double t = intersectSphere(start, vel);
  if(t != -1000000){
  Vec3 end_x;
  end_x << x1+t*x_, y1+t*y_, z1+t*z_;
  if(end_x[2] < l1-l3-l2 or end_x[2] < 0){return SE3Mat::Zero();}
  double theta_z = std::atan2(y_, x_) - M_PI/2;
  double theta_x = std::atan2(z_, std::sqrt(y_*y_+x_*x_))+M_PI/2;
  SO3Mat Rotz, Rotx, Rot;
  Rotx << 1, 0, 0, 0, std::cos(theta_x), -std::sin(theta_x),  0, std::sin(theta_x), std::cos(theta_x);
  Rotz << std::cos(theta_z), -std::sin(theta_z), 0,  std::sin(theta_z), std::cos(theta_z), 0,  0, 0, 1;
  Rot = Rotz*Rotx;
  return RpToTrans(Rot,end_x);
  }else{
    return SE3Mat::Zero();
  }
}

std::vector<Vec6> PathGeneration(SE3Mat s_pos, SE3Mat e_pos, Spatial6R spatial6R, double time, size_t N, double type = 0.0){
  std::vector<SE3Mat> SE3list;
  std::vector<Vec6> q_list;
  if (type == 2.0){
    Eigen::MatrixXd q_mat;
    Vec6 s_config, e_config;
    s_config = spatial6R.getConfig();
    spatial6R.setX(e_pos);
    e_config = spatial6R.getConfig();
    q_mat = JointTrajectory(s_config, e_config, time-1, N, QuinticTimeScaling);
    for(int i = 0; i < N; ++i){q_list.push_back(q_mat.row(i));}
    return q_list;
  }
  if(type == 0.0){
    SE3list = CartesianTrajectory(s_pos, e_pos, time-1, N, QuinticTimeScaling);
  }else if(type == 1.0){
    SE3list = ScrewTrajectory(s_pos, e_pos, time-1, N, QuinticTimeScaling);
  }
  Vec6 q_i, q_j;
  for(int i = 0; i < SE3list.size(); ++i){
    spatial6R.setX(SE3list[i]);
    q_j = spatial6R.getConfig();
    q_i << wrapToPI(q_j[0]), wrapToPI(q_j[1]), wrapToPI(q_j[2]),wrapToPI(q_j[3]),wrapToPI(q_j[4]),wrapToPI(q_j[5]);
    q_list.push_back(q_i);
  }
  return q_list;
}

int main(int argc, char **argv)
{
  std::ofstream logger;//, x_err_num,  q_diff, num_time, ana_time;
  logger.open("/home/ysu/log.txt");
  ros::init(argc, argv, "armTraj");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  double s_x, s_y, s_z, v_x, v_y, v_z, alpha, t;
  n.getParam("link_lengths/l1", l1);
  n.getParam("link_lengths/l2", l2);
  n.getParam("link_lengths/l3", l3);
  n.getParam("ball_start/x", s_x);
  n.getParam("ball_start/y", s_y);
  n.getParam("ball_start/z", s_z);
  n.getParam("ball_vel/x", v_x);
  n.getParam("ball_vel/y", v_y);
  n.getParam("ball_vel/z", v_z);
  n.getParam("trajectory_type/alpha", alpha);
  Vec3 s, v;
  SE3Mat X_e;
  s << s_x, s_y, s_z;
  v << v_x, v_y, v_z;
  t = intersectSphere(s, v);
  X_e = finalPosition(s, v);
  logger << "time: " << t << std::endl;
  logger << "Final_P" << X_e << std::endl;
  logger << "alpha: " << alpha << std::endl;
  ros::Duration(0.01).sleep();
  ros::Publisher configPub, WsPub, posPub;
  WsPub = n.advertise <std_msgs::Bool> ("/WsCheck", 5);
  configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5);
  posPub = n.advertise <geometry_msgs::Point> ("/armPos", 5);/* Fix this line. Do NOT change "/pubJointStates" */
  ros::Time start = ros::Time::now();
  sensor_msgs::JointState new_state;
  geometry_msgs::Point pos;
  std_msgs::Bool wscheck;

    
  if (X_e.isApprox(SE3Mat::Zero())){
    wscheck.data = false;
    WsPub.publish(wscheck);
    while (ros::ok()){WsPub.publish(wscheck); ros::spinOnce();}
  }else{
    wscheck.data = true;
    WsPub.publish(wscheck);
    new_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    new_state.header.stamp = ros::Time::now();
    new_state.position = { 0, 0, 0, 0, 0, 0};
    Spatial6R obj6r(l1, l2, l3);
    std::vector<double> wsRad = obj6r.getwsRad();
    int i = 0;
    std::vector<Vec6> q_list = PathGeneration(obj6r.getM(), X_e, obj6r, t*0.5, N, alpha);
    ros::Time start=ros::Time::now();
    double diff;
    for(int i = 0; i < q_list.size(); ++i){logger << q_list[i] << std::endl;}
    while (ros::ok())
      {
	if( i < N){
	  diff = double((ros::Time::now() - start).toSec());
	  i = int(double(N)*(diff/(t*0.5)));
	  //double t_lapse = (t-1)/N;
	  new_state.header.stamp = ros::Time::now();
	  new_state.position[0] = q_list[i][0];
	  new_state.position[1] = q_list[i][1];
	  new_state.position[2] = q_list[i][2];
	  new_state.position[3] = q_list[i][3];
	  new_state.position[4] = q_list[i][4];
	  new_state.position[5] = q_list[i][5];
	  configPub.publish(new_state);
	  //posPub.publish(pos);
	  WsPub.publish(wscheck);
	  ros::spinOnce();
	  //ros::Duration(t_lapse).sleep();	  
	}
	else{
	  new_state.position[0] = q_list[N-1][0];
	  new_state.position[1] = q_list[N-1][1];
	  new_state.position[2] = q_list[N-1][2];
	  new_state.position[3] = q_list[N-1][3];
	  new_state.position[4] = q_list[N-1][4];
	  new_state.position[5] = q_list[N-1][5];
	  obj6r.setConfig(q_list[N-1]);
	  Vec3 p_i = obj6r.getPos();
	  pos.x = p_i[0]; pos.y = p_i[1]; pos.z = p_i[2];
	  posPub.publish(pos);
	  configPub.publish(new_state);
	  ros::spinOnce();
	}
  
      }
    logger.close();
  //  x_err_ana.close();
  //x_err_num.close();
  //q_diff.close();
  //ana_time.close();
  //num_time.close();
  return 0;
  }
}
