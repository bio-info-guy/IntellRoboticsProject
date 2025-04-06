#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <Spatial6R.hpp>
#include <fstream>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;
visualization_msgs::Marker ball;
IRlibrary::Vec3 POSITION;
bool CAUGHT = false;
std::ofstream ball_file;
IRlibrary::Spatial6R obj6R;
double l1, l2, l3;

void catchCheckCallBack(const geometry_msgs::Point &msg) {
  IRlibrary::Vec6 q;
  IRlibrary::Spatial6R obj6R(l1, l2, l3);
  //q << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];
  //obj6R.setConfig(q);
  IRlibrary::Vec3 p;// = obj6R.getPos();
  p << msg.x, msg.y, msg.z;
  //ball_file << p[0] << " " << p[1] << " " << p[2] << std::endl;
  ball_file << (p-POSITION).norm() << std::endl;
  if((p - POSITION).norm() < 1e-2) {
    CAUGHT = true;
    marker.color.g = 1.0f;
    marker.color.r = 0.0f;
    marker.text = "ball has been caught";
  }
  else {
    marker.color.g = 0.0f;
    marker.color.r = 1.0f;
    marker.text = "ball has not been caught yet";
  }
  marker_pub.publish(marker);
}


void draw_ball(IRlibrary::Vec3 v3){
  ball.header.frame_id = "/base_link";
  ball.header.stamp = ros::Time::now();
  ball.ns= "basic_shapes";
  ball.id= 10;
  ball.type = visualization_msgs::Marker::SPHERE;
  ball.action= visualization_msgs::Marker::ADD;
  ball.pose.position.x = v3[0];
  ball.pose.position.y = v3[1];
  ball.pose.position.z = v3[2];
  ball.pose.orientation.x = 0.0;
  ball.pose.orientation.y = 0.0;
  ball.pose.orientation.z = 0.0;
  ball.pose.orientation.w = 1.0;
  ball.scale.x = 0.2;
  ball.scale.y = 0.2;
  ball.scale.z = 0.2;
  ball.color.a = 1.0; // Don't forget to set the alpha!
  ball.color.r = 1.0;
  ball.color.g = 0.0;
  ball.color.b = 0.5;
  //  ball.mesh_resource = "package://spatial6r_description/meshes/visuals/sphere_32_16.dae";
  //vis_pub.publish(ball)
}	    

void move_ball(IRlibrary::Vec3 v3){
  ball.pose.position.x = v3[0];
  ball.pose.position.y = v3[1];
  ball.pose.position.z = v3[2];
  marker_pub.publish(ball);
}


int main( int argc, char** argv )
{
  ball_file.open("/home/ysu/ball.txt");
  double s_x, s_y, s_z, v_x, v_y, v_z;//, l1, l2, l3;
  ros::init(argc, argv, "ball_monitor");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  n.getParam("ball_start/x", s_x);
  n.getParam("ball_start/y", s_y);
  n.getParam("ball_start/z", s_z);
  n.getParam("ball_vel/x", v_x);
  n.getParam("ball_vel/y", v_y);
  n.getParam("ball_vel/z", v_z);
  n.getParam("link_lengths/l1", l1);
  n.getParam("link_lengths/l2", l2);
  n.getParam("link_lengths/l3", l3);
  //  obj6R.setLinks(l1, l2, l3);
  uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;

  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  
  marker.ns = "catch_logo";
  marker.id = 0;
  
  marker.type = shape;
  
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.position.x = 6;
  marker.pose.position.y = -5;
  marker.pose.position.z = 1;
  marker.text = "Ready for Ball";
  
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 0.3;
  
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  IRlibrary::Vec3 start, velocity;
  start << s_x, s_y, s_z;
  POSITION = start;
  velocity << v_x, v_y, v_z;

  marker.lifetime = ros::Duration();
  ball.lifetime = ros::Duration();
  ros::Time s = ros::Time::now();
  ros::Subscriber catchCheckSub;
  ros::Subscriber configSub;
  double diff;
  draw_ball(start);
  marker_pub.publish(ball);
  while (ros::ok())
    {
      catchCheckSub = n.subscribe("/armPos", 5, &catchCheckCallBack);
      if(!CAUGHT){
      diff = double((ros::Time::now() - s).toSec());
      POSITION = start + diff*velocity;
      move_ball(POSITION);}
      ros::spinOnce();
    }

ball_file.close();
}
