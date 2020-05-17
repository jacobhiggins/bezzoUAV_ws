#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "mesh_vis.h"


MeshVis::MeshVis()
{
  marker.ns = "mesh_visualization";

}

MeshVis::~MeshVis()
{

}

void MeshVis::setType(const std::string &msg_)
{
  int len = msg_.length();
  
  if (len > 10)
  {
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource  = msg_;
  }
  else if (len == 6)
  {
    marker.type = visualization_msgs::Marker::POINTS;
  }

  else if (len == 4)
  {
    marker.type = visualization_msgs::Marker::LINE_STRIP;
  }
}

void MeshVis::setAction()
{
  marker.action = visualization_msgs::Marker::ADD;
}

void MeshVis::setID(const int ID_)
{
  marker.id = ID_;
}

void MeshVis::setHeader(const std_msgs::Header &header_)
{
  marker.header = header_;
  marker.header.stamp = ros::Time::now();
}

void MeshVis::setPos(const geometry_msgs::Point &point_)
{
  marker.pose.position = point_;
}

void MeshVis::setOrientation(const geometry_msgs::Quaternion &quat_)
{
  marker.pose.orientation = quat_;
}

void MeshVis::setColor(const double a_, const double r_, const double g_, const double b_)
{
  marker.color.a = a_;
  marker.color.r = r_;
  marker.color.g = g_;
  marker.color.b = b_;

}
void MeshVis::setScale(const double x_, const double y_, const double z_)
{
  marker.scale.x = x_;
  marker.scale.y = y_;
  marker.scale.z = z_;
}



visualization_msgs::Marker MeshVis::getMarker()
{
  return marker;
}


