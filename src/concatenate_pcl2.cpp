#include <iostream>
#include <string>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <dynamic_reconfigure/server.h>
#include <concatenate_pcl2/concatenateConfig.h>

class Concatenate
{
public:

  Concatenate()
  {
    nh_.param<float>("theta",    theta,    -M_PI);
    nh_.param<float>("x_offset", x_offset, -1.15);
    nh_.param<float>("y_offset", y_offset, -0.23);

    nh_.param<std::string>("cloud1",    cloud_topic1,    "/points2_front");
    nh_.param<std::string>("cloud2",    cloud_topic2,    "/points2_rear");
    nh_.param<std::string>("cloud_out", out_cloud_topic, "/points2_out");

    front_subscriber = nh_.subscribe(cloud_topic1, 100, &Concatenate::frontCallBack, this);
    rear_subscriber  = nh_.subscribe(cloud_topic2 , 100, &Concatenate::rearCallBack, this);
    out_publisher    = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 10, false);

    transform_1 = Eigen::Matrix4f::Identity();

    transform_1 (0,0) = std::cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = std::cos (theta);

    transform_1 (0,3) = x_offset;
    transform_1 (1,3) = y_offset;

    // f = boost::bind(&Concatenate::dynamicCallBack, this, _1, _2);
    // server.setCallback(f);
  }

  // void dynamicCallBack(concatenate_pcl2::concatenateConfig &config, uint32_t level)
  // {
  //   x_offset = config.x_offset;
  //   y_offset = config.y_offset;

  //   transform_1 = Eigen::Matrix4f::Identity();

  //   transform_1 (0,0) = std::cos (theta);
  //   transform_1 (0,1) = -sin(theta);
  //   transform_1 (1,0) = sin (theta);
  //   transform_1 (1,1) = std::cos (theta);

  //   transform_1 (0,3) = x_offset;
  //   transform_1 (1,3) = y_offset;
  // }

  void frontCallBack(const sensor_msgs::PointCloud2 &cloud)
  {
    pcl2_front = cloud;
  }

  void rearCallBack(const sensor_msgs::PointCloud2 &cloud)
  {
    pcl_ros::transformPointCloud (transform_1, cloud, pcl2_rear);
  }

  sensor_msgs::PointCloud2& getFrontPCL2()
  {
    return pcl2_front;
  }


  sensor_msgs::PointCloud2& getRearPCL2()
  {
    return pcl2_rear;
  }

  void PublishOut(const sensor_msgs::PointCloud2 &cloud)
  {
    out_publisher.publish(cloud);
  }

private:
  ros::NodeHandle nh_;

  float theta;
  float x_offset;
  float y_offset;
  std::string cloud_topic1;
  std::string cloud_topic2;
  std::string out_cloud_topic;

  ros::Publisher out_publisher;
  ros::Subscriber front_subscriber;
  ros::Subscriber rear_subscriber;

  // dynamic_reconfigure::Server<concatenate_pcl2::concatenateConfig> server;
  // dynamic_reconfigure::Server<concatenate_pcl2::concatenateConfig>::CallbackType f;

  Eigen::Matrix4f transform_1;

  sensor_msgs::PointCloud2 pcl2_front, pcl2_rear;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv,"concatenate_pcl2");

  Concatenate c;

  sensor_msgs::PointCloud2 cloud_out;

  ros::Rate loop_rate(15);

  while(ros::ok())
  {
    cloud_out.header.stamp = ros::Time::now();
    pcl::concatenatePointCloud (c.getFrontPCL2(), c.getRearPCL2(), cloud_out);

    c.PublishOut(cloud_out);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}