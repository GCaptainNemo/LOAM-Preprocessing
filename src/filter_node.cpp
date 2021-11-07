#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include "common.h"

ros::Publisher pub_pcl_out1;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
void LivoxMsgCallback(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() == 0) return;   
  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      // PointXYZINormal
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      if (livox_msg->points[i].tag != 16)
      {
        // 去除噪点（1. 基于能量判断的噪点 2. 基于空间位置判断的噪点）
        continue;
      }
      // 实现时间归一化
      float s = livox_msg->points[i].offset_time / (float)time_end;
      // The integer part is line number and the decimal part is timestamp
      pt.intensity = livox_msg->points[i].line + s*0.1; 
      // ROS_INFO("intensity-------- %.6f ",pt.intensity);
      pt.curvature = livox_msg->points[i].reflectivity * 0.1;
      // ROS_INFO("pt.curvature-------- %.3f ",pt.curvature);
      pcl_in.push_back(pt);
      // ROS_INFO("pt.tag -----------%d", livox_msg->points[i].tag);
    }
  }

  /// timebase 5ms ~ 50000000, so 10 ~ 1ns

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  //   ROS_INFO("livox1 republish %u points at time %f buf size %ld",
  //   pcl_in.size(),
  //           timestamp.toSec(), livox_data.size());

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCallback);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_repub", 100);
  ros::spin();
}

