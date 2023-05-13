#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out0;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver2::msg::CustomMsg::SharedPtr> livox_data;

void LivoxMsgCbk1(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      float s = livox_msg->points[i].offset_time / (float)time_end;

      pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
      pt.curvature = s*0.1;
      pcl_in.push_back(pt);
    }
  }

  unsigned long timebase_ns = livox_data[0]->timebase;
  rclcpp::Time timestamp(timebase_ns);

  sensor_msgs::msg::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp = rclcpp::Time(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out1->publish(std::move(pcl_ros_msg));
  livox_data.clear();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::cout << "livox_repub" << std::endl;
  auto node = std::make_shared<rclcpp::Node>("livox_repub");
  auto sub_livox_msg1 = node->create_subscription<livox_ros_driver2::msg::CustomMsg>("livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = node->create_publisher<sensor_msgs::msg::PointCloud2>("livox_pcl0", rclcpp::QoS(100));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
