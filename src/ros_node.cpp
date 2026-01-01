#include "minimal_mapping_tool/ros_node.hpp"
#include <pcl_conversions/pcl_conversions.h>

RosNode::RosNode() : Node("mapping_node") {
    // Initialize with a default topic often used in simulations
    this->setTopic("/camera/depth/points"); 
}

void RosNode::setTopic(const QString & topic) {
    std::string topic_str = topic.toStdString();
    if (topic_str == current_topic_) return;
    
    current_topic_ = topic_str;
    
    // Subscribe using SensorData QoS (best effort) for low latency
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        current_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RosNode::topicCallback, this, std::placeholders::_1));
}

void RosNode::topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert ROS msg to PCL cloud
    PointCloudT::Ptr pcl_cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *pcl_cloud);
    
    // Emit signal to GUI thread (Thread-safe via Qt QueuedConnection)
    emit cloudReceived(pcl_cloud);
}
