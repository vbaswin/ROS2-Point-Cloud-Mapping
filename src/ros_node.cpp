
#include "minimal_mapping_tool/ros_node.hpp"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>

RosNode::RosNode() : Node("mapping_node") {
    // Initialize with a default topic often used in simulations
    this->setTopic("/camera/depth/points");
}

void RosNode::setTopic(const QString &topic) {
    std::string topic_str = topic.toStdString();
    if (topic_str == current_topic_)
        return;

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

    PointCloudT::Ptr filtered_cloud(new PointCloudT);
    filtered_cloud->reserve(pcl_cloud->size());

    for (auto &point : pcl_cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        std::swap(point.r, point.b);
        filtered_cloud->push_back(point);
    }

    filtered_cloud->width = filtered_cloud->size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    // Emit signal to GUI thread (Thread-safe via Qt QueuedConnection)
    emit cloudReceived(filtered_cloud);
}
