#pragma once
#include <QObject>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// Register PCL type for Qt
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;
Q_DECLARE_METATYPE(PointCloudT::Ptr)

/**
 * RosNode: Handles ROS subscriptions and converts messages to PCL types.
 * Inherits QObject to allow signal/slot communication with GUI.
 */
class RosNode : public QObject, public rclcpp::Node {
    Q_OBJECT
public:
    RosNode();
    void setTopic(const QString &topic);

signals:
    // Emitted when a cloud is received/converted
    void cloudReceived(PointCloudT::Ptr cloud);

private:
    void topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::string current_topic_;
};
