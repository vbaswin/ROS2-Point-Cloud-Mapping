#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "minimal_mapping_tool/main_window.hpp"
#include "minimal_mapping_tool/ros_node.hpp"

int main(int argc, char** argv) {
    // 1. Init Qt and ROS
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);
    
    // 2. Register Metadata for Signal/Slot
    qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr");
    
    // 3. Create Node
    auto node = std::make_shared<RosNode>();
    
    // 4. Create Worker Thread for ROS
    // We use std::thread for simplicity. The node lives until main exits.
    std::thread ros_thread([node](){
        rclcpp::spin(node);
    });
    
    // 5. Launch GUI
    MainWindow window(node);
    window.setWindowTitle("ROS2 Point Cloud Mapping Tool");
    window.resize(1024, 768);
    window.show();
    
    // 6. Run Qt Event Loop
    int result = app.exec();
    
    // 7. Cleanup
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    return result;
}
