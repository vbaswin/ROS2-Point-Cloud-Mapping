#pragma once
#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QLabel>
#include "ros_node.hpp"
#include "cloud_mapper.hpp"

/**
 * MainWindow: The View/Controller. Manages VTK widget and Dashboard.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(std::shared_ptr<RosNode> node, QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void updateCloud(PointCloudT::Ptr cloud);
    void toggleMapping(bool checked);
    void saveMap();

private:
    void setupUi();
    void updateDashboard(double fitness, size_t points);

    // Core components
    std::shared_ptr<RosNode> node_;
    CloudMapper mapper_;
    pcl::visualization::PCLVisualizer::Ptr visualizer_;
    
    // UI Components
    QVTKOpenGLNativeWidget* vtk_widget_;
    QLabel* lbl_fitness_; // Dashboard: Overlap quality
    QLabel* lbl_points_;  // Dashboard: Map size
    bool is_mapping_ = false;
};
