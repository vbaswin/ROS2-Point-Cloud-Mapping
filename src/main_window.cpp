#include "minimal_mapping_tool/main_window.hpp"
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>

MainWindow::MainWindow(std::shared_ptr<RosNode> node, QWidget *parent)
    : QMainWindow(parent), node_(node) {
  setupUi();

  // Initialize PCL Visualizer linked to VTK widget
  auto render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  render_window->AddRenderer(renderer);

  vtk_widget_->setRenderWindow(render_window);
  visualizer_.reset(new pcl::visualization::PCLVisualizer(
      renderer, render_window, "viewer", false));
  visualizer_->setBackgroundColor(0.1, 0.1, 0.1);
  visualizer_->addCoordinateSystem(1.0);

  // Connect ROS signal to GUI slot
  connect(node_.get(), &RosNode::cloudReceived, this, &MainWindow::updateCloud);
}

MainWindow::~MainWindow() {}

void MainWindow::setupUi() {
  QWidget *central = new QWidget(this);
  setCentralWidget(central);
  QVBoxLayout *main_layout = new QVBoxLayout(central);

  // 1. Dashboard Section (Stats & Controls)
  QGroupBox *dashboard = new QGroupBox("Mapping Dashboard");
  QHBoxLayout *dash_layout = new QHBoxLayout(dashboard);

  lbl_fitness_ = new QLabel("Overlap Fitness: N/A");
  lbl_points_ = new QLabel("Map Points: 0");

  QPushButton *btn_map = new QPushButton("Start Mapping");
  btn_map->setCheckable(true);
  connect(btn_map, &QPushButton::toggled, this, &MainWindow::toggleMapping);

  QPushButton *btn_save = new QPushButton("Export Map");
  connect(btn_save, &QPushButton::clicked, this, &MainWindow::saveMap);

  dash_layout->addWidget(lbl_fitness_);
  dash_layout->addWidget(lbl_points_);
  dash_layout->addWidget(btn_map);
  dash_layout->addWidget(btn_save);

  main_layout->addWidget(dashboard);

  // 2. Visualization Section
  vtk_widget_ = new QVTKOpenGLNativeWidget();
  main_layout->addWidget(vtk_widget_, 1); // Expand to fill space
}

void MainWindow::updateCloud(PointCloudT::Ptr cloud) {
  if (!cloud || cloud->empty())
    return;

  if (is_mapping_) {
    // Add to map and get fitness score
    double fitness = mapper_.addCloud(cloud);

    // Update Visualizer with full map
    PointCloudT::Ptr map = mapper_.getMap();
    if (!visualizer_->updatePointCloud(map, "cloud")) {
      visualizer_->addPointCloud(map, "cloud");
    }

    // Update Dashboard stats
    updateDashboard(fitness, map->size());
  } else {
    // Just show live feed
    if (!visualizer_->updatePointCloud(cloud, "cloud")) {
      visualizer_->addPointCloud(cloud, "cloud");
    }
  }
  vtk_widget_->renderWindow()->Render();
}

void MainWindow::toggleMapping(bool checked) {
  is_mapping_ = checked;
  if (!checked)
    mapper_.clear(); // Optional: clear on stop
}

void MainWindow::saveMap() {
  QString filename =
      QFileDialog::getSaveFileName(this, "Save Map", "", "PCD Files (*.pcd)");
  if (!filename.isEmpty()) {
    mapper_.saveMap(filename.toStdString());
  }
}

void MainWindow::updateDashboard(double fitness, size_t points) {
  // Fitness: lower is better. Color code red if alignment is poor (> 0.05)
  QString color = (fitness > 0.05) ? "red" : "green";
  lbl_fitness_->setText(
      QString("Overlap Fitness: <span style='color:%1'>%2</span>")
          .arg(color)
          .arg(fitness, 0, 'f', 4));
  lbl_points_->setText(QString("Map Points: %1").arg(points));
}
