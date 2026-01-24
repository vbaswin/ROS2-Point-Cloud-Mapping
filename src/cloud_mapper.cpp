#include "minimal_mapping_tool/cloud_mapper.hpp"
#include <pcl/io/pcd_io.h>

CloudMapper::CloudMapper() : global_map_(new CloudT) {
    // Setup ICP: limit distance to 10cm to prevent matching distinct walls
    icp_.setMaxCorrespondenceDistance(0.1);
    icp_.setMaximumIterations(30);
    icp_.setTransformationEpsilon(1e-8);

    // Setup VoxelGrid: Downsample map to 5cm leaf size to maintain performance
    voxel_filter_.setLeafSize(0.05f, 0.05f, 0.05f);
}

double CloudMapper::addCloud(CloudPtr new_cloud) {
    if (new_cloud->empty())
        return -1.0;

    CloudPtr downsampled(new CloudT);
    voxel_filter_.setInputCloud(new_cloud);
    voxel_filter_.filter(*downsampled);

    if (downsampled->size() < 100)
        return -1.0;

    // First frame initialization
    if (global_map_->empty()) {
        *global_map_ = *new_cloud;
        return 0.0;
    }

    // Align new cloud to the existing global map
    CloudT aligned_cloud;
    icp_.setInputSource(new_cloud);
    icp_.setInputTarget(global_map_);
    icp_.align(aligned_cloud);

    // If alignment converged, merge and downsample
    if (icp_.hasConverged()) {
        *global_map_ += aligned_cloud;

        // Downsample immediate map to prevent unbounded memory growth
        CloudPtr temp(new CloudT);
        voxel_filter_.setInputCloud(global_map_);
        voxel_filter_.filter(*temp);
        global_map_ = temp;

        // Return fitness score for the dashboard
        return icp_.getFitnessScore();
    }

    return -1.0; // Alignment failed
}

CloudMapper::CloudPtr CloudMapper::getMap() const {
    return global_map_;
}

void CloudMapper::clear() {
    global_map_->clear();
}

void CloudMapper::saveMap(const std::string &filename) {
    // Export functionality for "bigger maps down the line"
    pcl::io::savePCDFileBinary(filename, *global_map_);
}
