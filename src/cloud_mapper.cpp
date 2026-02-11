#include "minimal_mapping_tool/cloud_mapper.hpp"
#include <chrono>
#include <cstddef>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <vtkDataSetAttributes.h>

CloudMapper::CloudMapper() : global_map_(new CloudT) {
    input_filter_.setLeafSize(0.08f, 0.08f, 0.08f);
    map_filter_.setLeafSize(0.05f, 0.05f, 0.05f);

    // Setup ICP: limit distance to 10cm to prevent matching distinct walls
    icp_.setMaxCorrespondenceDistance(0.5);
    icp_.setMaximumIterations(20);
    icp_.setTransformationEpsilon(1e-6);
    icp_.setEuclideanFitnessEpsilon(1e-6);
}

double CloudMapper::addCloud(CloudPtr new_cloud) {
    frame_count_++;
    if (frame_count_ % 3 != 0) {
        return -2.0;
    }
    if (!new_cloud || new_cloud->empty())
        return -1.0;

    auto start = std::chrono::high_resolution_clock::now();

    CloudPtr downsampled(new CloudT);
    input_filter_.setInputCloud(new_cloud);
    input_filter_.filter(*downsampled);

    const size_t MAX_POINTS = 3000;
    if (downsampled->size() > MAX_POINTS) {
        CloudPtr limited(new CloudT);
        limited->reserve(MAX_POINTS);
        size_t step = downsampled->size() / MAX_POINTS;
        for (size_t i = 0; i < downsampled->size() && limited->size() < MAX_POINTS; i += step) {
            limited->push_back(downsampled->points[i]);
        }
        limited->width = limited->size();
        limited->height = 1;
        limited->is_dense = true;
        downsampled = limited;
    }

    if (downsampled->size() < 30)
        return -1.0;

    // First frame initialization
    if (global_map_->empty()) {
        *global_map_ = *downsampled;
        std::cout << "[Mapper] Initialized with " << global_map_->size() << " points" << std::endl;
        return 0.0;
    }

    CloudPtr icp_target = global_map_;
    const size_t MAX_icp_target = 10000;
    if (global_map_->size() > MAX_icp_target) {
        icp_target.reset(new CloudT);
        size_t step = global_map_->size() / MAX_icp_target;
        for (size_t i = 0; i < global_map_->size() && icp_target->size() < MAX_icp_target; i += step) {
            icp_target->push_back(global_map_->points[i]);
        }
        icp_target->width = icp_target->size();
        icp_target->height = 1;
        icp_target->is_dense = true;
    }

    // Align new cloud to the existing global map
    CloudT aligned_cloud;
    icp_.setInputSource(downsampled);
    icp_.setInputTarget(icp_target);
    icp_.align(aligned_cloud);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if (!icp_.hasConverged()) {
        std::cout << "[Mapper] ICP failed (" << duration.count() << "ms)" << std::endl;
        return -1.0;
    }

    *global_map_ += aligned_cloud;

    // Downsample immediate map to prevent unbounded memory growth
    CloudPtr compressed(new CloudT);
    map_filter_.setInputCloud(global_map_);
    map_filter_.filter(*compressed);

    const size_t MAX_MAP = 500000;
    if (compressed->size() > MAX_MAP) {
        std::cout << "[Mapper] WARNING: Map hit " << compressed->size()
                  << " points (cap=" << MAX_MAP << "). consider coarser map_filter leaf."
                  << std::endl;
        pcl::VoxelGrid<PointT> emergency_filter;
        emergency_filter.setLeafSize(0.08f, 0.08f, 0.08f);
        CloudPtr reduced(new CloudT);
        emergency_filter.setInputCloud(compressed);
        emergency_filter.filter(*reduced);
        global_map_ = reduced;
    } else {
        global_map_ = compressed;
    }

    const double fitness = icp_.getFitnessScore();
    std::cout << "[Mapper] ICP: " << duration.count() << "ms | map: "
              << global_map_->size() << " pts | fitness: " << fitness
              << std::endl;

    return fitness;
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
    std::cout << "[Mapper] Saved " << global_map_->size()
              << " points to " << filename << std::endl;
}
