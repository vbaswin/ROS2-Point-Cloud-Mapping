#include "minimal_mapping_tool/cloud_mapper.hpp"
#include <chrono>
#include <cstddef>
#include <pcl/io/pcd_io.h>
#include <vtkDataSetAttributes.h>

CloudMapper::CloudMapper() : global_map_(new CloudT) {
    voxel_filter_.setLeafSize(0.08f, 0.08f, 0.08f);

    // Setup ICP: limit distance to 10cm to prevent matching distinct walls
    icp_.setMaxCorrespondenceDistance(0.2);
    icp_.setMaximumIterations(10);
    icp_.setTransformationEpsilon(1e-5);
    icp_.setEuclideanFitnessEpsilon(1e-5);
}

double CloudMapper::addCloud(CloudPtr new_cloud) {
    frame_count_++;
    if (frame_count_ % 3 != 0) {
        return -2.0;
    }
    if (new_cloud->empty())
        return -1.0;

    auto start = std::chrono::high_resolution_clock::now();

    CloudPtr downsampled(new CloudT);
    voxel_filter_.setInputCloud(new_cloud);
    voxel_filter_.filter(*downsampled);

    const size_t MAX_POINTS = 2000;
    if (downsampled->size() > MAX_POINTS) {
        CloudPtr limited(new CloudT);
        limited->reserve(MAX_POINTS);
        size_t step = downsampled->size() / MAX_POINTS;
        for (size_t i = 0; i < downsampled->size() && limited->size() < MAX_POINTS; i += step) {
            limited->push_back(downsampled->points[i]);
        }
        limited->width = limited->size();
        limited->height = 1;
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

    CloudPtr target = global_map_;
    const size_t MAX_TARGET = 5000;
    if (global_map_->size() > MAX_TARGET) {
        target.reset(new CloudT);
        size_t step = global_map_->size() / MAX_TARGET;
        for (size_t i = 0; i < global_map_->size() && target->size() < MAX_TARGET; i += step) {
            target->push_back(global_map_->points[i]);
        }
        target->width = target->size();
        target->height = 1;
    }

    // Align new cloud to the existing global map
    CloudT aligned_cloud;
    icp_.setInputSource(downsampled);
    icp_.setInputTarget(target);
    icp_.align(aligned_cloud);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // If alignment converged, merge and downsample
    if (icp_.hasConverged()) {
        *global_map_ += aligned_cloud;

        // Downsample immediate map to prevent unbounded memory growth
        CloudPtr temp(new CloudT);
        voxel_filter_.setInputCloud(global_map_);
        voxel_filter_.filter(*temp);

        const size_t MAX_MAP = 15000;
        if (temp->size() > MAX_MAP) {
            CloudPtr limited(new CloudT);
            limited->points.assign(
                temp->points.end() - MAX_MAP,
                temp->points.end());
            limited->width = MAX_MAP;
            limited->height = 1;
            limited->is_dense = true;
            global_map_ = limited;
        } else {
            global_map_ = temp;
        }

        std::cout << "[Mapper] ICP: " << duration.count() << "ms, map: " << global_map_->size() << " pts, fitness: " << icp_.getFitnessScore() << std::endl;
        // Return fitness score for the dashboard
        return icp_.getFitnessScore();
    }

    std::cout << "[Mapper] ICP failed after " << duration.count() << "ms" << std::endl;
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
