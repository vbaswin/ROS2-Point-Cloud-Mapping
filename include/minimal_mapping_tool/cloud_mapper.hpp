#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

/**
 * CloudMapper: Responsible for aligning incoming clouds to a global map.
 * Implements SRP: Only handles PCL logic.
 */
class CloudMapper {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    using CloudPtr = CloudT::Ptr;

    CloudMapper();

    // Aligns a new cloud and returns the alignment fitness score (lower is better)
    double addCloud(CloudPtr new_cloud);

    CloudPtr getMap() const;
    void clear();
    void saveMap(const std::string &filename);

private:
    CloudPtr global_map_;
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
    pcl::VoxelGrid<PointT> voxel_filter_;
    int frame_count_ = 0;
};
