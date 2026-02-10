
// ============================================================
// Test Fixture
// ============================================================
// A "fixture" is a class that sets up common state for multiple tests.
// This follows the DRY principle — we don't repeat setup code in every test.
// gtest creates a FRESH instance of this fixture for EACH test,
// so tests are completely isolated from each other.
//
//
#include "minimal_mapping_tool/cloud_mapper.hpp"
#include <gtest/gtest.h>
#include <memory>

using PointCloudT = CloudMapper::CloudT;

class CloudMapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        mapper_ = std::make_unique<CloudMapper>();
    }

    // Helper: creates a small synthetic point cloud
    // This is a "test utility" — a pure function that generates test data.
    // In industry, these often live in a separate test_helpers.hpp file.
    static PointCloudT::Ptr makeCloud(std::size_t num_points,
                                      float x_offset = 0.0f) {
        auto cloud = std::make_shared<PointCloudT>();
        cloud->reserve(num_points);
        for (std::size_t i = 0; i < num_points; ++i) {
            pcl::PointXYZRGB pt;
            pt.x = x_offset + static_cast<float>(i) * 0.01f;
            pt.y = static_cast<float>(i) * 0.01f;
            pt.z = 0.0f;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            cloud->push_back(pt);
        }
        return cloud;
    }

    std::unique_ptr<CloudMapper> mapper_;
};

// ============================================================
// Unit Tests
// ============================================================

// TEST_F means "test using a fixture" — it gets a fresh CloudMapperTest instance
// Test names follow: TEST_F(FixtureName, WhatWeAreTestingAndExpectedBehavior)
//
//
TEST_F(CloudMapperTest, InitialMapIsEmpty) {
    // ARRANGE: mapper_ is created in SetUp() — already done
    // ACT: nothing — we're testing initial state
    // ASSERT: A newly created mapper should have an empty global map
    const auto map = mapper_->getMap();
    EXPECT_TRUE(map->empty()) << "A fresh CloudMapper should have an empty global map";
}

TEST_F(CloudMapperTest, AddCloudPopulatesGlobalMap) {
    auto cloud = makeCloud(100);
    mapper_->addCloud(cloud);
    EXPECT_FALSE(mapper_->getMap()->empty()) << "After adding a cloud, global map should not be empty";
}

TEST_F(CloudMapperTest, ClearResetsGlobalMap) {
    mapper_->addCloud(makeCloud((100)));
    ASSERT_FALSE(mapper_->getMap()->empty()); // precondition

    // ACT
    mapper_->clear();

    // ASSERT
    EXPECT_TRUE(mapper_->getMap()->empty())
        << "After clear(), the global map should be empty";
}

TEST_F(CloudMapperTest, AddCloudReturnsFiniteScore) {
    // ARRANGE
    auto cloud = makeCloud(50);

    // ACT
    const double fitness = mapper_->addCloud(cloud);

    // ASSERT: ICP fitness score should be a real number, not NaN or infinity
    EXPECT_TRUE(std::isfinite(fitness)) << "ICP fitness score should be a finite number, got: " << fitness;
}

TEST_F(CloudMapperTest, AddEmptyCloudDoesNotCrash) {
    // This is a DEFENSIVE test — it verifies edge-case behavior.
    // In industry, these are often the tests that save you from
    // production crashes. "What happens if the sensor sends no data?"
    auto empty_cloud = std::make_shared<PointCloudT>();

    // ACT & ASSERT: Should not throw or segfault
    EXPECT_NO_THROW(mapper_->addCloud(empty_cloud));
}
