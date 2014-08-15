#include "gmock/gmock.h"
#include "pointcloud.h"

TEST(PointCloudTest, PointCloudDontEmptyWhenCreated) {
   PointCloud pointcloud;

   EXPECT_EQ(pointcloud.getNbPoints(), 0);
}
