#include "pointcloudfilter.h"

#include "pointmatcher/PointMatcher.h"

using namespace PointMatcherSupport;

void PointCloudFilter::minRadius(PointCloud &pointcloud, float radius) {
    PM::DataPointsFilter* filterMin(
                PM::get().DataPointsFilterRegistrar.create(
                    "MinDistDataPointsFilter",
                    map_list_of
                    ("dim", toParam(-1))
                    ("minDist", toParam(radius))));

    filterMin->inPlaceFilter(pointcloud.dataPoints);
}

void PointCloudFilter::maxRadius(PointCloud &pointCloud, float radius) {
    PM::DataPointsFilter* filterMax(
                PM::get().DataPointsFilterRegistrar.create(
                    "MaxDistDataPointsFilter",
                    map_list_of
                    ("dim", toParam(-1))
                    ("maxDist", toParam(radius))));

    filterMax->inPlaceFilter(pointCloud.dataPoints);
}

void PointCloudFilter::maxDensity(PointCloud &pointCloud, float density) {
    PM::DataPointsFilter* filter(
                PM::get().DataPointsFilterRegistrar.create(
                    "MaxDensityDataPointsFilter",
                    map_list_of("maxDensity", toParam(density))));

    filter->inPlaceFilter(pointCloud.dataPoints);
}

void PointCloudFilter::boundingBox(PointCloud &pointCloud,
                                   float xMin, float xMax,
                                   float yMin, float yMax,
                                   float zMin, float zMax,
                                   bool removeInside) {
    PM::DataPointsFilter* filter(
                PM::get().DataPointsFilterRegistrar.create(
                    "BoundingBoxDataPointsFilter",
                    map_list_of
                    ("xMin", toParam(xMin))
                    ("xMax", toParam(xMax))
                    ("yMin", toParam(yMin))
                    ("yMax", toParam(yMax))
                    ("zMin", toParam(zMin))
                    ("zMax", toParam(zMax))
                    ("removeInside", toParam(removeInside))));

    filter->inPlaceFilter(pointCloud.dataPoints);
}
