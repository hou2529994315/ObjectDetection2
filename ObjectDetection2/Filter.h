#pragma once
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
class Filter
{
public:
	void passThroughFilter(PointCloudT::Ptr& cloud);
	void voxelGridFilter(PointCloudT::Ptr& cloud);
	void statisticalFilter(PointCloudT::Ptr& cloud);
};

