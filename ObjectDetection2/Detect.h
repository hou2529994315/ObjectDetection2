#pragma once
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Detect
{
public:
	PointCloudT::Ptr detectPlain(PointCloudT::Ptr& cloud);
	int detectObject(PointCloudT::Ptr& cloud);
	std::vector<std::vector<float>> getWHD() {
		return w_h_d;
	}
	std::vector<Eigen::Quaternionf> getRotat() {
		return rotat;
	}
	std::vector<Eigen::Vector3f> getTranslat() {
		return translat;
	}

private:
	std::vector<std::vector<float>> w_h_d;
	std::vector<Eigen::Quaternionf> rotat;	// a quaternion-based rotation to apply to the cube
	std::vector<Eigen::Vector3f> translat;		// a translation to apply to the cube from 0,0,0
	void bbox(PointCloudT::Ptr cloud);
};

