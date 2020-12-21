#include "Filter.h"
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

void Filter::passThroughFilter(PointCloudT::Ptr& cloud)
{
	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;
	pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
	pass.setInputCloud(cloud);//设置输入点云
	pass.setFilterFieldName("z");//设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(0.0, 1.5);//设置在过滤字段上的范围
	pass.filter(*cloud);//执行过滤，过滤结果在cloud_filtered
	pass.setFilterFieldName("x");//设置过滤时所需要点云类型的X字段
	pass.setFilterLimits(-0.5, 0.5);//设置在过滤字段上的范围
	pass.filter(*cloud);//执行过滤，过滤结果在cloud_filtered

	//std::cerr << "PointCloud after passthroughfiltering: " << cloud->width * cloud->height
	//	<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

}

void Filter::voxelGridFilter(PointCloudT::Ptr& cloud)
{
	/******************************************************************************
			创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
	**********************************************************************************/
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;  //创建滤波对象
	sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	sor.filter(*cloud);           //执行滤波处理，存储输出

	//std::cerr << "PointCloud after voxelgridfiltering: " << cloud->width * cloud->height
	//	<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

}

void Filter::statisticalFilter(PointCloudT::Ptr& cloud)
{
	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
  //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
	sor.setInputCloud(cloud);                           //设置待滤波的点云
	sor.setMeanK(50);                               //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值
	sor.filter(*cloud);                    //存储

	//std::cerr << "PointCloud after statisticalfiltering: " << cloud->width * cloud->height
	//	<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
}
