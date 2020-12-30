#include "Detect.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>//按索引提取点云
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

PointCloudT::Ptr Detect::detectPlain(PointCloudT::Ptr& cloud)
{
	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //记录点云的序号

	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// 创建一个分割器
	seg.setInputCloud(cloud);
	seg.setModelType(pcl::SACMODEL_PLANE);//设置目标几何形状
	seg.setMethodType(pcl::SAC_RANSAC); //分割方法：随机采样法
	seg.setDistanceThreshold(0.05); //设置误差阈值
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}

	//按照索引提取点云　　内点
	pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;//索引提取器
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());

	extract_indices.setInputCloud(cloud);//设置输入点云
	extract_indices.setIndices(boost::make_shared<const pcl::PointIndices>(*inliers));//设置索引
	extract_indices.filter(*cloud_plane);//提取对于索引的点云 内点
	std::cerr << "plane point size : " << cloud_plane->points.size() << std::endl;

	// Remove the planar inliers, extract the rest
	extract_indices.setNegative(true);
	extract_indices.filter(*cloud);
	std::cerr << "rest point size : " << cloud->points.size() << std::endl;

	return cloud_plane;
}

//欧式聚类进行检测，并生成最小包围盒
int Detect::detectObject(PointCloudT::Ptr& cloud)
{
	// 使用kdtree提取物体
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.02); //设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize(100);    //设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize(25000);  //设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);     //设置点云的搜索机制
	ec.setInputCloud(cloud); //设置原始点云 
	ec.extract(cluster_indices);  //从点云中提取聚类


	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		//按照索引提取点云　内点
		pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;//索引提取器
		extract_indices.setInputCloud(cloud);//设置输入点云
		extract_indices.setIndices(boost::make_shared<const pcl::PointIndices>(*it));//设置索引
		extract_indices.filter(*cloud_cluster);//提取对于索引的点云 内点
		bbox(cloud_cluster);
		//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
	}
	return cluster_indices.size();
}

/// <summary>
/// 计算输入点云的最小包围盒
/// </summary>
/// <param name="cloud"></param>
void Detect::bbox(PointCloudT::Ptr cloud)
{

	// 计算一组点的三维（X-Y-Z）质心并将其作为三维向量返回
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);

	// 计算一组给定点的3x3协方差矩阵
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);

	// 计算协方差矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f pcaEigenVectors = eigen_solver.eigenvectors();	// 协方差矩阵特征向量
	Eigen::Vector3f pcaEigenValues = eigen_solver.eigenvalues();	// 协方差矩阵特征值

	/* 协方差矩阵的特征向量表示OBB包围盒的方向。大的特征值对应大的方差，
	所以应该让OBB包围盒沿着最大特征值对应的特征向量的方向。*/
	pcaEigenVectors.col(2) = pcaEigenVectors.col(0).cross(pcaEigenVectors.col(1)); //校正主方向间垂直
	pcaEigenVectors.col(0) = pcaEigenVectors.col(1).cross(pcaEigenVectors.col(2));
	pcaEigenVectors.col(1) = pcaEigenVectors.col(2).cross(pcaEigenVectors.col(0));

	// 打印信息
	//std::cout << "质心点:\n" << pcaCentroid << std::endl;
	//std::cout << "协方差矩阵特征值:\n" << pcaEigenValues << std::endl;
	//std::cout << "协方差矩阵特征向量:\n" << pcaEigenVectors << std::endl;

	// 计算出来的特征向量进行正交标准化
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();	   // 返回单位矩阵的表达式
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();  // 返回单位矩阵逆矩阵的表达式
	tm.block<3, 3>(0, 0) = pcaEigenVectors.transpose();    // 特征向量的转置矩阵
	tm.block<3, 1>(0, 3) = -1.0f * (pcaEigenVectors.transpose()) * (pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	// 打印信息
	//std::cout << "单位变换矩阵tm(4x4):\n" << tm << std::endl;
	//std::cout << "单位逆变矩阵tm_inv(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);	// 应用由矩阵tm定义的刚性变换,即正交标准化

	// 将各点的坐标投影到计算出的坐标轴上，求出中心和半长度
	pcl::PointXYZRGB min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());
	Eigen::Affine3f tm_inv_aff(tm_inv);	// 单位逆变矩阵变换，用作逆变换回输入点云的坐标系
	pcl::transformPoint(c1, c, tm_inv_aff);	// 恢复初始点云的包围盒型心
	// 打印信息
	//std::cout << "型心c1(3x1):\n" << c1 << std::endl;
	//std::cout << "型心c(3x1):\n" << c << std::endl;

	/*
	输入点云转换至远点后,求得变换后点云的最大最小x,y,z轴的坐标,
	此时(max.x,max.y,max.z),(max.x,min.y,max.z),(max.x,max.y,min.z),(min.x,max.y,max.z),
	(min.x,max.y,min.z),(min.x,min.y,max.z),(min.x,min.y,max.z),(min.x,min.y,min.z)
	即为变换后点云的包围盒,也是原始输入点云包围盒顶点坐标经过变化后的坐标
	*/
	Eigen::Vector3f whd, whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  // 后续画图时用作画坐标轴尺度参数
	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));// a quaternion-based rotation to apply to the cube
	const Eigen::Vector3f    bboxT(c);		// a translation to apply to the cube from 0,0,0
	rotat.push_back(bboxQ);
	translat.push_back(bboxT);
	std::vector<float> temp;
	temp.push_back(whd(0));
	temp.push_back(whd(1));
	temp.push_back(whd(2));
	w_h_d.push_back(temp);

	// 包围盒所需参数：viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");

}