#pragma once
#include <QThread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CaptureThread : public QThread
{
	Q_OBJECT
public:
	CaptureThread();
	~CaptureThread();

	bool enableCapture = false;
	bool enableDetection = false;
	pcl::PointCloud<PointT>::Ptr getCloud();
	

protected:
	void run();

private:
	bool exit = false;
	pcl::PointCloud<PointT>::Ptr cloud;

signals:
	void mySignal();
};

