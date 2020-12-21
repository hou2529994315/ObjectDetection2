#pragma once
#include <QThread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/PolygonMesh.h>
#include <Kinect.h>
#include <opencv2\opencv.hpp>

using namespace cv;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct Cube {
	int count;
	std::vector<std::vector<float>> w_h_d;
	std::vector<Eigen::Quaternionf> rotat;
	std::vector<Eigen::Vector3f> translat;
};

class CaptureThread : public QThread
{
	Q_OBJECT
public:
	CaptureThread();
	~CaptureThread();

	bool enableCapture = false;
	bool enableDetection = false;
	bool enableSave = false;
	

	pcl::PointCloud<PointT>::Ptr getCloud();
	void captureOnePCD() 
	{
		enableSave = true;
	}
	
	Cube getCube() 
	{
		return cube;
	}
private:
	IKinectSensor* pSensor;
	ICoordinateMapper* pMapper;
	const static int iDWidth = 512, iDHeight = 424;//深度图尺寸
	const static int iCWidth = 1920, iCHeight = 1080;//彩色图尺寸
	CameraSpacePoint depth2xyz[iDWidth * iDHeight];
	ColorSpacePoint depth2rgb[iCWidth * iCHeight];

	

	bool exit = false;
	pcl::PointCloud<PointT>::Ptr cloud;
	bool initKinect();
	Cube cube;

	Mat depthData();
	Mat RGBData();
	void capture();
	void run();


signals:
	void mySignal();
};

