#include "CaptureThread.h"
#include "Filter.h"
#include "Detect.h"
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<string>
#include <QDebug>
#include <QTime>

CaptureThread::CaptureThread()
{
	//初始化相机
	initKinect();

	cloud.reset(new pcl::PointCloud<PointT>);
}

CaptureThread::~CaptureThread()
{
	exit = true;
}


void CaptureThread::run()
{
	qDebug() << "等待相机预热 5s" << endl;
	msleep(5000);
	qDebug() << "预热完成" << endl;

	char strfilepath[256] = "cloud5.pcd";
	while (!exit) {
		//msleep(500);
		if (enableCapture == true) {
			
			QTime t;
			t.start();
			if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {
				qDebug() << u8"读取失败" << endl;
				continue;
			}
			//capture();
			qDebug() << u8"读取：" << t.restart() / 1000 << "s" << endl;

			if (enableDetection == true) {
				PointCloudT::Ptr cloud_temp(new PointCloudT);
				pcl::copyPointCloud(*cloud, *cloud_temp);

				//点云预处理
				Filter filter;
				filter.passThroughFilter(cloud_temp);//直通滤波
				filter.voxelGridFilter(cloud_temp);//体素滤波
				filter.statisticalFilter(cloud_temp);//统计滤波

				qDebug() << "预处理：" << t.restart()/1000 << "s" << endl;

				//检测
				Detect detect;
				PointCloudT::Ptr cloud_plane = detect.detectPlain(cloud_temp);//分割地面
				cube.count = detect.detectObject(cloud_temp);//提取物体,count为物体数量
				cube.w_h_d = detect.getWHD();
				cube.rotat = detect.getRotat();// a quaternion-based rotation to apply to the cube
				cube.translat = detect.getTranslat();// a translation to apply to the cube from 0,0,0
				qDebug() << "检测：" << t.restart() / 1000 << "s" << endl;
			}
			else {
				//qDebug() << "不检测" << endl;
			}

			emit mySignal();//提醒主窗口刷新图像	
		}
	}
}

pcl::PointCloud<PointT>::Ptr CaptureThread::getCloud()
{
	return cloud;
}

/// <summary>
/// 采集一帧图像，采集完成后放入cloud中
/// </summary>
void CaptureThread::capture()
{
	pcl::PointCloud<PointT>::Ptr cloudTemp;
	cloudTemp.reset(new pcl::PointCloud<PointT>);

	Mat mColorImg = RGBData();//彩色图分辨率 1920*1080
	Mat mDepthImg = depthData(); //深度图分辨率 512*424

	
	//坐标映射
	pMapper->MapDepthFrameToColorSpace(iDHeight * iDWidth, reinterpret_cast<UINT16*>(mDepthImg.data), iDHeight * iDWidth, depth2rgb);//深度图到颜色的映射
	pMapper->MapDepthFrameToCameraSpace(iDHeight * iDWidth, reinterpret_cast<UINT16*>(mDepthImg.data), iDHeight * iDWidth, depth2xyz);//深度图到相机三维空间的映射


	float maxX = depth2xyz[0].X, maxY = depth2xyz[0].Y, maxZ = depth2xyz[0].Z;
	float minX = depth2xyz[0].X, minY = depth2xyz[0].Y, minZ = depth2xyz[0].Z;
	for (size_t i = 0; i < iDWidth; i++)
	{
		for (size_t j = 0; j < iDHeight; j++)
		{
			//坐标映射后的图像可能会超出0~1920，0~1080，所以需要添加限定条件
			//pcl::PointXYZRGBA pointTemp;
			pcl::PointXYZRGB pointTemp;
			if (depth2xyz[i + j * iDWidth].Z > 0.5 && 
				depth2rgb[i + j * iDWidth].X < 1920 && 
				depth2rgb[i + j * iDWidth].X > 0 && 
				depth2rgb[i + j * iDWidth].Y < 1080 && 
				depth2rgb[i + j * iDWidth].Y > 0)
			{
				pointTemp.x = -depth2xyz[i + j * iDWidth].X;
				if (depth2xyz[i + j * iDWidth].X > maxX) 
					maxX = -depth2xyz[i + j * iDWidth].X;
				if (depth2xyz[i + j * iDWidth].X < minX) 
					minX = -depth2xyz[i + j * iDWidth].X;

				pointTemp.y = depth2xyz[i + j * iDWidth].Y;
				if (depth2xyz[i + j * iDWidth].Y > maxY)
					maxY = depth2xyz[i + j * iDWidth].Y;
				if (depth2xyz[i + j * iDWidth].Y < minY) 
					minY = depth2xyz[i + j * iDWidth].Y;

				pointTemp.z = depth2xyz[i + j * iDWidth].Z;
				if (depth2xyz[i + j * iDWidth].Z != 0.0)
				{
					if (depth2xyz[i + j * iDWidth].Z > maxZ) 
						maxZ = depth2xyz[i + j * iDWidth].Z;
					if (depth2xyz[i + j * iDWidth].Z < minZ) 
						minZ = depth2xyz[i + j * iDWidth].Z;
				}

				pointTemp.b = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[0];
				pointTemp.g = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[1];
				pointTemp.r = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[2];
				pointTemp.a = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[3];
				cloudTemp->push_back(pointTemp);
			}
		}
	}
	pcl::copyPointCloud(*cloudTemp, *cloud);

	if (enableSave == true) 
	{
		static int count = 0;
		count++;


		std::string str2 = ".pcd";
		std::string filename = "cloud" + std::to_string(count) + str2;
		
		//保存
		pcl::io::savePCDFile(filename, *cloud);
		enableSave = false;
	}
}

/// <summary>
/// 初始化kinect
/// </summary>
/// <returns></returns>
bool CaptureThread::initKinect()
{
	if (FAILED(GetDefaultKinectSensor(&pSensor))) return false;
	if (pSensor)
	{
		pSensor->get_CoordinateMapper(&pMapper);
		pSensor->Open();
		qDebug() << "已打开相机" << endl;
		return true;
	}
	else return false;
}

//获取深度帧
Mat CaptureThread::depthData()
{
	IDepthFrameSource* pFrameSource = nullptr;
	pSensor->get_DepthFrameSource(&pFrameSource);
	IDepthFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);
	IDepthFrame* pFrame = nullptr;
	Mat mDepthImg(iDHeight, iDWidth, CV_16UC1);
	while (true)
	{
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			pFrame->CopyFrameDataToArray(iDWidth * iDHeight, reinterpret_cast<UINT16*>(mDepthImg.data));
			qDebug() << "已获取深度帧" << endl;
			pFrame->Release();
			return mDepthImg;
			break;
		}
	}
}

//获取彩色帧
Mat CaptureThread::RGBData()
{
	IColorFrameSource* pFrameSource = nullptr;
	pSensor->get_ColorFrameSource(&pFrameSource);
	IColorFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);
	IColorFrame* pFrame = nullptr;
	Mat mColorImg(iCHeight, iCWidth, CV_8UC4);
	while (true)
	{
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			pFrame->CopyConvertedFrameDataToArray(iCWidth * iCHeight * 4, mColorImg.data, ColorImageFormat_Bgra);
			qDebug() << "已获取彩色帧" << endl;
			pFrame->Release();
			return mColorImg;
			break;
		}
	}
}

