#include "CaptureThread.h"

#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。

#include <QDebug>

CaptureThread::CaptureThread()
{
	cloud.reset(new pcl::PointCloud<PointT>);
}

CaptureThread::~CaptureThread()
{
	exit = true;
}

void CaptureThread::run()
{
	qDebug() << "执行线程" << endl;
	char strfilepath[256] = "rabbit.pcd";
	while (!exit) {
		msleep(500);
		if (enableCapture == true) {
			
			if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {
				qDebug() << u8"读取失败" << endl;
				continue;
			}


			if (enableDetection == true) {
				qDebug() << u8"检测" << endl;
			}
			else {
				qDebug() << "不检测" << endl;
			}

			emit mySignal();//提醒主窗口刷新图像
			
		}
	}
}

pcl::PointCloud<PointT>::Ptr CaptureThread::getCloud()
{
	return cloud;
}