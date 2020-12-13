#pragma once
//QT
#include <QString>
#include <QStack>
#include <QQueue>
#include <QObject>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/PolygonMesh.h>

//VTK
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <iostream>  
#include <QThread>



#include <QObject>
#include <QDebug>
#include "CaptureThread.h"
class Operation : public QObject
{
	Q_OBJECT
private:
	pcl::PointCloud<PointT>::Ptr cloud;

	void initializationData();

	CaptureThread *captureThread = new CaptureThread();

public:
	Operation();
	~Operation();

	
	void startCapture();
	void stopCapture();
	void startDetection();
	void stopDetection();
	pcl::PointCloud<PointT>::Ptr getCloud()
	{
		return cloud;
	}
private slots:
	void mySlot() {
		cloud = captureThread->getCloud();
		emit mySignal();
	}

signals:
	void mySignal();

};

