#include "Operation.h"

Operation::Operation() 
{
	initializationData();
	captureThread->start();

	//个人信息填写完成后保存信息并跳转回图像操作界面
	void (CaptureThread:: * CTSignal)() = &CaptureThread::mySignal;
	connect(captureThread, CTSignal, this, &Operation::mySlot);
}
Operation::~Operation()
{
	delete captureThread;
}

void Operation::initializationData() 
{
	cloud.reset(new pcl::PointCloud<PointT>);
}
void Operation::startCapture() 
{
	captureThread->enableCapture = true;
}
void Operation::stopCapture()
{
	captureThread->enableCapture = false;
}
void Operation::startDetection()
{
	captureThread->enableDetection = true;
}
void Operation::stopDetection()
{
	captureThread->enableDetection = false;
}