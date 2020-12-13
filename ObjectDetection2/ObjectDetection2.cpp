#include "ObjectDetection2.h"

ObjectDetection2::ObjectDetection2(QWidget *parent)
    : QMainWindow(parent)
{
    ui->setupUi(this);

	thread->start();
	operation->moveToThread(thread);

	//Initialize the QVTK window
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
	viewer->setBackgroundColor(255, 255, 255);
	ui->qvtkWidget->update();

	connect(ui->actionRun, &QAction::triggered, this, &ObjectDetection2::startCapture);
	connect(ui->actionStop, &QAction::triggered, this, &ObjectDetection2::stopCapture);
	connect(ui->actionStartDetection, &QAction::triggered, this, &ObjectDetection2::startDetection);
	connect(ui->actionStopDetection, &QAction::triggered, this, &ObjectDetection2::stopDetection);


	//刷新图像
	void (Operation:: *operationSignal)() = &Operation::mySignal;
	connect(operation, operationSignal, this, &ObjectDetection2::mySlot);

}

void ObjectDetection2::startCapture()
{
	operation->startCapture();
}
void ObjectDetection2::stopCapture() 
{
	operation->stopCapture();
}

void ObjectDetection2::startDetection()
{
	operation->startDetection();
}
void ObjectDetection2::stopDetection()
{
	operation->stopDetection();
}
void ObjectDetection2::mySlot() 
{
	pcl::PointCloud<PointT>::Ptr cloud = operation->getCloud();

	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	viewer->addPointCloud(cloud, "cloud");
	ui->qvtkWidget->update();
	qDebug() << "刷新窗口" << endl;
}
