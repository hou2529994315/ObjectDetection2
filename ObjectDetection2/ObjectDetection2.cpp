#include "ObjectDetection2.h"

ObjectDetection2::ObjectDetection2(QWidget *parent)
    : QMainWindow(parent)
{
    ui->setupUi(this);

	//thread->start();
	//operation->moveToThread(thread);

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
	connect(ui->actionSave, &QAction::triggered, this, &ObjectDetection2::saveOne);


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

	Cube cube = operation->getCube();
	for (int i = 0; i < cube.count; i++)
	{
		std::stringstream ss;
		ss << "cloud_cluster_" << i;
		viewer->addCube(cube.translat[i], cube.rotat[i], cube.w_h_d[i][0], cube.w_h_d[i][1], cube.w_h_d[i][2], ss.str());
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, ss.str());
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());
	}


	ui->qvtkWidget->update();
	//qDebug() << "刷新窗口" << endl;
}

void ObjectDetection2::saveOne() 
{
	operation->saveOne();
}