#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_ObjectDetection2.h"

//QT
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QProgressBar>
#include <QTreeWidgetItem>
#include <QTextCodec>
#include <QFileDialog>

//PCL
#include <pcl/polygonmesh.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

//VTK
#include <vtkRenderWindow.h>
#include <vtkAutoInit.h> 
//VTK_MODULE_INIT(vtkRenderingVolumeOpenGL);
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include "Operation.h"

class ObjectDetection2 : public QMainWindow
{
    Q_OBJECT

public:
    ObjectDetection2(QWidget *parent = Q_NULLPTR);


private:
    Ui::ObjectDetection2Class *ui = new Ui::ObjectDetection2Class();
    Operation *operation = new Operation();

    QThread *thread = new QThread;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    QAction* openFileTool;
    QAction* exitTool;

private slots:
    void startCapture();
    void stopCapture();
    void startDetection();
    void stopDetection();
    void saveOne();

    void mySlot();

};
