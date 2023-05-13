#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKOpenGLNativeWidget.h>
#include "../3DPavementDetectorDll/PavementDetector.h"
#include <librealsense2/rs.hpp>

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget* parent = nullptr);
	~MainWindow();
	void onTimerTimeout();
	

private:
	Ui::MainWindowClass ui;
	PavementDetector* detector;
	rs2::pipeline pipe;
	rs2::pointcloud pc;
	rs2::points points;
	QVTKOpenGLNativeWidget* qvtkWidget;
	QTimer* timer_;
	pcl::visualization::PCLVisualizer::Ptr viewer_;
	

private slots:
	void on_pushButton_triggered();
	//void updateViewer();
};