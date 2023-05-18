#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <QTimer>
#include <opencv2/opencv.hpp>
#include "controllers/filemanager.h"
#include "controllers/CameraManager.h"
#include "controllers/ProjectManager.h"
#include "controllers/PointCloudCameraManager.h"
#include <librealsense2/rs.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKOpenGLNativeWidget.h>
#include "../3DPavementDetectorDll/PavementDetector.h"


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event) override;

signals:
    void newImage(const cv::Mat& image);

private slots:
    void updateImage(const cv::Mat& image1, const cv::Mat& image2);
    
    //void updatePonitCloudFrame(DetectionResult detectionResult,  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb);
    
    void clearPlayer();

    void update_frame();

    void on_actionOpen_File_triggered();
    void on_actionOpen_Dir_triggered();
    void on_actionAbout_triggered();

    /*��ͼ��ɼ��Խ���ͳһ����*/
    void enableAllActions();
    void unableAllActions();
    void enableFileActions();
    void unableFileActions();
    void enableDetectActions();
    void unableDetectActions();
    /*��ͼ��ɼ��Խ���ͳһ����*/

    /*action�ۺ���*/
    void on_action_run_triggered();
    void on_action_pause_triggered();
    void on_actionNext_Image_triggered();
    void on_actionPrevious_Image_triggered();
    void on_actionUnload_triggered();
    void on_action_openCamera_triggered();
    void on_action_newProj_triggered();
    void on_action_newSln_triggered();
    void on_action_openProj_triggered();
    void on_actionAuto_Save_triggered();
    void on_action_openOutputDir_triggered();
    void on_action_pointcloud_detect_triggered();
    /*action�ۺ���*/

    void on_action_Unload_triggered();

private:
    Ui::MainWindow ui;
    cv::VideoCapture capture;
    int current_frame = 0;
    QTimer timer;
    QImage qt_image;

    FileManager fileManager;
    CameraManager cameraManager;
    ProjectManager projectManager;
    PointCloudCameraManager pointCloudCameraManager;

    QList<QAction*> allRelatedActions;
    QList<QAction*> fileRelatedActions;
    QList<QAction*> detectRelatedActions;

    rs2::pipeline pipe_;
    rs2::pointcloud pc_;
    rs2::points points_;
    QVTKOpenGLNativeWidget* qvtkWidget;
    QTimer* pointCloudDetectTimer_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;

    bool autoSavePointCloudDetect = false;

    int ngNum = 0;

private:
    void _setupActionList();
    void _setupToolBarAndStatusBar();
    void _setupFileManager();
    void _setupCameraManager();
    void _setupPointCloudCameraManager();

    //ж�ص�ǰ�ļ��
    void on_actionUnload_withoutAsk();

    void onDetectTimerTimeout();
    
    // �л������idx��Ӧ��֡
    bool switchFrame(int idx);

};

