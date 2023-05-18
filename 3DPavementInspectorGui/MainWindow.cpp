#include "MainWindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QScrollArea>
#include <QScrollBar>
#include <QDateTime>
#include <QDir>
#include <QTextStream>
#include <QInputDialog>
#include <QXmlStreamWriter>
#include <QStandardPaths>
#include <QDockWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkSmartPointer.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h> 
#include <librealsense2/rs.hpp>
#include <chrono>
#include <thread>   
#include <QTimer>
#include <pcl/visualization/vtk.h>
#include "widgets/NewSolutionDialog.h"

using pcl_ptr_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
pcl_ptr_xyzrgb points_to_pcl_xyzrgb1(const rs2::points& points, const rs2::video_frame& color)
{
    pcl_ptr_xyzrgb cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
    const int color_bytes_per_pixel = color.get_bytes_per_pixel();

    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;

        int index = ptr - points.get_vertices();
        int color_index = index * color_bytes_per_pixel;
        p.r = color_data[color_index];
        p.g = color_data[color_index + 1];
        p.b = color_data[color_index + 2];

        ptr++;
    }

    return cloud;
}

using pcl_ptr_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr;
pcl_ptr_xyz points_to_pcl_xyz1(const rs2::points& points)
{
    pcl_ptr_xyz cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    fileManager(this),
    cameraManager(this),
    projectManager(this),
    pointCloudCameraManager(this)
{
    ui.setupUi(this);
    _setupActionList();
    _setupToolBarAndStatusBar();
    _setupFileManager();
    _setupCameraManager();
    _setupPointCloudCameraManager();

    unableAllActions();
}

MainWindow::~MainWindow()
{
    
}

/*--------------------------------界面初始化----------------------------*/

void MainWindow::_setupActionList()
{
    fileRelatedActions << ui.actionOpen_File
        << ui.actionOpen_Dir
        << ui.action_openCamera,

        detectRelatedActions << ui.actionNext_Image
        << ui.actionZoom_in
        << ui.actionZoom_out
        << ui.actionFit_Window
        << ui.action_run
        << ui.action_pause
        << ui.actionPrevious_Image
        << ui.action_Unload
        << ui.actionLoad,

        allRelatedActions << ui.actionOpen_File
        << ui.actionOpen_Dir
        << ui.actionNext_Image
        << ui.actionZoom_in
        << ui.actionZoom_out
        << ui.actionFit_Window
        << ui.action_run
        << ui.action_pause
        << ui.action_openCamera
        << ui.actionPrevious_Image
        << ui.action_Unload
        << ui.actionLoad;

}

void MainWindow::_setupToolBarAndStatusBar()
{

    ui.mainToolBar->setIconSize(QSize(32, 32));

    ui.actionOpen_Dir->setEnabled(false);
    ui.actionOpen_File->setEnabled(false);

}

void MainWindow::_setupFileManager()
{
    connect(ui.frameListWidget, &QListWidget::itemSelectionChanged, [this]() {
        auto items = ui.frameListWidget->selectedItems();
        if (items.length() == 1) {
            int idx = ui.frameListWidget->row(items[0]);
            if (idx != fileManager.getCurIdx()) {
                bool t = switchFrame(idx);
            }
        }
        });

    // fileListSetup . update ui list
    connect(&fileManager, &FileManager::fileListSetup, [this]() {
        ui.fileListWidget->clear();
        if (fileManager.getMode() == Close) return;
        for (const QString& image : fileManager.allImageFiles()) {
            ui.fileListWidget->addItem(FileManager::getNameWithExtension(image));
        }
        ui.fileListWidget->item(fileManager.getCurIdx())->setSelected(true);
        });

    ui.actionOpen_Dir->setEnabled(false);
    ui.actionOpen_File->setEnabled(false);
}

void MainWindow::_setupCameraManager() {
    connect(&cameraManager, &CameraManager::newImage, this, &MainWindow::updateImage);
    connect(&cameraManager, &CameraManager::clearPlayer, this, &MainWindow::clearPlayer);
}

void MainWindow::_setupPointCloudCameraManager() {
    connect(&pointCloudCameraManager, &PointCloudCameraManager::newPointCloud, this, &MainWindow::updatePonitCloudFrame);
    //connect(&pointCloudCameraManager, &PointCloudCameraManager::clearPlayer, this, &MainWindow::clearPlayer);
}

/*--------------------------------界面初始化----------------------------*/

/*--------------------------------action槽函数----------------------------*/

void MainWindow::on_action_newSln_triggered()
{
    // 创建文件夹选择框
    QString dir = QFileDialog::getExistingDirectory(this, tr("选择目录"), "");

    NewSolutionDialog dialog(this);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    QString solutionName = dialog.getSolutionName();
    QString projectName = dialog.getProjectName();

    // 创建解决方案文件夹
    QString solutionDir = dir + "/" + solutionName;
    QDir().mkdir(solutionDir);

    // 创建解决方案文件
    QFile solutionFile(solutionDir + "/" + solutionName + ".solution");
    solutionFile.open(QIODevice::WriteOnly);
    QTextStream out_solution(&solutionFile);
    out_solution << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << Qt::endl;
    out_solution << "<solution>" << Qt::endl;
    out_solution << "  <name>" << solutionName << "</name>" << Qt::endl;
    out_solution << "  <created>" << QDateTime::currentDateTime().toString(Qt::ISODate) << "</created>" << Qt::endl;
    out_solution << "</solution>" << Qt::endl;
    solutionFile.close();

    // 创建项目文件夹
    QString projectDir = solutionDir + "/" + projectName;
    QDir().mkdir(projectDir);

    // 创建项目文件
    QFile projectFile(projectDir + "/" + projectName + ".proj");
    projectFile.open(QIODevice::WriteOnly);
    QTextStream out_project(&projectFile);
    out_project << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << Qt::endl;
    out_project << "<project>" << Qt::endl;
    out_project << "  <name>" << projectName << "</name>" << Qt::endl;
    out_project << "  <created>" << QDateTime::currentDateTime().toString(Qt::ISODate) << "</created>" << Qt::endl;
    out_project << "</project>" << Qt::endl;
    projectFile.close();

    // 创建 output 文件夹
    QDir().mkdir(projectDir + "/output");

    QMessageBox::information(this, tr("解决方案创建成功"), tr("解决方案已创建在 %1").arg(solutionDir));

    projectManager.setProjectPath(projectDir);
    projectManager.setOutputPath(projectDir + "/output");

    enableFileActions();

    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date_time_str = current_date_time.toString("hh:mm:ss");
    ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已创建解决方案 " + solutionName);
}

void MainWindow::on_action_newProj_triggered()
{
    // 弹出文件夹选择框
    QString path = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), "");

    if (!path.isEmpty()) {
        // 弹出输入文件名对话框
        bool ok;
        QString projectName = QInputDialog::getText(this, tr("输入项目名称"), tr("项目名称:"), QLineEdit::Normal, "", &ok);

        if (ok && !projectName.isEmpty()) {
            // 创建项目文件夹
            QString folderPath = path + "/" + projectName;
            QDir folderDir(folderPath);

            if (!folderDir.exists()) {
                if (!folderDir.mkdir(folderPath)) {
                    QMessageBox::warning(this, tr("创建文件夹失败"), tr("无法在该位置创建文件夹"));
                    return;
                }
            }

            // 在项目文件夹中创建output文件夹
            QString outputPath = folderPath + "/output";
            QDir outputDir(outputPath);

            if (!outputDir.exists()) {
                if (!outputDir.mkdir(outputPath)) {
                    QMessageBox::warning(this, tr("创建output文件夹失败"), tr("无法在该位置创建output文件夹"));
                    return;
                }
            }

            // 创建项目文件
            QString projectFilePath = folderPath + "/" + projectName + ".proj";
            QFile projectFile(projectFilePath);

            if (!projectFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QMessageBox::warning(this, tr("创建项目文件失败"), tr("无法在该位置创建项目文件"));
                return;
            }

            // 写入项目文件内容
            QTextStream out(&projectFile);
            out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << Qt::endl;
            out << "<project>" << Qt::endl;
            out << "  <name>" << projectName << "</name>" << Qt::endl;
            out << "  <created>" << QDateTime::currentDateTime().toString(Qt::ISODate) << "</created>" << Qt::endl;
            out << "</project>" << Qt::endl;

            projectFile.close();
            QMessageBox::information(this, tr("项目创建成功"), tr("项目已创建在 %1").arg(folderPath));
            projectManager.setProjectPath(folderPath);
            projectManager.setOutputPath(outputPath);

            enableFileActions();

            QDateTime current_date_time = QDateTime::currentDateTime();
            QString current_date_time_str = current_date_time.toString("hh:mm:ss");
            ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已创建新项目 " + projectName);
        }
    }
}

void MainWindow::on_action_openProj_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open Project", "", "Project Files (*.proj)");

    if (!fileName.isEmpty()) {
        // do something with the selected file
        projectManager.setProjectPath(fileName);
        projectManager.setOutputPath(fileName + "/output");
        enableFileActions();

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已加载项目" + QString(fileName).section('/', -1));
    }

}

void MainWindow::on_actionOpen_File_triggered()
{
    if (fileManager.getMode() != Close) {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Unload Dectect", "确定要打开新文件吗?（打开新文件将关闭当前检测）",
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No)
            return;
        on_actionUnload_withoutAsk();
    }

    QString fileName = QFileDialog::getOpenFileName(this, "Open a file", "/",
        "Files ( *.mp4);");

    if (!fileName.isNull() && !fileName.isEmpty()) {
        enableDetectActions();
        fileManager.setSingleVideo(fileName);
        QString title = QString("%1").arg(fileName);
        ui.playerDockWidget->setWindowTitle(title);

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已加载离线视频");
    }
}

void MainWindow::on_actionOpen_Dir_triggered()
{
    if (fileManager.getMode() != Close) {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Unload Detect", "确定要打开新文件吗?（打开新文件将关闭当前检测）",
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No)
            return;
        on_actionUnload_withoutAsk();
    }
    QString dirName = QFileDialog::getExistingDirectory(this, "open a dir", "/");
    if (!dirName.isNull() && !dirName.isEmpty()) {
        enableDetectActions();

        QDir dir(dirName);
        QStringList images = dir.entryList(QStringList() << "*.jpg" << "*.png", QDir::Files);
        images.sort();
        if (!dirName.endsWith('/')) dirName += "/";
        QStringList tmp;
        for (auto& image : images) {
            tmp.push_back(dirName + image);
        }
        images = tmp;
        fileManager.setMultiImage(images);
        QString title = QString("%1").arg(dirName);
        ui.playerDockWidget->setWindowTitle(title);

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已加载图片序列");
    }
}

void MainWindow::on_action_run_triggered()
{
    if (!timer.isActive()) {
        if (!capture.isOpened()) {
            if (fileManager.count() > 0) {
                QString currentFile = fileManager.getCurrentImageFile();
                if (fileManager.getMode() == Video) {
                    // If the current file is a video, open it using cv::VideoCapture
                    capture.open(currentFile.toStdString());
                }
                else {
                    // If the current file is a folder, play all image files as a video
                    QStringList imageFiles = fileManager.allImageFiles();
                    if (imageFiles.length() > 0) {
                        std::vector<cv::Mat> frames;
                        for (const QString& fileName : imageFiles) {
                            cv::Mat frame = cv::imread(fileName.toStdString());
                            if (frame.empty()) {
                                // Skip empty frames
                                continue;
                            }
                            frames.push_back(frame);
                        }
                        if (frames.size() > 0) {
                            // Create a video writer and write the frames as a video
                            cv::VideoWriter writer;
                            QString outputPath = fileManager.getDir(currentFile) + "output.avi";
                            int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
                            writer.open(outputPath.toStdString(), codec, 30, frames[0].size());
                            if (writer.isOpened()) {
                                for (const cv::Mat& frame : frames) {
                                    writer.write(frame);
                                }
                                writer.release();
                                capture.open(outputPath.toStdString());
                            }
                            else {
                                qWarning() << "Failed to open video writer for" << outputPath;
                            }
                        }
                    }
                }
            }
        }
        connect(&timer, SIGNAL(timeout()), this, SLOT(update_frame()));

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已开始检测离线序列");

        timer.start(33); // 设置30fps的帧率
    }
}

void MainWindow::on_action_pause_triggered()
{
    if (timer.isActive()) {
        timer.stop(); // stop the timer to pause the video playback

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已暂停检测");
    }
}

void MainWindow::on_actionNext_Image_triggered()
{
    if (capture.isOpened()) {
        // Move capture to the next frame
        //capture.grab();
        update_frame();
    }
}

void MainWindow::on_actionPrevious_Image_triggered()
{
    if (capture.isOpened()) {
        // Get the current frame number
        int frame_number = capture.get(cv::CAP_PROP_POS_FRAMES);
        // Move capture to the previous frame
        capture.set(cv::CAP_PROP_POS_FRAMES, frame_number - 2);
        // Update the displayed frame
        update_frame();
    }
}

void MainWindow::on_actionUnload_triggered()
{
    if (fileManager.getMode() == Close) return;

    // 弹出确认卸载项目选项框
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Unload Project", "确定要关闭当前检测吗?",
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No)
        return;

    // 停止定时器和update_frame线程
    timer.stop();
    disconnect(&timer, SIGNAL(timeout()), this, SLOT(update_frame()));

    // 清空图像
    ui.image_player->clear();

    // 释放capture对象
    if (capture.isOpened()) {
        capture.release();
    }

    ui.playerDockWidget->setWindowTitle("");
    ui.frameListWidget->clear();
    fileManager.close();

    /*unableActions()*/
    unableDetectActions();

    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date_time_str = current_date_time.toString("hh:mm:ss");
    ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已关闭检测");
}

void MainWindow::on_actionUnload_withoutAsk() {
    if (fileManager.getMode() == Close) return;

    // 停止定时器和update_frame线程
    timer.stop();
    disconnect(&timer, SIGNAL(timeout()), this, SLOT(update_frame()));

    // 清空图像
    ui.image_player->clear();

    // 释放capture对象
    if (capture.isOpened()) {
        capture.release();
    }

    ui.playerDockWidget->setWindowTitle("");
    fileManager.close();
    ui.frameListWidget->clear();
    unableDetectActions();

    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date_time_str = current_date_time.toString("hh:mm:ss");
    ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已关闭离线检测");
}

void MainWindow::on_action_Unload_triggered()
{
    if (fileManager.getMode() == Close) return;

    // 弹出确认卸载项目选项框
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Unload Project", "确定要关闭当前检测吗?",
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No)
        return;

    // 停止定时器和update_frame线程
    timer.stop();
    disconnect(&timer, SIGNAL(timeout()), this, SLOT(update_frame()));

    // 清空图像
    ui.image_player->clear();

    // 释放capture对象
    if (capture.isOpened()) {
        capture.release();
    }

    ui.playerDockWidget->setWindowTitle("");
    ui.frameListWidget->clear();
    fileManager.close();

    /*unableActions()*/
    unableDetectActions();

    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date_time_str = current_date_time.toString("hh:mm:ss");
    ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已关闭检测");
}

void MainWindow::on_action_openCamera_triggered()
{
    if (cameraManager.getMode() == cameraOpen) {
        cameraManager.closeCamera();

        //@TODO 临时写法，待改进
        ui.cameraListWidget->clear();
        //

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已关闭深度相机");

        return;
    }
    cameraManager.getCamera();
    if (cameraManager.getMode() != cameraClose) {
        return;
    }
    else {
        on_actionUnload_triggered();
        cameraManager.openCamera();

        //@TODO 临时写法，待改进
        QListWidgetItem* item = new QListWidgetItem(QString("intel RealSense D415"));
        ui.cameraListWidget->addItem(item);

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已打开深度相机实时检测");
    }
}

void MainWindow::on_actionAbout_triggered()
{
    QString about_text;
    about_text = "<p>Auther : XIAO HAN</p>";
    about_text += "<p>Date : 2023.04</p>";
    about_text += "<p>To get more information and tutorial, please refer to the ";
    about_text += "<a href='https://github.com/XanHiaoo'>Documentation</a></p>";
    QMessageBox::about(this, "About", about_text);
}

void MainWindow::on_action_pointcloud_detect_triggered()
{
    if (pointCloudCameraManager.getMode() == PointCloudcameraOpen) {
        pointCloudCameraManager.closeCamera();

        //@TODO 临时写法，待改进
        ui.cameraListWidget->clear();
        //

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已关闭三维相机");

        return;
    }
    pointCloudCameraManager.getCamera();
    if (pointCloudCameraManager.getMode() != PointCloudcameraClose) {
        return;
    }
    else {
        on_actionUnload_triggered();
        

        //@TODO 临时写法，待改进
        QListWidgetItem* item = new QListWidgetItem(QString("intel RealSense D415"));
        ui.cameraListWidget->addItem(item);

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date_time_str = current_date_time.toString("hh:mm:ss");
        ui.TipsplainTextEdit->appendPlainText(current_date_time_str + "已打开深度相机实时检测");

        // 隐藏 QLabel image_player
        ui.image_player->hide();

        // 创建一个新的 QVTKOpenGLNativeWidget
        qvtkWidget = new QVTKOpenGLNativeWidget;

        // 将新窗口部件的大小和位置设置为与 QLabel image_player 相匹配
        qvtkWidget->setGeometry(ui.image_player->geometry());

        // 用新的 QVTKOpenGLNativeWidget 替换 QLabel image_player
        ui.dockWidgetContents_3->layout()->addWidget(qvtkWidget);

        vtkSmartPointer<vtkOpenGLRenderer> ren = vtkSmartPointer<vtkOpenGLRenderer>::New();
        vtkSmartPointer< vtkGenericOpenGLRenderWindow> renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        renWin->AddRenderer(ren);
        qvtkWidget->setRenderWindow(renWin);
        qvtkWidget->paintingActive();
        viewer_.reset(new pcl::visualization::PCLVisualizer(ren, renWin, "3D Viewer", false));
        viewer_->setBackgroundColor(1.0, 1.0, 1.0);
        viewer_->setCameraPosition(0, 0, -2, 0, 0, 0);
        viewer_->setupInteractor(qvtkWidget->interactor(), qvtkWidget->renderWindow());
        pointCloudCameraManager.openCamera();
    }
    
}

//void MainWindow::on_action_pointcloud_detect_triggered()
//{
//
//    // 隐藏 QLabel image_player
//    ui.image_player->hide();
//
//    // 创建一个新的 QVTKOpenGLNativeWidget
//    qvtkWidget = new QVTKOpenGLNativeWidget;
//
//    // 将新窗口部件的大小和位置设置为与 QLabel image_player 相匹配
//    qvtkWidget->setGeometry(ui.image_player->geometry());
//
//    // 用新的 QVTKOpenGLNativeWidget 替换 QLabel image_player
//    ui.dockWidgetContents_3->layout()->addWidget(qvtkWidget);
//
//    vtkSmartPointer<vtkOpenGLRenderer> ren = vtkSmartPointer<vtkOpenGLRenderer>::New();
//    vtkSmartPointer< vtkGenericOpenGLRenderWindow> renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
//    renWin->AddRenderer(ren);
//    qvtkWidget->setRenderWindow(renWin);
//    qvtkWidget->paintingActive();
//    viewer_.reset(new pcl::visualization::PCLVisualizer(ren, renWin, "3D Viewer", false));
//    viewer_->setBackgroundColor(1.0, 1.5, 1.0);
//    viewer_->setCameraPosition(0, 0, -2, 0, 0, 0);
//    viewer_->setupInteractor(qvtkWidget->interactor(), qvtkWidget->renderWindow());
//
//    timer_ = new QTimer(this);
//    timer_->setInterval(33); // 50ms
//    connect(timer_, &QTimer::timeout, this, &MainWindow::onTimerTimeout);
//    pipe.start();
//    timer_->start();
//}
/*--------------------------------action槽函数----------------------------*/

/*--------------------------------界面功能实现函数----------------------------*/

void MainWindow::update_frame()
{
    cv::Mat frame;
    capture >> frame;
    if (frame.empty())
        return;

    // 在每一帧的中间画一个100x100的方框
    cv::Mat modified_frame = frame.clone();
    cv::rectangle(modified_frame, cv::Point(modified_frame.cols / 2 - 50, modified_frame.rows / 2 - 50), cv::Point(modified_frame.cols / 2 + 50, modified_frame.rows / 2 + 50), cv::Scalar(0, 255, 0), 10);

    // 将修改后的帧与原始帧拼合在一起
    cv::Mat new_frame(frame.rows, frame.cols * 2, frame.type());
    frame.copyTo(new_frame(cv::Rect(0, 0, frame.cols, frame.rows)));
    modified_frame.copyTo(new_frame(cv::Rect(frame.cols, 0, frame.cols, frame.rows)));

    // 转换图像格式
    cv::cvtColor(new_frame, new_frame, cv::COLOR_BGR2RGB);
    qt_image = QImage(new_frame.data, new_frame.cols, new_frame.rows, QImage::Format_RGB888);

    // 设置图像
    ui.image_player->setPixmap(QPixmap::fromImage(qt_image));

    // 打印当前播放的帧数
    int frame_number = capture.get(cv::CAP_PROP_POS_FRAMES);
    // 更新自定义列表小部件以显示当前帧数
    QListWidgetItem* item = new QListWidgetItem(QString("Frame %1").arg(frame_number));;

    ui.frameListWidget->addItem(item);
    qDebug() << "Playing frame: " << frame_number;
}

bool MainWindow::switchFrame(int idx) {
    // Set the current frame number to the index of the selected item
    capture.set(cv::CAP_PROP_POS_FRAMES, idx);

    // Call the update_frame() function to display the selected frame
    update_frame();

    return true;
}

void MainWindow::updateImage(const cv::Mat& image1, const cv::Mat& image2)
{

    // 在每一帧的中间画一个100x100的方框
    cv::Mat modified_frame = image1.clone();
    cv::rectangle(modified_frame, cv::Point(modified_frame.cols / 2 - 50, modified_frame.rows / 2 - 50), cv::Point(modified_frame.cols / 2 + 50, modified_frame.rows / 2 + 50), cv::Scalar(0, 255, 0), 10);

    // 将修改后的帧与原始帧拼合在一起
    cv::Mat new_frame(image1.rows, image1.cols * 2, image1.type());
    image1.copyTo(new_frame(cv::Rect(0, 0, image1.cols, image1.rows)));
    image2.copyTo(new_frame(cv::Rect(image1.cols, 0, image1.cols, image1.rows)));

    // 转换图像格式
    cv::cvtColor(new_frame, new_frame, cv::COLOR_BGR2RGB);
    qt_image = QImage(new_frame.data, new_frame.cols, new_frame.rows, QImage::Format_RGB888);
    // Set the image on the label
    ui.image_player->setPixmap(QPixmap::fromImage(qt_image));
}

void MainWindow::updatePonitCloudFrame( const DetectionResult detectionResult, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb)
{
    auto start_time = std::chrono::steady_clock::now();
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();

    int v1(0);
    viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer_->setBackgroundColor(1.0, 1.0, 1.0, v1);
    viewer_->addPointCloud(cloud_xyzrgb, "cloud_v1", v1);
    int v2(0);
    viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer_->setBackgroundColor(1.0, 1.0, 1.0, v2);
    viewer_->addPointCloud(cloud_xyzrgb, "cloud_v2", v2);

    if (detectionResult.code == DETECTION_NG) {
        // Add each point cloud in bulge to the viewer with a green color
        for (size_t i = 0; i < detectionResult.bulge.size(); ++i)
        {
            std::string cloud_name = "bulge_" + std::to_string(i);
            viewer_->addPointCloud<pcl::PointXYZ>(detectionResult.bulge[i], cloud_name, v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, cloud_name);
            viewer_->addText3D("bulge" + std::to_string(i) + " " + std::to_string(detectionResult.bulgeArea[i]).substr(0, 5),
                pcl::PointXYZ(detectionResult.bulge[i]->points[0].x, detectionResult.bulge[i]->points[0].y,
                    detectionResult.bulge[i]->points[0].z + 0.1), 0.02, 0.0, 0.0, 1.0, "text_bulge" + std::to_string(i), v2);

        }

        // Add each point cloud in hole to the viewer with a red color
        for (size_t i = 0; i < detectionResult.hole.size(); ++i)
        {
            std::string cloud_name = "hole_" + std::to_string(i);
            viewer_->addPointCloud<pcl::PointXYZ>(detectionResult.hole[i], cloud_name, v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cloud_name);
            viewer_->addText3D("hole" + std::to_string(i) + " " + std::to_string(detectionResult.holeArea[i]).substr(0, 5),
                pcl::PointXYZ(detectionResult.hole[i]->points[0].x, detectionResult.hole[i]->points[0].y,
                    detectionResult.hole[i]->points[0].z + 0.1), 0.02, 0.0, 0.0, 1.0, "text_hole" + std::to_string(i), v2);
        }
    }
    viewer_->spin();
    auto end_time = std::chrono::steady_clock::now();
    qDebug() << "渲染执行时间: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms";
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    // Stop the image processing loop when the window is closed
    cameraManager.setStopProcessing(true);
    if (cameraManager.getImageProcessingLoop() && cameraManager.getImageProcessingLoop()->joinable()) {
        cameraManager.getImageProcessingLoop()->join();
    }
}

void MainWindow::clearPlayer()
{
    ui.image_player->clear();
}

/*--------------------------------界面功能实现函数----------------------------*/

/*--------------------------------action可见性----------------------------*/

void MainWindow::enableAllActions()
{
    for (auto action : allRelatedActions)
        action->setEnabled(true);
}

void MainWindow::unableAllActions()
{
    for (auto action : allRelatedActions)
        action->setEnabled(false);
}

void MainWindow::enableDetectActions()
{
    for (auto action : detectRelatedActions)
        action->setEnabled(true);
}

void MainWindow::unableDetectActions()
{
    for (auto action : detectRelatedActions)
        action->setEnabled(false);
}

void MainWindow::enableFileActions()
{
    for (auto action : fileRelatedActions)
        action->setEnabled(true);
}

void MainWindow::unableFileActions()
{
    for (auto action : fileRelatedActions)
        action->setEnabled(false);
}

/*--------------------------------action可见性----------------------------*/
void MainWindow::onTimerTimeout() {
    PavementDetector* detector = new PavementDetector();
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();
    points = pc.calculate(depth);
    auto cloud_xyzrgb = points_to_pcl_xyzrgb1(points, color);
    auto cloud_xyz = points_to_pcl_xyz1(points);
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();

    int v1(0);
    viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer_->setBackgroundColor(1.0, 1.0, 1.0, v1);
    viewer_->addPointCloud(cloud_xyzrgb, "cloud_v1", v1);
    //viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,0.0, "cloud");
    detector->detect(cloud_xyz);
    DetectionResult detectionResult = detector->getResult();

    int v2(0);
    viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer_->setBackgroundColor(1.0, 1.0, 1.0, v2);
    viewer_->addPointCloud(cloud_xyzrgb, "cloud_v2", v2);

    if (detectionResult.code == DETECTION_NG) {
        // Add each point cloud in bulge to the viewer with a green color
        for (size_t i = 0; i < detectionResult.bulge.size(); ++i)
        {
            std::string cloud_name = "bulge_" + std::to_string(i);
            viewer_->addPointCloud<pcl::PointXYZ>(detectionResult.bulge[i], cloud_name, v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, cloud_name);
            viewer_->addText3D("bulge" + std::to_string(i) + " " + std::to_string(detectionResult.bulgeArea[i]).substr(0, 5),
                pcl::PointXYZ(detectionResult.bulge[i]->points[0].x, detectionResult.bulge[i]->points[0].y,
                    detectionResult.bulge[i]->points[0].z + 0.1), 0.02, 0.0, 0.0, 1.0, "text_bulge" + std::to_string(i), v2);

        }

        // Add each point cloud in hole to the viewer with a red color
        for (size_t i = 0; i < detectionResult.hole.size(); ++i)
        {
            std::string cloud_name = "hole_" + std::to_string(i);
            viewer_->addPointCloud<pcl::PointXYZ>(detectionResult.hole[i], cloud_name, v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cloud_name);
            viewer_->addText3D("hole" + std::to_string(i) + " " + std::to_string(detectionResult.holeArea[i]).substr(0, 5),
                pcl::PointXYZ(detectionResult.hole[i]->points[0].x, detectionResult.hole[i]->points[0].y,
                    detectionResult.hole[i]->points[0].z + 0.1), 0.02, 0.0, 0.0, 1.0, "text_hole" + std::to_string(i), v2);
        }
    }
    viewer_->spin();
}



