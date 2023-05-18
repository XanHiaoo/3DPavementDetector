#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include <QObject>
#include <librealsense2/rs.hpp>
#include <QThread>
#include <QString>
#include <opencv2/opencv.hpp>

// 表示相机打开模式的枚举类型
//  Close 表示没有相机打开
enum CameraMode{
    cameraClose, cameraOpen, cameraDisConnect
};
class CameraManager : public QObject
{
    Q_OBJECT
public:
    explicit CameraManager(QObject *parent = nullptr);

    CameraMode getMode() const { return mode_; }

    void setStopProcessing(bool stop_processing) { stop_processing_ = stop_processing; }
    bool getStopProcessing() const { return stop_processing_; }

    void setContext(rs2::context ctx) { ctx_ = ctx; }
    rs2::context getContext() const { return ctx_; }

    void setColorMap(rs2::colorizer color_map) { color_map_ = color_map; }
    rs2::colorizer getColorMap() const { return color_map_; }

    void setPipeline(rs2::pipeline pipe) { pipe_ = pipe; }
    rs2::pipeline getPipeline() const { return pipe_; }

    std::unique_ptr<std::thread>& getImageProcessingLoop() {
        return image_processing_loop_;
    }
    void setImageProcessingLoop(std::unique_ptr<std::thread> loop) {
        image_processing_loop_ = std::move(loop);
    }

    void getCamera();
    void openCamera();
    void closeCamera();
    void processImages();
    cv::Mat detectDefects(const cv::Mat& frame,const cv::Mat& colorImage);

signals:
    void newImage(const cv::Mat& imageOrg,const cv::Mat& imageDetect);
    void clearPlayer();

private:
    CameraMode mode_;
    bool stop_processing_ = false;
    rs2::context ctx_;
    rs2::colorizer color_map_;
    rs2::pipeline pipe_;
    std::unique_ptr<std::thread> image_processing_loop_;

    //void updateImage(const cv::Mat& frame);
};

#endif // CameraManager_H
