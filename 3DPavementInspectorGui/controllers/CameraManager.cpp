#include "CameraManager.h"
#include <QtDebug>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QMessageBox>
#include <QFile>

CameraManager::CameraManager(QObject *parent) : QObject(parent)
{
    mode_ = cameraDisConnect;
}

void CameraManager::getCamera(){
    auto cameraList = ctx_.query_devices(); // Get a snapshot of currently connected devices
    if (cameraList.size() == 0){
        mode_ = cameraDisConnect;
    }
    else{
        mode_ = cameraClose;
    }
}

void CameraManager::openCamera(){
    // Declare depth colorizer for pretty visualization of depth data
    color_map_ = rs2::colorizer();

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe_ = rs2::pipeline();
    // Start streaming with default recommended configuration
    pipe_.start();

    // Connect the signal to the slot
    if (!image_processing_loop_ || !image_processing_loop_->joinable()) {
        // If the image processing loop is not already running, start it
        stop_processing_ = false;
        image_processing_loop_ = std::make_unique<std::thread>(&CameraManager::processImages, this);
        mode_ = cameraOpen;
    }
}

void CameraManager::closeCamera(){
    stop_processing_= true;
    if (stop_processing_&& image_processing_loop_->joinable()) {
        image_processing_loop_->join();
    }
    mode_ = cameraClose;
    emit clearPlayer();
}

void CameraManager::processImages()
{
    while (!stop_processing_){
        try {
            rs2::frameset data = pipe_.wait_for_frames(); // Wait for next set of frames from the camera
            rs2::frame depth = data.get_depth_frame().apply_filter(color_map_);
            rs2::frame color = data.get_color_frame();
            // Query frame size (width and height)
            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();

            // Create OpenCV matrix of size (w,h) from the colorized depth data
            cv::Mat cv_image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat color_image(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat result_image = detectDefects(depth_image,color_image);
            // Emit a signal to update the label with the new image
            emit newImage(cv_image,result_image);
        }
        catch (const rs2::error& e) {
            break ;
            //std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        }
        catch (const std::exception& e) {
            break ;
            //std::cerr << e.what() << std::endl;
        }
    }
}

cv::Mat CameraManager::detectDefects(const cv::Mat& frame,const cv::Mat& colorImage){
    // Convert the image to grayscale
    cv::Mat gray_image;
    cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);

    // Apply binary thresholding to get the edges
    //cv::Mat thresholded_image;
    //threshold(gray_image, thresholded_image, 30, 255, cv::THRESH_BINARY);

    // Find the contours in the thresholded image
    std::vector<std::vector<cv::Point>> contours;
    findContours(gray_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw the contours and convex hulls on the original image
    cv::Mat result_image = colorImage.clone();
    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area < 500 || area > 100000) continue; // Ignore small contours

        cv::Rect bbox = boundingRect(contours[i]);
        rectangle(result_image, bbox, cv::Scalar(0, 255, 0), 2);
    }

    // Convert BGR to RGB
    cvtColor(result_image, result_image, cv::COLOR_BGR2RGB);

    // Show the result
    return result_image;
}
//cv::Mat CameraManager::detectDefects(const cv::Mat& frame, const cv::Mat& colorImage) {
//    // Convert the image to grayscale
//    cv::Mat gray_image;
//    cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);
//
//    // Calculate the average grayscale value
//    cv::Scalar mean_gray = cv::mean(gray_image);
//    double average_gray = mean_gray[0];
//
//    // Color the regions greater than the average as green and less than the average as red
//    cv::Mat result_image = colorImage.clone();
//    for (int y = 0; y < gray_image.rows; ++y) {
//        for (int x = 0; x < gray_image.cols; ++x) {
//            if (gray_image.at<uchar>(y, x) > average_gray + 50) {
//                result_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);  // Green color
//            }
//            else if (gray_image.at<uchar>(y, x) < average_gray - 50) {
//                result_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);  // Red color
//            }
//        }
//    }
//
//    // Return the result
//    cvtColor(result_image, result_image, cv::COLOR_BGR2RGB);
//    return result_image;
//}
