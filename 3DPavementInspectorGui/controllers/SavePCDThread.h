#pragma once
#ifndef SAVEPCDTHREAD_H
#define SAVEPCDTHREAD_H

#include <QThread.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h> 

class SavePCDThread : public QThread {
public:
    SavePCDThread(const QString& filePath, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mergedCloud)
        : filePath_(filePath), mergedCloud_(mergedCloud) {}

protected:
    void run() override {
        pcl::io::savePCDFile(filePath_.toStdString(), *mergedCloud_);
    }

private:
    QString filePath_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud_;
};
#endif // SAVEPCDTHREAD_H