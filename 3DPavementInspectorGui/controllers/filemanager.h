#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <exception>
#include <string>


// 表示文件打开模式的枚举类型
//  Close 表示没有文件打开
//  Open 表示打开文件
enum FileMode{
    Close, Open, Pic, Video
};

class FileManager : public QObject
{
    Q_OBJECT
public:
    // 从完整的文件路径中获取文件所在文件夹的路径 example: "../../../abc.d" => "../../../"
    static QString getDir(QString fileName);
    // 从完整的文件路径中获取文件名（不含扩展名） example: "../../../abc.d" => abc
    static QString getName(QString fileName);
    // 从完整的文件路径中获取文件名（含扩展名）   example: "../../../abc.d" => abc.d
    static QString getNameWithExtension(QString fileName);

    explicit FileManager(QObject *parent = nullptr);

    QString getCurrentImageFile() const { return imageFiles[curIdx]; }

    FileMode getMode() const { return mode; }

    const QStringList &allImageFiles() const { return imageFiles; }

    int getCurIdx() const  { return curIdx; }

    int count() const { return imageFiles.length(); }  

    // 关闭打开的文件
    void close();

    void setSingleVideo(QString fileName);
    void setMultiImage(QStringList fileNames);

signals:
    void fileListSetup();           // 文件列表载入完毕

public slots:
    void prevFile();            // 切换到上一张
    void nextFile();            // 切换到下一张
    void selectFile(int idx);   // 切换到指定的一张图片

private:
    QStringList imageFiles;
    QStringList outputFiles;

    int curIdx;
    FileMode mode;

};

#endif // FILEMANAGER_H
