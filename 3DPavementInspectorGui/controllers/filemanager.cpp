#include "filemanager.h"
#include <QtDebug>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QMessageBox>
#include <QFile>


QString FileManager::getDir(QString fileName)
{
    QStringList list = fileName.split('/');
    list.pop_back();
    return list.join("/")+"/";
}

QString FileManager::getName(QString fileName)
{
    QStringList list = fileName.split('/');
    fileName = list.back();
    list = fileName.split('.');
    list.pop_back();
    return list.join('.');
}

QString FileManager::getNameWithExtension(QString fileName)
{
    QStringList list = fileName.split('/');
    return list.back();
}


FileManager::FileManager(QObject *parent) : QObject(parent)
{
    mode = Close;
}

void FileManager::close()
{
    mode = Close;
    imageFiles.clear();
    outputFiles.clear();
    curIdx=0;

    emit fileListSetup();
}

void FileManager::setSingleVideo(QString fileName)
{
    imageFiles.clear();
    outputFiles.clear();
    mode = Video;
    curIdx = 0;
    imageFiles<<fileName;

    emit fileListSetup();
}

void FileManager::setMultiImage(QStringList fileNames)
{
    imageFiles.clear();
    outputFiles.clear();

    mode = Pic;
    curIdx = 0;
    fileNames.sort();
    for (auto fileName: fileNames){
        imageFiles<<fileName;
        outputFiles<<getDir(fileName) + getName(fileName);
    }

    emit fileListSetup();
}


void FileManager::prevFile(){
    if (curIdx>0) selectFile(curIdx-1);
}

void FileManager::nextFile(){
    if (curIdx<count()-1) selectFile(curIdx+1);
}

void FileManager::selectFile(int idx){
    if (curIdx==idx) return;
    curIdx=idx;
}



