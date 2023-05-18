#ifndef PROJECTMANAGER_H
#define PROJECTMANAGER_H

#include <QObject>

class ProjectManager : public QObject
{
    Q_OBJECT

public:
    explicit ProjectManager(QObject* parent = nullptr);

    QString projectPath() const { return m_projectPath; }
    QString outputPath() const { return m_outputPath; }
    QString projectName() const { return m_projectName; }
    bool autoSavePointCloudDetect() const { return m_autoSavePointCloudDetect; }

    void setProjectPath(const QString& path) { m_projectPath = path; }
    void setOutputPath(const QString& path) { m_outputPath = path; }
    void setProjectName(const QString& name) { m_projectName = name; }
    void setAutoSavePointCloudDetect(bool autoSave) { m_autoSavePointCloudDetect = autoSave; }

signals:
    void projectPathChanged();

private:
    QString m_projectPath = "";
    QString m_outputPath = "";
    QString m_projectName = "";
    bool m_autoSavePointCloudDetect = false;

signals:

};

#endif // PROJECTMANAGER_H
