#ifndef PROJECTMANAGER_H
#define PROJECTMANAGER_H

#include <QObject>

class ProjectManager : public QObject
{
    Q_OBJECT

public:
    explicit ProjectManager(QObject *parent = nullptr);

    QString projectPath() const { return m_projectPath; }
    QString outputPath() const { return m_outputPath; }

    void setProjectPath(const QString& path) { m_projectPath = path; }
    void setOutputPath(const QString& path) { m_outputPath = path; }

signals:
    void projectPathChanged();

private:
    QString m_projectPath;
    QString m_outputPath;

signals:

};

#endif // PROJECTMANAGER_H
