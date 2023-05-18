// newsolutiondialog.h
#ifndef NEWSOLUTIONDIALOG_H
#define NEWSOLUTIONDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class NewSolutionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit NewSolutionDialog(QWidget *parent = nullptr);
    ~NewSolutionDialog();
    QString getSolutionName() const;
    QString getProjectName() const;
private:
    QLabel *m_solutionNameLabel;
    QLabel *m_projectNameLabel;
    QLineEdit *m_solutionNameLineEdit;
    QLineEdit *m_projectNameLineEdit;
    QPushButton *m_okButton;
    QPushButton *m_cancelButton;
};

#endif // NEWSOLUTIONDIALOG_H
