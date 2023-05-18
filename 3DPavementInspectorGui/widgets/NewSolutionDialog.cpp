#include "NewSolutionDialog.h"
#include <QGridLayout>

NewSolutionDialog::NewSolutionDialog(QWidget *parent) :
    QDialog(parent)
{
    setWindowTitle("创建解决方案");
    m_solutionNameLabel = new QLabel("解决方案名:");
    m_solutionNameLineEdit = new QLineEdit;

    m_projectNameLabel = new QLabel("项目名:");
    m_projectNameLineEdit = new QLineEdit;

    m_okButton = new QPushButton("确认");
    m_cancelButton = new QPushButton("取消");

    connect(m_okButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(m_solutionNameLabel, 0, 0);
    layout->addWidget(m_solutionNameLineEdit, 0, 1);
    layout->addWidget(m_projectNameLabel, 1, 0);
    layout->addWidget(m_projectNameLineEdit, 1, 1);
    layout->addWidget(m_okButton, 2, 0);
    layout->addWidget(m_cancelButton, 2, 1);

    setLayout(layout);
}

NewSolutionDialog::~NewSolutionDialog()
{
    delete m_solutionNameLabel;
    delete m_projectNameLabel;
    delete m_solutionNameLineEdit;
    delete m_projectNameLineEdit;
    delete m_okButton;
    delete m_cancelButton;
}

QString NewSolutionDialog::getSolutionName() const
{
    return m_solutionNameLineEdit->text();
}

QString NewSolutionDialog::getProjectName() const
{
    return m_projectNameLineEdit->text();
}
