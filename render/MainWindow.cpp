#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include "MainWindow.h"
MainWindow::
MainWindow() :QMainWindow()
{
    setWindowTitle("Renderer");
}
MainWindow::
MainWindow(std::string _motion, std::string _ppo, bool _parametric)
{   
    MainWindow();
    initLayoutSetting(_motion, _ppo, _parametric);
}
void
MainWindow::
initLayoutSetting(std::string _motion, std::string _ppo, bool _parametric) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1300,750);
    setMinimumSize(1300,750);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new MotionWidget(_motion, _ppo, _parametric);

    mMotionWidget->setMinimumSize(1000,650);
    mMotionWidget->setMaximumSize(1000,650);

    motionlayout->addWidget(mMotionWidget);

    QHBoxLayout *buttonlayout = new QHBoxLayout();
    buttonlayout->addStretch(1);

    QPushButton* button = new QPushButton("reset", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(reset())); 
    buttonlayout->addWidget(button);
    
    button = new QPushButton("prev", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(prevFrame())); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("play", this);
    button->setCheckable(true);
    connect(button, SIGNAL(toggled(bool)), this, SLOT(togglePlay(const bool&))); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("next", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(nextFrame())); 
    buttonlayout->addWidget(button);    
    buttonlayout->addStretch(1);

    motionlayout->addLayout(buttonlayout);
    mMainLayout->addLayout(motionlayout);

    if(_parametric) {
        QVBoxLayout *paramlayout = new QVBoxLayout();
        std::vector<std::string> labels;
        labels.push_back("gravity");
        labels.push_back("energy");

        QFormLayout *paramFormlayout = new QFormLayout();
        for(int i = 0; i < labels.size(); i++) {
            QSlider* param = new QSlider(Qt::Horizontal);
            param->setMinimum(0);
            param->setMaximum(20);
            param->setSingleStep(1);
            param->setProperty("i", i);
            
            paramFormlayout->addRow(QString::fromStdString(labels[i]), param);
            connect (param, SIGNAL(valueChanged(int)), mMotionWidget, SLOT(setParamValue(const int&)));
        }

        paramlayout->addStretch(1);
        paramlayout->addLayout(paramFormlayout);
        paramlayout->addStretch(1);
           
        button = new QPushButton("set (param network)", this);
        button->setStyleSheet("margin-bottom: 10px;"
                               "padding: 5px;");
        paramlayout->addWidget(button);

        connect (button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(setParamFromNetwork()));

        button = new QPushButton("set (elite set)", this);
        button->setStyleSheet("margin-bottom: 10px;"
                               "padding: 5px;");
        paramlayout->addWidget(button);

        connect (button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(setParamFromEliteSet()));

        QHBoxLayout* paramButtonlayout = new QHBoxLayout();

    //     button = new QPushButton("set", this);
    //     button->setStyleSheet("margin-bottom: 10px;"
    //                            "padding: 5px;");
    //     paramButtonlayout->addWidget(button);

    //     connect (button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(setGoalParam()));
   
    //  show training data
        button = new QPushButton("prev sample", this);
        button->setStyleSheet("margin-bottom: 10px;"
                               "padding: 5px;");
        paramButtonlayout->addWidget(button);
        connect (button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(showPrevMotion()));

        button = new QPushButton("next sample", this);
        button->setStyleSheet("margin-bottom: 10px;"
                               "padding: 5px;");
        paramButtonlayout->addWidget(button);
        connect (button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(showNextMotion()));
        
        paramlayout->addLayout(paramButtonlayout);

        mMainLayout->addLayout(paramlayout);
    }
    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
}
void 
MainWindow::
togglePlay(const bool& _toggled)
{
    auto button = qobject_cast<QPushButton*>(sender());
    if(_toggled) {
        button->setText("pause");
    } else {
        button->setText("play");
    }
    mMotionWidget->togglePlay();
}

