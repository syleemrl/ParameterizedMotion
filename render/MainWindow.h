#ifndef __RENDER_MAIN_WINDOW_H__
#define __RENDER_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include "MotionWidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
public:
    MainWindow();
    MainWindow(std::string _motion, std::string _ppo, bool _parametric);

protected:
	QHBoxLayout* mMainLayout;
	QPushButton* mButton;
	MotionWidget* mMotionWidget;

	void initLayoutSetting(std::string _motion, std::string _ppo, bool _parametric);

};
#endif