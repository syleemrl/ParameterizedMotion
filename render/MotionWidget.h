#ifndef __RENDER_MOTION_WIDGET_H__
#define __RENDER_MOTION_WIDGET_H__
#include <vector>
#include <QOpenGLWidget>
#include <QTimerEvent>
#include <QKeyEvent>

#pragma push_macro("slots")
#undef slots
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "Camera.h"
#include "GLFunctions.h"
#include "DARTInterface.h"
#include "ReferenceManager.h"
#include "Functions.h"
#include "Controller.h"
#include "EliteSet.h"

#pragma pop_macro("slots")

namespace p = boost::python;
namespace np = boost::python::numpy;

class MotionWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	MotionWidget();
	MotionWidget(std::string _motion, std::string _ppo, bool _parametric);
	void togglePlay();
public slots:
	void nextFrame();
	void prevFrame();
	void reset();

	void setParamFromNetwork();
	void setParamFromEliteSet();

	void setParamValue(const int &x);

	void showPrevMotion();
	void showNextMotion();

protected:
 	void initNetworkSetting(std::string _ppo, bool _parametric);
 	void runPPO();

	void initializeGL() override;	
	void resizeGL(int w,int h) override;
	void paintGL() override;
	void initLights();

	void timerEvent(QTimerEvent* _event);
	void keyPressEvent(QKeyEvent* _event);

	void mousePressEvent(QMouseEvent* _event);
	void mouseMoveEvent(QMouseEvent* _event);
	void mouseReleaseEvent(QMouseEvent* _event);
	void wheelEvent(QWheelEvent* _event);

	void drawGround();
	void drawSkeletons();
	void setFrame(int _n);

	Camera* mCamera;
	int	mPrevX, mPrevY;
	Qt::MouseButton mButton;
	bool mIsDrag;
	bool mTrackCamera;
	
	bool mPlay;
	int mCurFrame;
	int mTotalFrame;

	std::vector<Eigen::VectorXd> mMotionBVH;
	std::vector<Eigen::VectorXd> mMotionPPO;
	std::vector<Eigen::VectorXd> mMotionRef;
	std::vector<double> mTiming;

	dart::dynamics::SkeletonPtr mSkelBVH;
	dart::dynamics::SkeletonPtr mSkelPPO;
	dart::dynamics::SkeletonPtr mSkelRef;

	p::object mPPO;
	p::object mParamNet;
	SIM::ReferenceManager* mReferenceManager;
	SIM::Controller* mController;
	SIM::EliteSet* mEliteSet;

	std::vector<SIM::ParamNode*> mEliteMotions;
	std::vector<double> mEliteFitness;

	int mMotionCounter;

	bool mIsParametric;
	Eigen::VectorXd mParamGoal;

	bool mDrawPPO;
	bool mDrawRef;
};
#endif
