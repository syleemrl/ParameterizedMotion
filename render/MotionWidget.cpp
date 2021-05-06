#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "MotionWidget.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
MotionWidget::
MotionWidget()
  :mCamera(new Camera(1000, 650)), mCurFrame(0), mPlay(false), mTrackCamera(true), 
  mDrawPPO(false), mDrawRef(false)
{
	startTimer(30);
}
MotionWidget::
MotionWidget(std::string _motion, std::string _ppo, bool _parametric)
  :MotionWidget()
{
	mIsParametric = _parametric;

	mCurFrame = 0;
	mTotalFrame = 0;

	std::string skelPath = std::string(PM_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
	std::string motionPath = std::string(PM_DIR) + std::string("/data/motion/") + _motion;

	if(!boost::filesystem::exists(skelPath)) {
		SIM::SkeletonBuilder::generateNewSkeleton(motionPath, skelPath);
	}
    mSkelBVH = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
    mSkelPPO = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
    mSkelRef = SIM::SkeletonBuilder::buildFromFile(skelPath).first;

    mReferenceManager = new SIM::ReferenceManager(SIM::SkeletonBuilder::buildFromFile(skelPath).first);
    mReferenceManager->loadMotionFromBVH(motionPath);
   
    mMotionBVH.clear();
    for(int i = 0; i < 2 * TERMINAL_ITERATION * mReferenceManager->getMotionLength(); i++) {
        Eigen::VectorXd p = mReferenceManager->getBVHFrame(i).position;
        mMotionBVH.push_back(p);

        if(_parametric) {
	        p(3) += 1.5;
	        mMotionRef.push_back(p);
        }
    }

	mTotalFrame = mMotionBVH.size() - 1;

	GUI::setSkeletonColor(mSkelBVH, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));
	GUI::setSkeletonColor(mSkelPPO, Eigen::Vector4d(235./255., 235./255., 235./255., 1.0));
	GUI::setSkeletonColor(mSkelRef, Eigen::Vector4d(73./255., 235./255., 73./255., 1.0));

	setFocusPolicy( Qt::StrongFocus );
	initNetworkSetting(_ppo, _parametric);

}
void
MotionWidget::
initNetworkSetting(std::string _ppo, bool _parametric) {

    Py_Initialize();
    np::initialize();

    try {
 		p::object sys_module = p::import("sys");
		p::str module_dir = (std::string(PM_DIR)+"/network").c_str();
		sys_module.attr("path").attr("insert")(1, module_dir);

    	if(_ppo != "") {
    		mDrawPPO = true;
    		int numParams = 0;
   			std::string paramPath = std::string(PM_DIR)+ std::string("/network/output/") + SIM::split(_ppo,'/')[0];
 
    		if(_parametric) {
    			mDrawRef = true;

				mEliteSet = new SIM::EliteSet(mReferenceManager->getNumDof(), mReferenceManager->getMotionLength());
				mEliteSet->setConfigurations();
				mEliteSet->loadParamSpace(paramPath + "/param_space");
				mEliteMotions = mEliteSet->traverseTree();

				mReferenceManager->setEliteSet(mEliteSet);
				numParams = mEliteSet->getNumParam();

				mMotionCounter = 0;
				mParamGoal.resize(numParams);

				p::object regression = p::import("regression");
				mParamNet = regression.attr("Regression")();
				mParamNet.attr("init_run")(paramPath, numParams + 2, mReferenceManager->getNumDof());

			}
    		
    		mController = new SIM::Controller(mReferenceManager, _parametric, true, numParams);

    		p::object ppo_main = p::import("ppo");
			mPPO = ppo_main.attr("PPO")();
			std::string ppoPath = std::string(PM_DIR)+ std::string("/network/output/") + _ppo;
			mPPO.attr("init_run")(ppoPath,
								 mController->getNumState(), 
								 mController->getNumAction());

    		if(_parametric) {
    			Eigen::VectorXd p = mEliteSet->getParamGoal();
				mParamGoal = mEliteSet->normalize(p);

				std::vector<Eigen::VectorXd> inputMatrix;
				double A = mReferenceManager->getMotionLength() / 2.0;
				for(int i = 0; i < mReferenceManager->getMotionLength(); i++) {
					Eigen::VectorXd input(mParamGoal.rows() + 2);
					input << sin(i / A * M_PI), cos(i / A * M_PI), mParamGoal;

					inputMatrix.push_back(input);
				}
				p::object out = mParamNet.attr("run")(SIM::toNumPyArray(inputMatrix));
				np::ndarray nout = np::from_object(out);
				Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, mReferenceManager->getMotionLength(), mReferenceManager->getNumDof());
				
				std::vector<Eigen::VectorXd> displacements;
				for(int i = 0; i < mReferenceManager->getMotionLength(); i++) {
					displacements.push_back(outputMatrix.row(i));
				}
				mReferenceManager->loadMotionFromDisplacement(displacements);

    			mController->setParamGoal(p);
    		}

			runPPO();

    	}
    
    } catch (const p::error_already_set&) {
        PyErr_Print();
    }    
}
void
MotionWidget::
runPPO() {
	mController->reset(false);

	int count = 0;
	while(!mController->isTerminalState()) {
		Eigen::VectorXd state = mController->getState();

		p::object a = mPPO.attr("run")(SIM::toNumPyArray(state));
		np::ndarray na = np::from_object(a);
		Eigen::VectorXd action = SIM::toEigenVector(na, mController->getNumAction());
		mController->setAction(action);
		mController->step();
		count += 1;
	}
	std::vector<Eigen::VectorXd> pos;

	mTiming.clear();
 	// mMotionBVH.clear();
    mMotionRef.clear();
    mMotionPPO.clear();

	for(int i = 0; i < count; i++) {
		mTiming.push_back(mController->getPhase(i));

		// Eigen::VectorXd position = mController->getBVHPosition(i);
		// mMotionBVH.push_back(position);

		// position = mController->getTargetPosition(i);
		// position(3) += 1.5;
		// mMotionRef.push_back(position);

		Eigen::VectorXd position = mController->getPosition(i);
		position(3) += 1.5;
		// if(mIsParametric)
		// 	position(3) += 1.5;
		mMotionPPO.push_back(position);
	}

	mTotalFrame = count-1;
	mCurFrame = 0;
}
void
MotionWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
MotionWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
MotionWidget::
setFrame(int n)
{
    mSkelBVH->setPositions(mMotionBVH[n]);
    if(mDrawPPO)
    	mSkelPPO->setPositions(mMotionPPO[n]);
    // if(mDrawRef)
    // 	mSkelRef->setPositions(mMotionRef[n]);
}
void
MotionWidget::
drawSkeletons()
{
	GUI::drawSkeleton(mSkelBVH, 0);
	if(mDrawPPO)
		GUI::drawSkeleton(mSkelPPO, 0);
	// if(mDrawRef)
	// 	GUI::drawSkeleton(mSkelRef, 0);
}	
void
MotionWidget::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelBVH->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void
MotionWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);
	
	if(mTrackCamera){
		Eigen::Vector3d com = mSkelBVH->getRootBodyNode()->getCOM();
		Eigen::Isometry3d transform = mSkelBVH->getRootBodyNode()->getTransform();
		if(mIsParametric) {
			com = mSkelPPO->getRootBodyNode()->getCOM();
			transform = mSkelPPO->getRootBodyNode()->getTransform();
		}
		com[1] = 0.8;
		mCamera->setCenter(com);
	}
	mCamera->apply();

	drawGround();
	drawSkeletons();
	if(mDrawPPO) 
		GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mTiming[mCurFrame])+" / "+std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
	else 
		GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());

	if(!mDrawPPO && mDrawRef) {
		GUI::drawStringOnScreen(0.8, 0.1, std::to_string(mMotionCounter)+" / "+std::to_string(mEliteMotions.size()), true, Eigen::Vector3d::Zero());
		GUI::drawStringOnScreen(0.8, 0.05, std::to_string(mEliteMotions[mMotionCounter]->getFitness()), true, Eigen::Vector3d::Zero());
	}

}
void
MotionWidget::
initLights()
{

	float ambient[]           	 = {0.4, 0.4, 0.4, 1.0};
	float diffuse[]             = {0.4, 0.4, 0.4, 1.0};
	float frontShininess[] = {60.0};
	float frontSpecular[]  = {0.2, 0.2,  0.2,  1.0};
	float frontDiffuse[]   = {0.2, 0.2, 0.2, 1.0};
	float lmodelAmbient[]      = {0.2, 0.2,  0.2,  1.0};
	float lmodelTwoside[]      = {GL_TRUE};

	GLfloat position[] = {0.0, 1.0, 1.0, 0.0};
	GLfloat position1[] = {0.0, 1.0, -1.0, 0.0};

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodelAmbient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodelTwoside);

	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position1);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, frontShininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  frontSpecular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   frontDiffuse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	glEnable(GL_FOG);
	GLfloat fogColor[] = {200.0/256.0, 200.0/256.0, 200.0/256.0, 1};
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_DENSITY, 0.05);
	glFogf(GL_FOG_START, 20.0);
	glFogf(GL_FOG_END, 40.0);
}
void
MotionWidget::
timerEvent(QTimerEvent* _event)
{
	if(mPlay && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
	} 
	setFrame(mCurFrame);
	update();

}
void
MotionWidget::
keyPressEvent(QKeyEvent* _event)
{
	if(_event->key() == Qt::Key_Escape){
		exit(0);
	}
	if(_event->key() == Qt::Key_Space){
		mPlay = !mPlay;
		if(mPlay)
			std::cout << "Play." << std::endl;
		else 
			std::cout << "Pause." << std::endl;
	}
	if(_event->key() == Qt::Key_T){
		mTrackCamera = !mTrackCamera;
		std::cout << "track camera: " << mTrackCamera << std::endl;
	}
}
void
MotionWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
MotionWidget::
mouseMoveEvent(QMouseEvent* _event)
{
	if(!mIsDrag)
	return;

	if (mButton == Qt::MidButton)
		mCamera->translate(_event->x(), _event->y(), mPrevX, mPrevY);
	else if(mButton == Qt::LeftButton)
		mCamera->rotate(_event->x(), _event->y(), mPrevX, mPrevY);

	mPrevX = _event->x();
	mPrevY = _event->y();
	update();
}
void
MotionWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
MotionWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}
void
MotionWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
MotionWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
MotionWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
MotionWidget::
togglePlay() {
	mPlay = !mPlay;
}
void 
MotionWidget::
setParamFromNetwork() {
	mDrawPPO = true;

	Eigen::VectorXd p = mEliteSet->denormalize(mParamGoal);
	
	Eigen::VectorXd p_ = p;
	p_(0) = exp(p_(0)) * -9.81;
	std::cout << "new goal : " << p_.transpose() << " / density: " << mEliteSet->getDensity(mParamGoal) << std::endl;
	
	std::vector<Eigen::VectorXd> inputMatrix;
	double A = mReferenceManager->getMotionLength() / 2.0;
	for(int i = 0; i < mReferenceManager->getMotionLength(); i++) {
		Eigen::VectorXd input(mParamGoal.rows() + 2);
		input << sin(i / A * M_PI), cos(i / A * M_PI), mParamGoal;

		inputMatrix.push_back(input);
	}
	p::object out = mParamNet.attr("run")(SIM::toNumPyArray(inputMatrix));
	np::ndarray nout = np::from_object(out);
	Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, mReferenceManager->getMotionLength(), mReferenceManager->getNumDof());
	
	std::vector<Eigen::VectorXd> displacements;
	for(int i = 0; i < mReferenceManager->getMotionLength(); i++) {
		displacements.push_back(outputMatrix.row(i));
	}
	mReferenceManager->loadMotionFromDisplacement(displacements);
	mController->setParamGoal(p);
	mEliteSet->setParamGoal(p);

	runPPO();

}
void 
MotionWidget::
setParamFromEliteSet() {
	mDrawPPO = true;
	Eigen::VectorXd p = mEliteSet->denormalize(mParamGoal);
	Eigen::VectorXd p_ = p;
	p_(0) = exp(p_(0)) * -9.81;
	std::cout << "new goal : " << p_.transpose() << " / density: " << mEliteSet->getDensity(mParamGoal) << std::endl;
	
	std::vector<Eigen::VectorXd> displacements = mEliteSet->getWeightedKNN(p);
	mReferenceManager->loadMotionFromDisplacement(displacements);
	mController->setParamGoal(p);
	mEliteSet->setParamGoal(p);

	runPPO();

}
void 
MotionWidget::
showPrevMotion() {
	mDrawPPO = false;
	mMotionCounter -= 1;
	if(mMotionCounter < 0)
		mMotionCounter = mEliteMotions.size() - 1;
	
	std::vector<Eigen::VectorXd> displacements = mEliteMotions[mMotionCounter]->getDisplacement();
	mReferenceManager->loadMotionFromDisplacement(displacements);
    
    mMotionBVH.clear();
    mMotionRef.clear();
    double frame = 0;
    while(1) {
        Eigen::VectorXd p = mReferenceManager->getBVHFrame(frame).position;
        mMotionBVH.push_back(p);

        SIM::Frame f = mReferenceManager->getFrame(frame);
        p = f.position;
        p(3) += 1.5;
       	mMotionRef.push_back(p);

       	frame += f.phaseDelta;
    	if(frame >= TERMINAL_ITERATION * mReferenceManager->getMotionLength())
    		break;
    }
	mCurFrame = 0;
	mTotalFrame = mMotionBVH.size() - 1;
}
void 
MotionWidget::
showNextMotion() {
	mDrawPPO = false;
	mMotionCounter += 1;
	if(mMotionCounter > mEliteMotions.size() - 1)
		mMotionCounter = 0;

	std::vector<Eigen::VectorXd> displacements = mEliteMotions[mMotionCounter]->getDisplacement();
	mReferenceManager->loadMotionFromDisplacement(displacements);

    mMotionBVH.clear();
    mMotionRef.clear();
    double frame = 0;
    while(1) {
        Eigen::VectorXd p = mReferenceManager->getBVHFrame(frame).position;
        mMotionBVH.push_back(p);

        SIM::Frame f = mReferenceManager->getFrame(frame);
        p = f.position;
        p(3) += 1.5;
       	mMotionRef.push_back(p);

       	frame += f.phaseDelta;
    	if(frame >= TERMINAL_ITERATION * mReferenceManager->getMotionLength())
    		break;
    }
	mCurFrame = 0;
	mTotalFrame = mMotionBVH.size() - 1;
}
void 
MotionWidget::
setParamValue(const int &x){
	auto slider = qobject_cast<QSlider*>(sender());
    auto i = slider->property("i").toInt();
    mParamGoal(i) =  x * 1 / 20.0;
}