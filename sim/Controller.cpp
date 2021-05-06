#include "Controller.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
#include "Functions.h"
#include <boost/filesystem.hpp>
#include <Eigen/QR>
#include <fstream>
#include <numeric>
#include <algorithm>
namespace SIM
{	

Controller::
Controller(ReferenceManager* _ref, bool _parametric, bool _render, int _numParam, int _id)
	:mControlHz(30),mSimulationHz(150)
{
	mIsParametric = _parametric;
	mRender = _render;
	mReferenceManager = _ref;
	mId = _id;

	if(mIsParametric || mRender)
		mPhaseDeltaWeight = 1;
	else
		mPhaseDeltaWeight = 0;

	mSimPerCon = mSimulationHz / mControlHz;
	mWorld = std::make_shared<dart::simulation::World>();

	mWorld->setGravity(Eigen::Vector3d(0,-9.81, 0));
	mWorld->setTimeStep(1.0/(double)mSimulationHz);
	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
	dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(mWorld->getConstraintSolver())->setBoxedLcpSolver(std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
	
	mGround = SkeletonBuilder::buildFromFile(std::string(PM_DIR)+std::string("/data/character/ground.xml")).first;
	mGround->getBodyNode(0)->setFrictionCoeff(1.0);
	mWorld->addSkeleton(mGround);
	
	std::string path = std::string(PM_DIR)+std::string("/data/character/") + std::string(CHARACTER_TYPE) + std::string(".xml");
	auto pair = SkeletonBuilder::buildFromFile(path);
	mCharacter = pair.first;
	mTorqueMap = pair.second;

	mWorld->addSkeleton(mCharacter);

	auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
	mCGL = collisionEngine->createCollisionGroup(mCharacter->getBodyNode(LeftFoot));
	mCGR = collisionEngine->createCollisionGroup(mCharacter->getBodyNode(RightFoot));
	mCGEL = collisionEngine->createCollisionGroup(mCharacter->getBodyNode(LeftToe));
	mCGER = collisionEngine->createCollisionGroup(mCharacter->getBodyNode(RightToe));
	mCGHL = collisionEngine->createCollisionGroup(mCharacter->getBodyNode(LeftHand));
	mCGHR = collisionEngine->createCollisionGroup(mCharacter->getBodyNode(RightHand));
	mCGG = collisionEngine->createCollisionGroup(mGround.get());
	
	mEndEffectors.clear();
	mEndEffectors.push_back(RightFoot);
	mEndEffectors.push_back(LeftFoot);
	mEndEffectors.push_back(RightHand);
	mEndEffectors.push_back(LeftHand);
	mEndEffectors.push_back(Head);

	mControllableDof = mCharacter->getNumDofs() - 6;
	mTotalDof = mCharacter->getNumDofs();

	mActions = Eigen::VectorXd::Zero(mControllableDof + 1);

	int posDim = (mCharacter->getNumBodyNodes() - 1) * 6;
	int velDim = mTotalDof;
	int eeDim = mEndEffectors.size() * 3;
	int nextPosDim = mEndEffectors.size() * 12 + 15;
	int stateDim = posDim + velDim + eeDim + nextPosDim + 4;
	if(mIsParametric)
		stateDim += _numParam;

	mStates = Eigen::VectorXd::Zero(stateDim);

	mTargetPosition = Eigen::VectorXd::Zero(mTotalDof);
	mTargetVelocity = Eigen::VectorXd::Zero(mTotalDof);

	mPDTargetPosition = Eigen::VectorXd::Zero(mTotalDof);
	mPDTargetVelocity = Eigen::VectorXd::Zero(mTotalDof);

	mNumState = mStates.size();
	mNumAction = mActions.size();

	mRewardLabels.clear();
	if(!mIsParametric) {
		mRewardLabels.push_back("total");
		mRewardLabels.push_back("p");
		mRewardLabels.push_back("ee");
		mRewardLabels.push_back("v");
		mRewardLabels.push_back("time");
	} else {
		mRewardLabels.push_back("continuous");
		mRewardLabels.push_back("spike");
		mRewardLabels.push_back("space");
		mRewardLabels.push_back("time");
	}

	mRewardParts.resize(mRewardLabels.size(), 0.0);
	mParamCur.resize(_numParam);
}
void
Controller::
stepBVH()
{	
	if(mIsTerminal)
		return;

	Frame pvTarget = mReferenceManager->getFrame(mCurrentFrame);
	mTargetPosition = pvTarget.position;
	mTargetVelocity = pvTarget.velocity;

	mCharacter->setPositions(mTargetPosition);
	mCharacter->setVelocities(mTargetVelocity);

	mCurrentFrame += 1;
	mCurrentFrameOnPhase += 1;
	mTimeElapsed += 1;

	if(mCurrentFrameOnPhase > mReferenceManager->getMotionLength()){
		mCurrentFrameOnPhase -= mReferenceManager->getMotionLength();
	}

	updateState();
	updateTerminalInfo();

	if(mRender) {
		updateRecord();
	}

}
void 
Controller::
step()
{			
	if(mIsTerminal)
		return;
	for(int i = 0; i < mControllableDof; i++){
		mActions[i] = dart::math::clip(mActions[i]*0.2, -0.7*M_PI, 0.7*M_PI);
	}
	mActions[mControllableDof] = dart::math::clip(mActions[mControllableDof]*1.5, -2.0, 1.0);
	mActions[mControllableDof] = exp(mActions[mControllableDof]);
	mPhaseDelta = mPhaseDeltaWeight * mActions[mControllableDof] + (1 - mPhaseDeltaWeight);

	mCurrentFrame += mPhaseDelta;
	mCurrentFrameOnPhase += mPhaseDelta;
	Frame pvTarget = mReferenceManager->getFrame(mCurrentFrame);
	mTargetPosition = pvTarget.position;
	mTargetVelocity = pvTarget.velocity;
	double targetPhaseDelta = pvTarget.phaseDelta;

	pvTarget = mReferenceManager->getBVHFrame(mCurrentFrame);
	mPDTargetPosition = pvTarget.position;
	mPDTargetVelocity = pvTarget.velocity;

	for(int i = 1; i < mCharacter->getNumBodyNodes(); i++){
		int idx = mCharacter->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
		int dof = mCharacter->getBodyNode(i)->getParentJoint()->getNumDofs();
		mPDTargetPosition.block(idx, 0, dof, 1) += mActions.block((i-1)*3, 0, dof, 1);
	}
	
	for(int i = 0; i < mSimPerCon; i++){
		// set torque limit 
		Eigen::VectorXd torque = mCharacter->getSPDForces(mPDTargetPosition, 600, 49, mWorld->getConstraintSolver());
		for(int j = 1; j < mCharacter->getNumBodyNodes(); j++) {
			int idx = mCharacter->getBodyNode(j)->getParentJoint()->getIndexInSkeleton(0);
			int dof = mCharacter->getBodyNode(j)->getParentJoint()->getNumDofs();
			std::string name = mCharacter->getBodyNode(j)->getName();
			double torquelim = mTorqueMap.find(name)->second;
			double torque_norm = torque.block(idx, 0, dof, 1).norm();
			
			torque.block(idx, 0, dof, 1) = std::max(-torquelim, std::min(torquelim, torque_norm)) * torque.block(idx, 0, dof, 1).normalized();
		}
		mCharacter->setForces(torque);
	
	//	mCharacter->setSPDTarget(mPDTargetPosition, 600, 49);
		mWorld->step(false);

		if(mIsParametric && mCurrentFrameOnPhase >= 18 && mTaskStage == 0) {
			Eigen::Vector3d curVel = mCharacter->getCOMLinearVelocity();
			if(mTakeOffVelocity < curVel(1)) {
				mTakeOffVelocity = curVel(1);
				mMomentum = mCharacter->getMass() * curVel;
			}
		}
	}
	mTimeElapsed += 1;
	if(mCurrentFrameOnPhase >= mReferenceManager->getMotionLength()){
		mCurrentFrameOnPhase -= mReferenceManager->getMotionLength();
		if(mIsParametric && !mIsTerminal) {

			mFitness.sumPos /= mCountTracking;
			mFitness.sumVel /= mCountTracking;
			mFitness.sumContact /= mCountTracking;
			if(mSaveTrajectory)
				mReferenceManager->saveTrajectory(mTrajectory, mFitness, mParamCur);

			clearEpisode();

		}
	}

	updateReward();
	mTargetPhaseDelta = targetPhaseDelta;	

	updateState();
	updateTerminalInfo();

	if(mRender) {
		updateRecord();
	}

	if(mIsParametric) {
		updateFitness();
		mTrajectory.first.push_back(mCharacter->getPositions());
		mTrajectory.second.push_back(mCurrentFrameOnPhase);
	}
	mPrevPositions = mCharacter->getPositions();

}
void
Controller::
updateRecord() 
{
	mRecordBVHPosition.push_back(mReferenceManager->getBVHFrame(mCurrentFrame).position);
	mRecordTargetPosition.push_back(mReferenceManager->getFrame(mCurrentFrame).position);
	mRecordPosition.push_back(mCharacter->getPositions());
	mRecordPhase.push_back(mCurrentFrame);
	bool rightContact = checkCollisionWithGround(RightToe) || checkCollisionWithGround(RightFoot);
	bool leftContact = checkCollisionWithGround(LeftToe) || checkCollisionWithGround(LeftFoot);

	mRecordFootContact.push_back(std::make_pair(rightContact, leftContact));
}
void 
Controller::
clearRecord() 
{
	mRecordPosition.clear();
	mRecordTargetPosition.clear();
	mRecordBVHPosition.clear();
	mRecordPhase.clear();
	mRecordFootContact.clear();
}
void
Controller::
clearEpisode()
{
	mTrajectory.first.clear();
	mTrajectory.second.clear();
	
	mParamCur.setZero();
	mTaskStage = 0;

	mCountTracking = 0;

	mFitness.sumPos = 0;
	mFitness.sumVel = 0;
	mFitness.sumContact = 0;
	mFitness.sumSlide = 0;
	mFitness.sumConstraint = 0;
	
	mRootZero = mCharacter->getPositions().segment<6>(0);

	mSaveTrajectory = true;
	mTakeOffVelocity = 0;
	mCondiff = 0;
	mMomentum.setZero();
}
std::vector<double> 
Controller::
getTrackingReward(Eigen::VectorXd _posCur, Eigen::VectorXd _posTarget, 
				  Eigen::VectorXd _velCur, Eigen::VectorXd _velTarget, 
				  bool _useVelocity)
{
	int num_body_nodes = mCharacter->getNumBodyNodes();

	Eigen::VectorXd posSave = mCharacter->getPositions();

	Eigen::VectorXd posDiff = mCharacter->getPositionDifferences(_posCur, _posTarget);
	Eigen::VectorXd velDiff;

	if(_useVelocity) {
		velDiff = mCharacter->getVelocityDifferences(_velCur, _velTarget);
	}

	mCharacter->setPositions(_posCur);

	std::vector<Eigen::Isometry3d> eeTransforms;
	Eigen::VectorXd eeDiff(mEndEffectors.size()*3);
	eeDiff.setZero();	
	for(int i = 0; i < mEndEffectors.size(); i++){
		eeTransforms.push_back(mCharacter->getBodyNode(mEndEffectors[i])->getWorldTransform());
	}
	Eigen::Vector3d comDiff = mCharacter->getCOM();
		
	mCharacter->setPositions(_posTarget);

	for(int i = 0; i < mEndEffectors.size(); i++){
		Eigen::Isometry3d diff = eeTransforms[i].inverse() * mCharacter->getBodyNode(mEndEffectors[i])->getWorldTransform();
		eeDiff.segment<3>(3*i) = diff.translation();
	}
	comDiff -= mCharacter->getCOM();

	double sigPos = 0.4; 
	double sigVel = 3;	
	double sigEE = 0.2;		
	double sigCOM = 0.2;		

	double posReward = expOfSquared(posDiff, sigPos);
	double velReward;
	if(_useVelocity)
	{
		velReward = expOfSquared(velDiff, sigVel);
	}
	double eeReward = expOfSquared(eeDiff, sigEE);
	double comReward = expOfSquared(comDiff, sigCOM);

	std::vector<double> rewards;
	rewards.clear();

	rewards.push_back(posReward);
	rewards.push_back(eeReward);
	rewards.push_back(comReward);

	if(_useVelocity) {
		rewards.push_back(velReward);
	}

	mCharacter->setPositions(posSave);

	return rewards;

}
double 
Controller::
getTaskReward()
{
	double rewardTask = 0;

	if(mCurrentFrameOnPhase >= 26 && mTaskStage == 0) {
		Eigen::Vector3d momentumGoal = Eigen::Vector3d(0, 160 * mParamGoal(1), 0);
		Eigen::Vector3d mDiff = momentumGoal - mMomentum;
		mDiff *= 0.1;
		mDiff(1) *= 2;

		double rewardMomentum = expOfSquared(mDiff, 1.5); 

		mParamCur(0) = mParamGoal(0);
		mParamCur(1) = mMomentum(1) / 160;

		rewardTask = 0.8 * rewardMomentum;

		mTaskStage = 1;
		if(mRender) {
			std::cout << "current parameter: " << mParamCur.transpose() << std::endl;
			std::cout << "momentum: " << mMomentum.transpose() << " / " << momentumGoal.transpose() << " / " << mDiff.transpose() << " / " << rewardMomentum << std::endl;
		}
	}
	if(mCurrentFrameOnPhase >= 30 && mTaskStage == 1) {
		std::vector<std::pair<bool, Eigen::Vector3d>> contactsRef = getContactInfo(mReferenceManager->getBVHFrame(mCurrentFrameOnPhase).position);
		std::vector<std::pair<bool, Eigen::Vector3d>> contactsCur = getContactInfo(mCharacter->getPositions());

		for(int i = 0; i < contactsCur.size(); i++) {
			if(contactsRef[i].first && !contactsCur[i].first) {
				mCondiff += pow(std::max(0.0, (contactsCur[i].second)(1) - 0.07), 2);
			} else if(!contactsRef[i].first && contactsCur[i].first) {
				mCondiff += pow(std::max(0.0, (contactsRef[i].second)(1) - 0.07), 2);
			}
		}

		if(mCurrentFrameOnPhase >= 44) {

			double rewardContact = exp(-mCondiff * 5);
			if(rewardContact < 0.5) {
				mSaveTrajectory = false;
			} 

			rewardTask = rewardContact;
			mFitness.sumConstraint = rewardContact;

			mTaskStage = 2;

			if(mRender) {
				std::cout << "contact: " << mCondiff << " / " << rewardContact << std::endl;
				std::cout << "final parameter: " <<  mParamCur.transpose() << std::endl;
			}
		}
	} 
	return rewardTask;
	
}
void 
Controller::
updateFitness()
{
	Eigen::VectorXd pSave = mCharacter->getPositions();
	Eigen::VectorXd vSave = mCharacter->getVelocities();

	Frame pvTarget = mReferenceManager->getBVHFrame(mCurrentFrameOnPhase);
	Eigen::VectorXd posBVH = pvTarget.position;
	Eigen::VectorXd velBVH = pvTarget.velocity;

	std::vector<std::pair<bool, Eigen::Vector3d>> contactBVH = getContactInfo(posBVH);
	std::vector<std::pair<bool, Eigen::Vector3d>> contactCur = getContactInfo(pSave);

	double conDiff = 0;
	for(int i = 0; i < contactCur.size(); i++) {
		if(contactBVH[i].first || contactCur[i].first) {
			conDiff += pow(((contactCur[i].second)(1) - (contactBVH[i].second)(1)) * 5, 2);
		}
	}

	Eigen::VectorXd pAligned = pSave;
	std::vector<Eigen::VectorXd> pZeroAndCurrent;
	pZeroAndCurrent.push_back(mRootZero);
	pZeroAndCurrent.push_back(pSave.segment<6>(0));
	pZeroAndCurrent = align(pZeroAndCurrent, mReferenceManager->getBVHFrame(0).position);
	pAligned.segment<6>(0) = pZeroAndCurrent[1];
	Eigen::VectorXd v = mCharacter->getPositionDifferences(pSave, mPrevPositions) / TIME_STEP;

	Eigen::VectorXd posDiff = mCharacter->getPositionDifferences(posBVH, pAligned);
	Eigen::VectorXd velDiff = mCharacter->getVelocityDifferences(velBVH, v);
	//scaling
	for(int i =0 ; i < velBVH.rows(); i++) {
		if(i >= 3 && i <= 5)
			velDiff(i) = velDiff(i) / std::max(0.2, velBVH(i));
		else
			velDiff(i) = velDiff(i) / std::max(0.5, velBVH(i));
	}
	int nBNodes = mCharacter->getNumBodyNodes();
	for(int i = 0; i < nBNodes; i++) {
		std::string name = mCharacter->getBodyNode(i)->getName();
		int idx = mCharacter->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
		if(name.compare(Root) == 0 ) {
			posDiff.segment<3>(idx) *= 3;
			posDiff.segment<3>(idx + 3) *= 5;
			posDiff(4) *= 0.2;

			velDiff.segment<3>(idx) *= 3;
			velDiff.segment<3>(idx + 3) *= 5;
			velDiff(4) *= 0.2;

		}
	}

	double footSlide = 0;
	for(int i = 0; i < 2; i++) {
		Eigen::Vector3d f; 
		if(i == 0) {
			f = mCharacter->getBodyNode(LeftFoot)->getWorldTransform().translation();
			f += mCharacter->getBodyNode(LeftToe)->getWorldTransform().translation();
			f /= 2.0;
		} else if(i == 1) {
			f = mCharacter->getBodyNode(RightFoot)->getWorldTransform().translation();
			f += mCharacter->getBodyNode(RightToe)->getWorldTransform().translation();
			f /= 2.0;
		}
		bool contact = f(1) < 0.07;
		if(mPrevContactInfo[i].first && contact) {
			Eigen::Vector3d vecSlide = mPrevContactInfo[i].second - f;
			vecSlide(1) = 0;
			footSlide += vecSlide.dot(vecSlide);
		}
		mPrevContactInfo[i] = std::pair<bool, Eigen::Vector3d>(f(1) < 0.07, f);
	}

	mFitness.sumContact += conDiff;
	mFitness.sumPos += posDiff.dot(posDiff) / posDiff.rows();
	mFitness.sumVel += velDiff.dot(velDiff) / velDiff.rows();
	mFitness.sumSlide += footSlide;
	mCountTracking += 1;
}
std::vector<std::pair<bool, Eigen::Vector3d>> 
Controller::
getContactInfo(Eigen::VectorXd _pos) 
{
	Eigen::VectorXd posSave = mCharacter->getPositions();
	Eigen::VectorXd velSave = mCharacter->getVelocities();
	
	mCharacter->setPositions(_pos);

	std::vector<std::string> contact;
	contact.push_back(LeftToe);
	contact.push_back(RightToe);
	contact.push_back(LeftFoot);
	contact.push_back(RightFoot);

	std::vector<std::pair<bool, Eigen::Vector3d>> result;
	result.clear();
	for(int i = 0; i < contact.size(); i++) {
		Eigen::Vector3d p = mCharacter->getBodyNode(contact[i])->getWorldTransform().translation();
		if(p[1] < 0.07) {
			result.push_back(std::pair<bool, Eigen::Vector3d>(true, p));
		} else {
			result.push_back(std::pair<bool, Eigen::Vector3d>(false, p));
		}
	}

	mCharacter->setPositions(posSave);

	return result;
}
void
Controller::
updateReward()
{
	if(mIsParametric) {	
		std::vector<double> rewardTracking = getTrackingReward(mCharacter->getPositions(), mTargetPosition,
															   mCharacter->getVelocities(), mTargetVelocity, true);
		double rewardTime = exp(-pow(mActions[mControllableDof] - mTargetPhaseDelta, 2)*40);
		double rewardSpace = 0.5 * rewardTracking[0] + 0.3 * rewardTracking[1] + 0.2 * rewardTracking[2];
		double rewardTotal = 0.8 * rewardSpace + 0.2 * rewardTime;
		double rewardTask = getTaskReward();
	
		mRewardParts.clear();

		if(dart::math::isNan(rewardTotal)){
			mRewardParts.resize(mRewardLabels.size(), 0.0);
		}
		else {
			mRewardParts.push_back(rewardTotal);
			mRewardParts.push_back(10 * rewardTask);
			mRewardParts.push_back(rewardSpace);
			mRewardParts.push_back(rewardTime);
		}
	} else {
		std::vector<double> rewardTracking = getTrackingReward(mCharacter->getPositions(), mTargetPosition,
															   mCharacter->getVelocities(), mTargetVelocity, true);
		double rewardTime = exp(-pow((mActions[mControllableDof] - 1),2)*40);
		mRewardParts.clear();
		double rewardTotal = 0.85 * (0.4 * rewardTracking[0] + 0.3 * rewardTracking[1] + 0.1 * rewardTracking[2] + 0.2 * rewardTracking[3]) + 0.15 * rewardTime;
		if(dart::math::isNan(rewardTotal)){
			mRewardParts.resize(mRewardLabels.size(), 0.0);
		} else {
			mRewardParts.push_back(rewardTotal);
			mRewardParts.push_back(rewardTracking[0]);
			mRewardParts.push_back(rewardTracking[1]);
			mRewardParts.push_back(rewardTracking[3]);
			mRewardParts.push_back(rewardTime);
		}
	}
}
void
Controller::
updateTerminalInfo()
{	
	Eigen::Isometry3d curRootInv = mCharacter->getRootBodyNode()->getWorldTransform().inverse();
	double rootY = mCharacter->getBodyNode(0)->getTransform().translation()[1];

	Eigen::VectorXd posSave = mCharacter->getPositions();
	mCharacter->setPositions(mTargetPosition);

	Eigen::Isometry3d rootDiff = curRootInv * mCharacter->getRootBodyNode()->getWorldTransform();
	Eigen::AngleAxisd root_diff_aa(rootDiff.linear());
	double angle = std::fmod(root_diff_aa.angle()+M_PI, 2*M_PI)-M_PI;
	Eigen::Vector3d rootPosDiff = rootDiff.translation();

	mCharacter->setPositions(posSave);

	// check nan
	if(dart::math::isNan(mCharacter->getPositions()) || dart::math::isNan(mCharacter->getVelocities())){
		mIsNan = true;
		mIsTerminal = true;
		mTerminalReason = 1;
	} 
	//characterConfigration
	else if(!mRender && rootPosDiff.norm() > TERMINAL_ROOT_DIFF_THRESHOLD){
		mIsTerminal = true;
		mTerminalReason = 2;
	} else if(!mRender && rootY < TERMINAL_ROOT_HEIGHT_LOWER_LIMIT || rootY > TERMINAL_ROOT_HEIGHT_UPPER_LIMIT){
		mIsTerminal = true;
		mTerminalReason = 3;
	} else if(!mRender && std::abs(angle) > TERMINAL_ROOT_DIFF_ANGLE_THRESHOLD){
		mIsTerminal = true;
		mTerminalReason = 4;
	} else if(mCurrentFrame > mReferenceManager->getMotionLength() * TERMINAL_ITERATION) {
		mIsTerminal = true;
		mTerminalReason =  8;
	}

	if(mRender) {
		if(mIsTerminal) std::cout << mTimeElapsed << " " << mTerminalReason << std::endl;
	}

}
void 
Controller::
reset(bool _RSI)
{
	mIsTerminal = false;
	mIsNan = false;

	mWorld->reset();
	mCharacter->clearConstraintImpulses();
	mCharacter->clearInternalForces();
	mCharacter->clearExternalForces();

	if(_RSI && !mIsParametric) {
		mCurrentFrame = (int) dart::math::Random::uniform(0.0, mReferenceManager->getMotionLength()*0.9);
	} else {
		mCurrentFrame = 0;
	}
	mTimeElapsed = 0;
	mStartFrame = mCurrentFrame;
	mCurrentFrameOnPhase = mCurrentFrame;

	Frame pvTarget = mReferenceManager->getFrame(mCurrentFrame);
	mTargetPosition = pvTarget.position;
	mTargetVelocity = pvTarget.velocity;
	mTargetPhaseDelta = pvTarget.phaseDelta;	
	mPhaseDelta = mTargetPhaseDelta;

	mPDTargetPosition = mTargetPosition;
	mPDTargetVelocity = mTargetVelocity;

	mCharacter->setPositions(mTargetPosition);
	mCharacter->setVelocities(mTargetVelocity);

	if(mRender) {
		clearRecord();
		updateRecord();
	}

	clearEpisode();
	updateState();

	mPrevPositions = mTargetPosition;
	mPrevContactInfo.clear();
	for(int i = 0; i < 2; i++) {
		Eigen::Vector3d f; 
		if(i == 0) {
			f = mCharacter->getBodyNode(LeftFoot)->getWorldTransform().translation();
			f += mCharacter->getBodyNode(LeftToe)->getWorldTransform().translation();
			f /= 2.0;
		} else if(i == 1) {
			f = mCharacter->getBodyNode(RightFoot)->getWorldTransform().translation();
			f += mCharacter->getBodyNode(RightToe)->getWorldTransform().translation();
			f /= 2.0;
		}
		mPrevContactInfo.push_back(std::pair<bool, Eigen::Vector3d>(f(1) < 0.07, f));
	}
	if(mIsParametric) {
		mTrajectory.first.push_back(mCharacter->getPositions());
		mTrajectory.second.push_back(mCurrentFrameOnPhase);
	}

}
void 
Controller::
setAction(Eigen::VectorXd _action)
{
	mActions = _action;
}
void 
Controller::
setParamGoal(Eigen::VectorXd _parameter)
{
	mParamGoal = _parameter;
	mWorld->setGravity(exp(mParamGoal(0))*Eigen::Vector3d(0,-9.81, 0));
}

Eigen::VectorXd 
Controller::
getEndEffectorStatePosAndVel(Eigen::VectorXd _pos, Eigen::VectorXd _vel) {
	Eigen::VectorXd ret;
	dart::dynamics::BodyNode* root = mCharacter->getRootBodyNode();
	Eigen::Isometry3d curRootInv = root->getWorldTransform().inverse();

	Eigen::VectorXd posSave = mCharacter->getPositions();
	Eigen::VectorXd velSave = mCharacter->getVelocities();

	mCharacter->setPositions(_pos);
	mCharacter->setVelocities(_vel);

	ret.resize(mEndEffectors.size()*12 +15);

	for(int i=0;i<mEndEffectors.size();i++)
	{		
		Eigen::Isometry3d transform = curRootInv * mCharacter->getBodyNode(mEndEffectors[i])->getWorldTransform();
		ret.segment<9>(9*i) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
							   transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2), 
							   transform.translation();
	}

	for(int i=0;i<mEndEffectors.size();i++)
	{
	    int idx = mCharacter->getBodyNode(mEndEffectors[i])->getParentJoint()->getIndexInSkeleton(0);
		ret.segment<3>(9*mEndEffectors.size() + 3*i) << _vel.segment<3>(idx);
	}

	Eigen::Isometry3d transform = curRootInv * mCharacter->getRootBodyNode()->getWorldTransform();

	Eigen::Vector3d rootAngularVelRelative = curRootInv.linear() * mCharacter->getRootBodyNode()->getAngularVelocity();
	Eigen::Vector3d rootLinearVelRelative = curRootInv.linear() * mCharacter->getRootBodyNode()->getCOMLinearVelocity();

	ret.tail<15>() << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
					  transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2),
					  transform.translation(), rootAngularVelRelative, rootLinearVelRelative;

	// restore
	mCharacter->setPositions(posSave);
	mCharacter->setVelocities(velSave);

	return ret;
}
bool
Controller::
checkCollisionWithGround(std::string _bodyName){
	auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
	dart::collision::CollisionOption option;
	dart::collision::CollisionResult result;
	if(_bodyName == RightFoot){
		bool isCollide = collisionEngine->collide(mCGR.get(), mCGG.get(), option, &result);
		return isCollide;
	}
	else if(_bodyName == LeftFoot){
		bool isCollide = collisionEngine->collide(mCGL.get(), mCGG.get(), option, &result);
		return isCollide;
	}
	else if(_bodyName == RightToe){
		bool isCollide = collisionEngine->collide(mCGER.get(), mCGG.get(), option, &result);
		return isCollide;
	}
	else if(_bodyName == LeftToe){
		bool isCollide = collisionEngine->collide(mCGEL.get(), mCGG.get(), option, &result);
		return isCollide;
	}
	else if(_bodyName == RightHand){
		bool isCollide = collisionEngine->collide(mCGHR.get(), mCGG.get(), option, &result);
		return isCollide;
	}
	else if(_bodyName == LeftHand){
		bool isCollide = collisionEngine->collide(mCGHL.get(), mCGG.get(), option, &result);
		return isCollide;
	}
	else{ // error case
		std::cout << "check collision : bad body name" << std::endl;
		return false;
	}
}
void
Controller::
updateState()
{
	if(mIsTerminal && mTerminalReason != 8){
		mStates = Eigen::VectorXd::Zero(mNumState);
		return;
	}	
	double rootHeight = mCharacter->getRootBodyNode()->getCOM()[1];

	Eigen::VectorXd p,v;
	v = mCharacter->getVelocities();
	int posDim = (mCharacter->getNumBodyNodes() - 1) * 6;
	p.resize(posDim);

	for(int i = 1; i < mCharacter->getNumBodyNodes(); i++){
		Eigen::Isometry3d transform = mCharacter->getBodyNode(i)->getRelativeTransform();
		p.segment<6>(6*(i-1)) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
								 transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2);
	}

	dart::dynamics::BodyNode* root = mCharacter->getRootBodyNode();
	Eigen::Isometry3d curRootInv = root->getWorldTransform().inverse();
	Eigen::VectorXd ee;
	ee.resize(mEndEffectors.size() * 3);
	for(int i = 0; i < mEndEffectors.size(); i++)
	{
		Eigen::Isometry3d transform = curRootInv * mCharacter->getBodyNode(mEndEffectors[i])->getWorldTransform();
		ee.segment<3>(3*i) << transform.translation();
	}
	Frame pvTarget = mReferenceManager->getFrame(mCurrentFrame+mTargetPhaseDelta);

	Eigen::VectorXd pNext = getEndEffectorStatePosAndVel(pvTarget.position, pvTarget.velocity);
	Eigen::Vector3d upvec = root->getTransform().linear()*Eigen::Vector3d::UnitY();
	double upvecAngle = atan2(std::sqrt(upvec[0]*upvec[0]+upvec[2]*upvec[2]),upvec[1]);
	

	if(mIsParametric) {
		mStates << p, v, upvecAngle, rootHeight, ee, pNext, mPhaseDelta, mCurrentFrameOnPhase, mParamGoal;
	} else {
		mStates << p, v, upvecAngle, rootHeight, ee, pNext, mPhaseDelta, mCurrentFrameOnPhase;
	}

}
}
