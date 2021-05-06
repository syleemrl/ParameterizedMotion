#ifndef __SIM_CONTROLLER_H__
#define __SIM_CONTROLLER_H__
#include "dart/dart.hpp"
#include <queue>
#include "ReferenceManager.h"
namespace SIM
{
class Controller
{
public:
	Controller(ReferenceManager* _ref, bool _parametric=false, bool _render=false, 
			   int _numParam=0, int _id=0);

	void step();
	void stepBVH();
	void reset(bool _RSI=true);
	
	void setAction(Eigen::VectorXd _action);
	void setPhaseDeltaWeight(double _weight) { mPhaseDeltaWeight = _weight; }
	Eigen::VectorXd getState() {return mStates; }
	std::vector<double> getRewardParts() {return mRewardParts; }
	std::vector<std::string> getRewardLabels() {return mRewardLabels; }
	
	int getNumState() {return mNumState; }
	int getNumAction() {return mNumAction; }
	
	double getTimeElapsed(){return mTimeElapsed;}
	double getCurrentFrame(){return mCurrentFrame;}
	double getCurrentFrameOnPhase(){return mCurrentFrameOnPhase;}
	double getStartFrame(){ return mStartFrame; }
	
	bool isTerminalState() {return mIsTerminal; }
	bool isNan() {return mIsNan;}

	// renderer
	Eigen::VectorXd getPosition(int _idx) { return mRecordPosition[_idx]; }
	Eigen::VectorXd getBVHPosition(int _idx) { return mRecordBVHPosition[_idx]; }
	Eigen::VectorXd getTargetPosition(int _idx) { return mRecordTargetPosition[_idx]; }
	double getPhase(int _idx) { return mRecordPhase[_idx]; }
	std::pair<bool, bool> getFootContact(int _idx) { return mRecordFootContact[_idx]; }

	void setParamGoal(Eigen::VectorXd _parameter);
protected:
	Eigen::VectorXd getEndEffectorStatePosAndVel(Eigen::VectorXd _pos, Eigen::VectorXd _vel);
	void updateReward();
	void updateState();
	void updateTerminalInfo();

	std::vector<double> getTrackingReward(Eigen::VectorXd _posCur, Eigen::VectorXd _posTarget, 
										  Eigen::VectorXd _velCur, Eigen::VectorXd _velTarget, 
										  bool _useVelocity);
	double getTaskReward();
	void updateFitness();

	std::vector<std::pair<bool, Eigen::Vector3d>> getContactInfo(Eigen::VectorXd _pos);

	bool checkCollisionWithGround(std::string bodyName);

	void updateRecord();
	void clearRecord();
	void clearEpisode();
	
	ReferenceManager* mReferenceManager;
	dart::simulation::WorldPtr mWorld;
	dart::dynamics::SkeletonPtr mGround;
	dart::dynamics::SkeletonPtr mCharacter;
	std::map<std::string, double> mTorqueMap;
	
	int mId;
	bool mIsParametric;
	bool mRender;

	double mStartFrame;
	double mCurrentFrame;
	double mTimeElapsed;
	double mCurrentFrameOnPhase;

	int mControlHz;
	int mSimulationHz;
	int mSimPerCon;
	
	Eigen::VectorXd mTargetPosition;
	Eigen::VectorXd mTargetVelocity;
	double mTargetPhaseDelta;

	Eigen::VectorXd mPDTargetPosition;
	Eigen::VectorXd mPDTargetVelocity;

	Eigen::VectorXd mActions;
	Eigen::VectorXd mStates;

	int mControllableDof;
	int mTotalDof;
	
	bool mIsTerminal;
	bool mIsNan;
	int mTerminalReason;

	std::vector<std::string> mEndEffectors;
	std::vector<std::string> mRewardLabels;
	std::vector<double> mRewardParts;

	std::unique_ptr<dart::collision::CollisionGroup> mCGEL, mCGER, mCGL, mCGR, mCGG, mCGHR, mCGHL; 

	std::vector<Eigen::VectorXd> mRecordPosition;
	std::vector<Eigen::VectorXd> mRecordBVHPosition;
	std::vector<Eigen::VectorXd> mRecordTargetPosition;
	std::vector<std::pair<bool, bool>> mRecordFootContact;
	std::vector<double> mRecordPhase;
	Eigen::VectorXd mPrevPositions;

	int mNumState;
	int mNumAction;

	double mPhaseDelta;
	double mPhaseDeltaWeight;

	Fitness mFitness;
	Eigen::VectorXd mParamGoal;
	Eigen::VectorXd mParamCur;
	std::pair<std::vector<Eigen::VectorXd>, std::vector<double>> mTrajectory;

	int mTaskStage;
	int mCountTracking;
	Eigen::Vector6d mRootZero;
	std::vector<std::pair<bool, Eigen::Vector3d>> mPrevContactInfo;

/////////////////////////////////////////////////
// for action parameter design
	bool mSaveTrajectory;
	double mTakeOffVelocity;
	Eigen::Vector3d mMomentum;
	double mCondiff;
//////////////////////////////////////////////////
};
}
#endif
