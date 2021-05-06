#ifndef __SIM_ELITE_SET_H__
#define __SIM_ELITE_SET_H__
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <random>
#include "ParamTree.h"
namespace SIM
{
struct Fitness
{
	double sumPos;
	double sumVel;
	double sumContact;
	double sumSlide;
	double sumConstraint;
};
class EliteSet
{
public:
	
	EliteSet(int _dof, int _motionLength);
	void setConfigurations();
	void saveParamSpace(std::string _path);
	void loadParamSpace(std::string _path);

	Eigen::VectorXd uniformSample(int _visited);
	bool updateEliteSet(std::vector<Eigen::VectorXd> _displacement, Fitness _f, Eigen::VectorXd _parameter);

	void setParamGoal(Eigen::VectorXd _p);

	double getDensity(Eigen::VectorXd _pn, bool _old=false);
	double getExploredRatio();
	int getNumParam() {return mNumParam; }
	Eigen::VectorXd getParamGoal() {return mParamGoal; }
	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> getTrainingData();
	std::vector<ParamNode*> traverseTree() { return mParamTree.traverse(); }
	std::vector<Eigen::VectorXd> getWeightedKNN(Eigen::VectorXd _param);
	int getExplorationRate() { return mExplorationRate; }
	double getVisitedRatio();

	Eigen::VectorXd normalize(Eigen::VectorXd _p);
	Eigen::VectorXd denormalize(Eigen::VectorXd _pn);
private:
	ParamTree mParamTree;
	ParamNode* mParamBVH;
	int mNumSamples;

	Eigen::VectorXd mParamScale;
	Eigen::VectorXd mParamScaleInv;
	Eigen::VectorXd mParamUnit;
	Eigen::VectorXd mParamGoal;
	Eigen::VectorXd mParamMin;
	Eigen::VectorXd mParamMax;

	double mRadiusNeighbor;
	double mThresholdIn;
	double mThresholdOut;
	int mThresholdOld;
	int mNumBlending;
	int mNumParam;
	int mNumDof;
	int mMotionLength;

	int mExplorationRate;

	std::random_device mRD;
	std::mt19937 mMT;
	std::uniform_real_distribution<double> mUniform;
};
}
#endif