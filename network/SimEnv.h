#ifndef __SIM_ENV_H__
#define __SIM_ENV_H__
#include "Controller.h"
#include "ReferenceManager.h"
#include "EliteSet.h"
#include <vector>
#include <string>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace p = boost::python;
namespace np = boost::python::numpy;
class SimEnv
{
public:
	
	SimEnv(int _nslave, std::string _directory, std::string _ref, bool _parametric, bool _loadParam);

	//For general properties
	int getNumState();
	int getNumAction();
	double getMotionLength();
	p::list getRewardLabels();

	//For each slave
	void step(int _id);
	void reset(int _id, bool _RSI);
	void setAction(np::ndarray _array, int _id);
	p::tuple getStepInfo(int _id);
	np::ndarray getState(int _id);
	np::ndarray getRewardVector(int _id);

	//For all slaves
	void stepAll();
	void resetAll(bool _RSI);
	void setActionAll(np::ndarray _array);
	void setWeightAll(double _weight);
	np::ndarray getStateAll();
	np::ndarray getRewardVectorAll();
	
	int getNumParam();
	void setParamGoal(np::ndarray _array);
	int getExplorationRate();
	void saveParamSpace(int _i);
	np::ndarray uniformSample(int _visited);
	double getVisitedRatio();

	void trainParamNetwork();

private:
	std::vector<SIM::Controller*> mSlaves;
	SIM::ReferenceManager* mReferenceManager;
	SIM::EliteSet* mEliteSet;
	p::object mParamNet;

	int mNumSlave;
	int mNumAction;
	int mNumState;
	int mNumParam;

	double mLossParam;
	bool mBootstrap;

	std::string mPath;
};


#endif