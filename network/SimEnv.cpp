#include "SimEnv.h"
#include <omp.h>
#include "dart/math/math.hpp"
#include "Functions.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
SimEnv::
SimEnv(int _nslave, std::string _directory, std::string _ref, bool _parametric, bool _loadParam) :mNumSlave(_nslave), mBootstrap(true)
{
	dart::math::seedRand();
	omp_set_num_threads(mNumSlave);

	std::string skelPath = std::string(PM_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
	std::string motionPath = std::string(PM_DIR) + std::string("/data/motion/") + _ref;
	mPath = _directory;
    mReferenceManager = new SIM::ReferenceManager(SIM::SkeletonBuilder::buildFromFile(skelPath).first);
	mReferenceManager->loadMotionFromBVH(motionPath);
	
	if(_parametric) {
		mEliteSet = new SIM::EliteSet(mReferenceManager->getNumDof(), mReferenceManager->getMotionLength());
		mEliteSet->setConfigurations();
		if(_loadParam)
			mEliteSet->loadParamSpace(mPath + "param_space");

		mReferenceManager->setEliteSet(mEliteSet);
		mNumParam = mEliteSet->getNumParam();

		Py_Initialize();
		np::initialize();
		try{
			p::object regression = p::import("regression");
			mParamNet = regression.attr("Regression")();
			mParamNet.attr("init_train")(mPath, mNumParam + 2, mReferenceManager->getNumDof(), _loadParam);
			mLossParam = 999999;
		}
		catch (const p::error_already_set&)
		{
			PyErr_Print();
		}	

	}

	for(int i = 0 ;i < mNumSlave; i++)
	{
		if(_parametric)
			mSlaves.push_back(new SIM::Controller(mReferenceManager, _parametric, false, mNumParam, i));
		else
			mSlaves.push_back(new SIM::Controller(mReferenceManager, _parametric, false, 0, i));
	}
	mNumAction = mSlaves[0]->getNumAction();
	mNumState = mSlaves[0]->getNumState();
}
//For general properties
int
SimEnv::
getNumState()
{
	return mNumState;
}
int
SimEnv::
getNumAction()
{
	return mNumAction;
}
double
SimEnv::
getMotionLength()
{
	return mReferenceManager->getMotionLength();
}
p::list 
SimEnv::
getRewardLabels()
{
	p::list l;
	std::vector<std::string> sl = mSlaves[0]->getRewardLabels();
	for(int i =0 ; i < sl.size(); i++) l.append(sl[i]);
	return l;
}
//For each slave
void 
SimEnv::
step(int _id)
{
	mSlaves[_id]->step();
}
void 
SimEnv::
reset(int _id,bool _RSI)
{
	mSlaves[_id]->reset(_RSI);
}
p::tuple 
SimEnv::
getStepInfo(int _id)
{
	// is_terminal, nan_occur, nt_elapsed, t_elapsed
	bool t = mSlaves[_id]->isTerminalState();
	bool n = mSlaves[_id]->isNan();
	int start = mSlaves[_id]->getStartFrame();
	double cur = mSlaves[_id]->getCurrentFrame();
	double te = mSlaves[_id]->getTimeElapsed();

	return p::make_tuple(t, n, cur - start, te);
}
np::ndarray
SimEnv::
getState(int _id)
{
	return SIM::toNumPyArray(mSlaves[_id]->getState());
}
void 
SimEnv::
setAction(np::ndarray _array, int _id)
{
	mSlaves[_id]->setAction(SIM::toEigenVector(_array, mNumAction));
}
np::ndarray
SimEnv::
getRewardVector(int _id)
{
	std::vector<double> ret = mSlaves[_id]->getRewardParts();

	return SIM::toNumPyArray(ret);
}
void
SimEnv::
stepAll()
{
#pragma omp parallel for
	for (int id = 0; id < mNumSlave; id++)
	{
		step(id);
	}
}
void
SimEnv::
resetAll(bool _RSI)
{
	for (int id = 0; id < mNumSlave; id++)
	{
		reset(id, _RSI);
	}
}
np::ndarray
SimEnv::
getStateAll()
{
	Eigen::MatrixXd states(mNumSlave, mNumState);

	for (int id = 0; id < mNumSlave; id++)
	{
		states.row(id) = mSlaves[id]->getState().transpose();
	}
	return SIM::toNumPyArray(states);
}
void
SimEnv::
setActionAll(np::ndarray np_array)
{
	Eigen::MatrixXd action = SIM::toEigenMatrix(np_array, mNumSlave, mNumAction);

	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setAction(action.row(id).transpose());
	}
}
void 
SimEnv::
setWeightAll(double _weight) 
{
	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setPhaseDeltaWeight(_weight);
	}
}
np::ndarray
SimEnv::
getRewardVectorAll()
{
	std::vector<std::vector<double>> rewards(mNumSlave);
	for (int id = 0; id < mNumSlave; ++id)
	{
		rewards.push_back(mSlaves[id]->getRewardParts());
	}

	return SIM::toNumPyArray(rewards);
}
int 
SimEnv::
getNumParam()
{
	return mNumParam;
}
void
SimEnv::
setParamGoal(np::ndarray _array) {
	Eigen::VectorXd tp = SIM::toEigenVector(_array, mNumParam);
	
	p::object loss_p = mParamNet.attr("random")();
	double r = SIM::toEigenVector(np::from_object(loss_p), 1)(0);

	std::vector<Eigen::VectorXd> displacements;
	if(r > 0.3 || mBootstrap) {
		displacements = mEliteSet->getWeightedKNN(tp);
	} else {
		std::cout << "from param network" << std::endl;
		Eigen::VectorXd ntp = mEliteSet->normalize(tp);
		std::vector<Eigen::VectorXd> inputMatrix;
		double A = mReferenceManager->getMotionLength() / 2.0;
		for(int i = 0; i < mReferenceManager->getMotionLength(); i++) {
			Eigen::VectorXd input(mNumParam + 2);
			input << sin(i / A * M_PI), cos(i / A * M_PI), ntp;
			inputMatrix.push_back(input);
		}
		p::object out = mParamNet.attr("run")(SIM::toNumPyArray(inputMatrix));
		np::ndarray nout = np::from_object(out);
		Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, mReferenceManager->getMotionLength(), mReferenceManager->getNumDof());
		for(int i = 0; i < mReferenceManager->getMotionLength(); i++) {
			displacements.push_back(outputMatrix.row(i));
		}
	}
	mReferenceManager->loadMotionFromDisplacement(displacements);

	for(int id = 0; id < mNumSlave; ++id) {
		mSlaves[id]->setParamGoal(tp);
	}
	mEliteSet->setParamGoal(tp);

}
int 
SimEnv::
getExplorationRate() {
	return mEliteSet->getExplorationRate();
}
void
SimEnv::
saveParamSpace(int _i) {
	if(_i != -1) {
		mEliteSet->saveParamSpace(mPath + "param_space" + std::to_string(_i));
	} else {
		mEliteSet->saveParamSpace(mPath + "param_space");
	}
}
np::ndarray 
SimEnv::
uniformSample(int _visited) {
	Eigen::VectorXd goal = mEliteSet->uniformSample(_visited);

	return SIM::toNumPyArray(goal);
}
double
SimEnv::
getVisitedRatio() {
	return mEliteSet->getVisitedRatio();
}
void
SimEnv::
trainParamNetwork() {
	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> data = mEliteSet->getTrainingData();

	if(data.first.size() == 0)
		return;

	np::ndarray x = SIM::toNumPyArray(data.first);
	np::ndarray y = SIM::toNumPyArray(data.second);

	p::list l;
	l.append(x);
	l.append(y);

	mParamNet.attr("set_training_data")(l);
	p::object loss_p = mParamNet.attr("train")();
	double loss = SIM::toEigenVector(np::from_object(loss_p), 1)(0);
	if(loss < 5*1e-3 || abs(loss - mLossParam) < 5*1e-5) {
		if(mBootstrap)
			std::cout << "bootstrap off" << std::endl;
		mBootstrap = false;
	}
	else {
		if(!mBootstrap)
			std::cout << "bootstrap on" << std::endl;
		mBootstrap = true;
	}
	mLossParam = loss;

}
using namespace boost::python;

BOOST_PYTHON_MODULE(simEnv)
{
	Py_Initialize();
	np::initialize();

	class_<SimEnv>("Env", init<int, std::string, std::string, bool, bool>())
		.def("getNumState",&SimEnv::getNumState)
		.def("getNumAction",&SimEnv::getNumAction)
		.def("getMotionLength",&SimEnv::getMotionLength)
		.def("getRewardLabels",&SimEnv::getRewardLabels)
		.def("step",&SimEnv::step)
		.def("reset",&SimEnv::reset)
		.def("getState",&SimEnv::getState)
		.def("setAction",&SimEnv::setAction)
		.def("getStepInfo",&SimEnv::getStepInfo)
		.def("getRewardVector",&SimEnv::getRewardVector)
		.def("stepAll",&SimEnv::stepAll)
		.def("resetAll",&SimEnv::resetAll)
		.def("getStateAll",&SimEnv::getStateAll)
		.def("setActionAll",&SimEnv::setActionAll)
		.def("setWeightAll",&SimEnv::setWeightAll)
		.def("getNumParam",&SimEnv::getNumParam)
		.def("setParamGoal",&SimEnv::setParamGoal)
		.def("getExplorationRate",&SimEnv::getExplorationRate)
		.def("saveParamSpace",&SimEnv::saveParamSpace)
		.def("getVisitedRatio",&SimEnv::getVisitedRatio)
		.def("uniformSample",&SimEnv::uniformSample)
		.def("trainParamNetwork",&SimEnv::trainParamNetwork);

}