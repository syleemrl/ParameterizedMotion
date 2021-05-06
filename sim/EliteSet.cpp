#include "EliteSet.h"
#include "Functions.h"
#include <iostream>
#include <fstream>
#include <algorithm>
namespace SIM
{

EliteSet::
EliteSet(int _dof, int _motionLength) :mRD(), mMT(mRD()), mUniform(0.0, 1.0), mNumDof(_dof), mMotionLength(_motionLength) 
{}
void 
EliteSet::
setConfigurations()
{
	mNumSamples = 1;

	mRadiusNeighbor = 0.015;
	mThresholdIn = 0.7;
	mThresholdOut = 0.3;
	mThresholdOld = 5;

	mNumParam = 2;
	mNumBlending = 5;

	mParamMin.resize(mNumParam);
	mParamMax.resize(mNumParam);
	mParamUnit.resize(mNumParam);
	mParamScale.resize(mNumParam);
	mParamScaleInv.resize(mNumParam);

	mParamMin << -1.5, 0.8;
	mParamMax << 0.5, 1.8;
	mParamUnit << 2.0, 1.0;

	for(int i = 0 ; i < mNumParam; i++) {
		mParamScaleInv[i] = (mParamMax[i] - mParamMin[i]);
		mParamScale[i] = 1 / mParamScaleInv[i];
	}

	Eigen::VectorXd pBVH(mNumParam);
	pBVH << 0, 1;
	mParamGoal = pBVH;
	pBVH = normalize(pBVH);

	std::vector<Eigen::VectorXd> dBVH;
	for(int i = 0; i < mMotionLength; i++) {
		Eigen::VectorXd d(mNumDof);
		d.setZero();
		dBVH.push_back(d);
	}
	
	mParamBVH = new ParamNode(pBVH, dBVH, 1, 0);
	mParamTree = ParamTree(mParamBVH, mParamUnit);

}
void 
EliteSet::
saveParamSpace(std::string _path)
{

	std::vector<ParamNode*> params = mParamTree.traverse();
	std::ofstream ofs(_path);
	
	ofs << params.size() << std::endl;

	for(int i = 0; i < params.size(); i++) {
		ofs << params[i]->getParamNormalized().transpose() << std::endl;
		ofs << params[i]->getFitness() << " " << params[i]->getNumUpdated() << std::endl;

		std::vector<Eigen::VectorXd> displacement = params[i]->getDisplacement(); 
		for(int j = 0; j < mMotionLength; j++) {
			ofs << displacement[j].transpose() << std::endl;
		}
	}

	ofs.close();
	std::cout << "save param space : " << params.size() << std::endl;

}
void 
EliteSet::
loadParamSpace(std::string _path)
{
	char buffer[256];

	std::ifstream is;
	is.open(_path);

	if(is.fail())
		return;

	is >> buffer;
	mNumSamples = atoi(buffer);
	std::cout << "load param space : " << mNumSamples << std::endl;

	std::vector<ParamNode*> params;
	for(int k = 0; k < mNumSamples; k++) {
		Eigen::VectorXd p(mNumParam);
		for(int i = 0; i < mNumParam; i++) {
			is >> buffer;
			p(i) = atof(buffer);
		}

		is >> buffer;
		double fitness = atof(buffer);

		is >> buffer;
		int updated = atoi(buffer);

		std::vector<Eigen::VectorXd> displacement;
		for(int i = 0; i < mMotionLength; i++) {
			Eigen::VectorXd d(mNumDof);
			for(int j = 0; j < mNumDof; j++) 
			{
				is >> buffer;
				d(j) = atof(buffer);
			}
			displacement.push_back(d);
		}

		ParamNode* param = new ParamNode(p, displacement, fitness, updated);
		params.push_back(param);
	}
	is.close();
	mParamTree.build(params);
}
Eigen::VectorXd 
EliteSet::
uniformSample(int _visited)
{
	int count = 0;
	bool warmup = false;
	while(1) {
		Eigen::VectorXd pn(mNumParam);
		for(int i = 0; i < mNumParam; i++) {
			pn(i) = mUniform(mMT);
		}
		
		if(_visited == -1) 
			return denormalize(pn);
		
		double density = getDensity(pn, true);

		if(!_visited) {
			double distMin = (Eigen::VectorXd::Zero(pn.rows()) - pn).dot(Eigen::VectorXd::Zero(pn.rows()) - pn);
			double distMax = (Eigen::VectorXd::Ones(pn.rows()) - pn).dot(Eigen::VectorXd::Ones(pn.rows()) - pn);

			if((distMin < 1e-2 || distMax < 1e-2) && density > mThresholdOut){
				continue;
			}
			if (density < mThresholdIn && density > mThresholdOut) {
				return denormalize(pn);
			}
		}
		if(_visited && density > mThresholdIn) {
			return denormalize(pn);
		}

		count += 1;
		if(count > 100000 && !_visited) {
			warmup = true;
			break;
		} else if(count > 100000 && _visited) {
			return denormalize(pn);
		}
	}
	if(warmup) {
		while(1) {
			Eigen::VectorXd pn(mNumParam);
			for(int i = 0; i < mNumParam; i++) {
				pn(i) = mUniform(mMT);
			}
			
			double density = getDensity(pn, true);
			if(density > 0.05 && density < mThresholdIn) {
				return denormalize(pn);
			} 
		}
	}
}
bool 
EliteSet::
updateEliteSet(std::vector<Eigen::VectorXd> _displacement, Fitness _f, Eigen::VectorXd _parameter)
{

	for(int i = 0; i < mNumParam; i++) {
		if(_parameter(i) > mParamMax(i) || _parameter(i) < mParamMin(i)) {
			return false;
		}
	}
	double fContact = exp(-_f.sumContact); 
	double fVel = exp(-_f.sumVel*0.01);
	double fPos = exp(-_f.sumPos*8);
	double fSlide = exp(-_f.sumSlide * 7);
	double fitness = fPos * fVel * fSlide * fContact;
	fitness = 0.9 * fitness + 0.1 * _f.sumConstraint;
	if(fitness < 0.1) {
		return false;
	}

	Eigen::VectorXd paramNormalized = normalize(_parameter);

	bool flag = true;
	std::vector<std::pair<double, ParamNode*>> candidates = mParamTree.getNeighborNodes(paramNormalized, mRadiusNeighbor);

	std::vector<ParamNode*> deletedParams;

	int numUpdate = mThresholdOld;
	double prevFitnessMax = 0;
	for(int i = 0 ; i < candidates.size(); i++) {
		if(candidates[i].second->getNumUpdated() > 0)
			candidates[i].second->decreaseNumUpdated();
		
		if(prevFitnessMax < candidates[i].second->getFitness())
			prevFitnessMax = candidates[i].second->getFitness();
		if(candidates[i].second->getFitness() < fitness) {
			deletedParams.push_back(candidates[i].second);
		if(numUpdate > candidates[i].second->getNumUpdated() || numUpdate == mThresholdOld)
			numUpdate = candidates[i].second->getNumUpdated();
		} else {
			flag = false;
			break;
		}
		if(!flag)
			break;
	}
	if(flag) {
		double d = getDensity(paramNormalized, true);

		if(d > mThresholdIn) {
			std::vector<std::pair<double, ParamNode*>> neighbors = mParamTree.getNeighborNodes(paramNormalized, 2 * mRadiusNeighbor);
			if(neighbors.size() != 0) 
			{
				double fitnessMean = 0;
				for(int i = 0; i < neighbors.size(); i++) {
					fitnessMean += neighbors[i].second->getFitness() / neighbors.size();
					if(numUpdate > neighbors[i].second->getNumUpdated() || numUpdate == mThresholdOld)
						numUpdate = neighbors[i].second->getNumUpdated();
				}
				prevFitnessMax = fitnessMean;
				if(fitnessMean > fitness + 0.05) {
					return false;
				}
			}
		}

		while(!deletedParams.empty()) {
			ParamNode* d = deletedParams.back();
			deletedParams.pop_back();
			mParamTree.deleteNode(d);
			mNumSamples -= 1;
		}
		ParamNode* p = new ParamNode(paramNormalized, _displacement, fitness, numUpdate);

	 	mParamTree.insertNode(p);
	 	mNumSamples += 1;
	 	double dist = mParamTree.getDistance(paramNormalized, normalize(mParamGoal));
		if(dist < 0.15 && (fitness >= prevFitnessMax + 0.02)) {
			mExplorationRate += 1;
		}
		std::cout << "#" << _parameter.transpose() << " / " << fitness<< " / " << numUpdate << " / " << dist << std::endl;
	} 
	return flag;
}
void 
EliteSet::
setParamGoal(Eigen::VectorXd _p)
{
	mParamGoal = _p; 
	mExplorationRate = 0;
}
double 
EliteSet::
getDensity(Eigen::VectorXd _pn, bool _old)
{
	double density = 0;
	std::vector<std::pair<double, ParamNode*>> distAndParams = mParamTree.getNeighborNodes(_pn, 0.2);
	for(int i = 0; i < distAndParams.size(); i++) {
		if(_old && distAndParams[i].second->getNumUpdated()) 
			continue;
		density += 0.1 * exp( - pow(distAndParams[i].first, 2) * 500);	
	}
	return density;
}
double 
EliteSet::
getExploredRatio()
{
	Eigen::VectorXd base(mNumParam);
	base = mParamUnit * 0.05;
	std::vector<Eigen::VectorXd> grids;

	grids.push_back(base);
	for(int i = 0; i < mNumParam; i++) {
		std::vector<Eigen::VectorXd> gridsTemp;
	
		double j = 1;
		while(j * mParamUnit(i) * 0.1 < 1) {
			for(int k = 0; k < grids.size(); k++) {
				Eigen::VectorXd p = grids[k];
				p(i) = j * mParamUnit(i) * 0.1;
				gridsTemp.push_back(p);
			}
			j += 0.5;
		}
		for(int j = 0; j < gridsTemp.size(); j++) {
			grids.push_back(gridsTemp[j]);
		}	
	}

	double tot = grids.size();
	double result = 0;
	for(int i = 0; i < grids.size(); i++) {
		if(getDensity(grids[i], true) > mThresholdOut)
			result += 1;
	}
	result /= tot;

	return result;
}
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>
EliteSet::
getTrainingData()
{
	std::vector<Eigen::VectorXd> x;
	std::vector<Eigen::VectorXd> y;
	
	std::vector<ParamNode*> params = mParamTree.traverse();

	for(int i = 0; i < params.size(); i++) {
		if(!params[i]->getNumUpdated()) {
			std::vector<Eigen::VectorXd> displacement = params[i]->getDisplacement(); 
			double A = mMotionLength / 2.0;
			for(int j = 0; j < mMotionLength; j++) {
				Eigen::VectorXd xElem(mNumParam + 2);
				xElem << sin(j / A * M_PI), cos(j / A * M_PI), params[i]->getParamNormalized();
				x.push_back(xElem);
				y.push_back(displacement[j]);
			}
		}
	}
	return std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>(x, y);
}
std::vector<Eigen::VectorXd> 
EliteSet::
getWeightedKNN(Eigen::VectorXd _param)
{
	std::vector<std::pair<double, ParamNode*>> ps = mParamTree.getKNearestNodes(normalize(_param), 50);
	std::vector<std::pair<double, ParamNode*>> psElite;
	for(int i = 0 ; i < ps.size(); i++) {
		if(!ps[i].second->getNumUpdated())
			psElite.push_back(ps[i]);
	}
	if(psElite.size() <= mNumBlending)
		return mParamBVH->getDisplacement();

	for(int i = 0; i < psElite.size(); i++) {
		psElite[i].first = psElite[i].second->getFitness()*exp(-pow(psElite[i].first,2)*12);
	}

	std::stable_sort(psElite.begin(), psElite.end(), 
		[] (std::pair<double, ParamNode*> i, std::pair<double, ParamNode*> j) { 
		return (i.first > j.first); 
	});

	std::vector<Eigen::VectorXd> meanDisplacement;   
 	std::vector<Eigen::Matrix3d> meanRootDisplacement;   

   	for(int i = 0; i < mMotionLength; i++) {
		meanDisplacement.push_back(Eigen::VectorXd::Zero(mNumDof));
		Eigen::Matrix3d m;
		m.setZero();
		meanRootDisplacement.push_back(m);
	}

	double weightSum = 0;
	for(int i = 0; i < mNumBlending; i++) {
		double w = psElite[i].first;
		weightSum += w;
	    std::vector<Eigen::VectorXd> displacement = psElite[i].second->getDisplacement();

	    for(int j = 0; j < mMotionLength; j++) {

			meanDisplacement[j] += w * displacement[j];
			meanRootDisplacement[j] += w * expMapRot(displacement[j].segment<3>(0));
	    }
	}
	for(int i = 0; i < mNumBlending; i++) {
		double w = psElite[i].first;
	}
	for(int i = 0; i < mMotionLength; i++) {
	    meanDisplacement[i] /= weightSum;
	    meanRootDisplacement[i] /= weightSum;
	    meanDisplacement[i].segment<3>(0) = logMap(meanRootDisplacement[i]);
	}

	return meanDisplacement;
}
Eigen::VectorXd 
EliteSet::
normalize(Eigen::VectorXd _p) 
{
	return (_p - mParamMin).cwiseProduct(mParamScale);
}
Eigen::VectorXd 
EliteSet::
denormalize(Eigen::VectorXd _pn) 
{
	return _pn.cwiseProduct(mParamScaleInv) + mParamMin;
}
double
EliteSet::
getVisitedRatio()
{
	Eigen::VectorXd base(mNumParam);
	base = Eigen::VectorXd::Ones(mNumParam) * 0.05;
	std::vector<Eigen::VectorXd> vecs;

	vecs.push_back(base);
	for(int i = 0; i < mNumParam; i++) {
		std::vector<Eigen::VectorXd> v;
	
		double range = 1;
		double j = 0.1;
		while(j < range) {
			for(int k = 0; k < vecs.size(); k++) {
				Eigen::VectorXd p = vecs[k];
				p(i) = j;
				v.push_back(p);
			}
			j += 0.05;
		}
		for(int j = 0; j < v.size(); j++) {
			vecs.push_back(v[j]);
		}	
	}

	double tot = vecs.size();
	double result = 0;
	for(int i = 0; i < vecs.size(); i++) {
		if(getDensity(vecs[i]) > 0.3)
			result += 1;
	}
	result /= tot;

	return result;
}
};