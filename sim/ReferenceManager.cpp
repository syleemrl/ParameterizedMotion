#include "ReferenceManager.h"
#include "Functions.h"
#include "BVHParser.h"
#include "Configurations.h"
namespace SIM
{
ReferenceManager::
ReferenceManager(dart::dynamics::SkeletonPtr _skel) 
:mSkel(_skel), mBlendingInterval(2), mBlend(false), mFixRootRot(false), mFixRootPos(false), mMotionLength(0)
{}
Eigen::VectorXd 
ReferenceManager::
computeVelocity(bool _smooth, Eigen::VectorXd _p2, Eigen::VectorXd _p1, Eigen::VectorXd _p0)
{
	Eigen::VectorXd vel;
	if(_smooth) {
		vel = 0.5 * mSkel->getPositionDifferences(_p2, _p1) / TIME_STEP 
		      +  0.5 * mSkel->getPositionDifferences(_p2, _p0) / (TIME_STEP*2);
	} else {
		vel = mSkel->getPositionDifferences(_p2, _p1) / TIME_STEP;
	}
	return vel;
}
void 
ReferenceManager::
loadMotionFromBVH(std::string _bvh)
{
	mMotionBVH.clear();
	std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>> bvhInfo = BVHParser::parse(_bvh);
	std::vector<std::pair<std::string, int>> idxList = BVHParser::getNodeIdx(std::get<0>(bvhInfo)); 
	double timeStep = std::get<1>(bvhInfo);
	int dof = mSkel->getNumDofs();

	double t = 0;

	int count = 0;
	for(int i = 0; i < std::get<2>(bvhInfo).size(); i++)
	{
		bool flag = false;
		Eigen::VectorXd	p;
		if(abs(timeStep - TIME_STEP) <= 1e-4) {
			p = std::get<2>(bvhInfo)[i];
			flag = true;
		}
		else if(t >= count * TIME_STEP) {

			if(i != 0) {
				Eigen::VectorXd p0 = std::get<2>(bvhInfo)[i-1];
				Eigen::VectorXd p1 = std::get<2>(bvhInfo)[i];

				double t0 = t - timeStep;
				double t1 = t;

				p = weightedSumPos(p0, p1, (count * TIME_STEP - t0) / (t1 - t0));
			} else {
				p = std::get<2>(bvhInfo)[i];
			}
			flag = true;
		}

		if(flag) {
			Eigen::VectorXd pos(dof);
			for(int j = 0; j < idxList.size(); j++) {
				dart::dynamics::BodyNode* bn = mSkel->getBodyNode(idxList[j].first);
				int idx = bn->getParentJoint()->getIndexInSkeleton(0);
				if(idx == 0) {
					pos.segment<3>(3) = p.segment<3>(0);
					pos.segment<3>(0) = p.segment<3>(3);
				} else {
					pos.segment<3>(idx) = p.segment<3>(idxList[j].second);
				}
			}
			Eigen::VectorXd vel = Eigen::VectorXd::Zero(dof);
			if(count >= 2) {
				vel = computeVelocity(false, pos, mMotionBVH[count - 1].position, mMotionBVH[count - 2].position);
			} else if(count == 1) {
				vel = computeVelocity(false, pos, mMotionBVH[0].position);
				mMotionBVH[0].velocity = vel;
			}
			mMotionBVH.push_back(Frame(pos, vel));
			count += 1;
		}
		if(t < count * TIME_STEP)
			t += timeStep;
	}
	mMotionLength = mMotionBVH.size();
	generateCycles(mMotionLength*TERMINAL_ITERATION, mMotionBVH, mMotionGen, 
								  mBlend, mFixRootRot, mFixRootPos);

	std::vector<Frame> mMotionGenBVH;
	generateCycles(mMotionLength*TERMINAL_ITERATION, mMotionBVH, mMotionGenBVH, 
								  mBlend, mFixRootRot, mFixRootPos);
	mMotionBVH = mMotionGenBVH;
}
void 
ReferenceManager::
generateCycles(int _numFrames, std::vector<Frame>& _clip, std::vector<Frame>& _gen,
							  bool _blend, bool _fixRootRot, bool _fixRootPos)
{
	_gen.clear();

	Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(_clip[0].position.head<6>());
	Eigen::Isometry3d T0Updated;

	int totalLength = _clip.size();
	for(int i = 0; i < _numFrames; i++) {
		int phase = i % totalLength;
		
		if(i < totalLength) {
			_gen.push_back(Frame(_clip[i]));
		} else {
			Eigen::VectorXd pos;
			if(phase == 0) {
				pos = _clip[0].position;
				Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(_gen.back().position.head<6>());
				Eigen::Isometry3d T01 = T1*T0.inverse();

				Eigen::Vector3d p01 = dart::math::logMap(T01.linear());			
				T01.linear() =  dart::math::expMapRot(projectToXZ(p01));
				T01.translation()[1] = 0;
				if(_fixRootPos) {
					T01.translation() = Eigen::Vector3d::Zero();
				}
				if(_fixRootRot) {
					T01.linear() = Eigen::Matrix3d::Identity();
				}
				T0Updated = T01*T0;
				pos.segment<6>(0) = dart::dynamics::FreeJoint::convertToPositions(T0Updated);
			}

			pos = _clip[phase].position;
			Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
			TCurrent = T0.inverse() * TCurrent;
			TCurrent = T0Updated * TCurrent;
			pos.segment<6>(0) = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
			Eigen::VectorXd vel = computeVelocity(false, pos, _gen[_gen.size() - 1].position, _gen[_gen.size() - 2].position);
			_gen.push_back(Frame(pos, vel, _clip[phase].phaseDelta));

			if(_blend && phase == 0) {
				int back = _gen.size();
				for(int j = mBlendingInterval; j > 0; j--) {
					double weight = 1 - j / (double)(mBlendingInterval+1);
					Eigen::VectorXd oldPos = _gen[back - j].position;
					_gen[back - j].position = weightedSumPos(oldPos, pos, weight);
					_gen[back - j].velocity = computeVelocity(false, _gen[back - j].position, _gen[back - j - 1].position, _gen[back - j - 2].position);
				}
			}
		}
	}
}
Frame 
ReferenceManager::
getBVHFrame(double _t)
{
	
	if(mMotionBVH.size()-1 < _t) {
	 	return mMotionBVH.back();
	}

	int k0 = (int) std::floor(_t);
	int k1 = (int) std::ceil(_t);	

	if (k0 == k1)
		return mMotionBVH[k0];
	else {
		return Frame(weightedSumPos(mMotionBVH[k0].position, mMotionBVH[k1].position, (_t-k0)), 
					 weightedSumVec(mMotionBVH[k0].velocity, mMotionBVH[k1].velocity, (_t-k0)),
					 (1- (_t-k0)) * mMotionBVH[k0].phaseDelta + (_t-k0) * mMotionBVH[k1].phaseDelta);		
	}
}
Frame 
ReferenceManager::
getFrame(double _t)
{

	if(mMotionGen.size()-1 < _t) {
	 	return mMotionGen.back();
	}
	
	int k0 = (int) std::floor(_t);
	int k1 = (int) std::ceil(_t);	

	if (k0 == k1)
		return mMotionGen[k0];
	else {
		return Frame(weightedSumPos(mMotionGen[k0].position, mMotionGen[k1].position, (_t-k0)), 
					 weightedSumVec(mMotionGen[k0].velocity, mMotionGen[k1].velocity, (_t-k0)),
					 (1- (_t-k0)) * mMotionGen[k0].phaseDelta + (_t-k0) * mMotionGen[k1].phaseDelta);		
	}
}
void 
ReferenceManager::
loadMotionFromDisplacement(std::vector<Eigen::VectorXd> _displacement)
{
	std::vector<Eigen::VectorXd> trajectory = getTrajectoryFromDisplacement(_displacement);

	int dof = trajectory[0].rows();

	std::vector<Frame> motion;
	for(int i = 0; i < mMotionLength; i++) {
		Eigen::VectorXd pos = trajectory[i].head(dof-1);
		Eigen::VectorXd vel(pos.rows());
		if(i >= 2) {
			vel = computeVelocity(false, pos, motion[i - 1].position, motion[i - 2].position);
		} else if( i == 1) {
			vel = computeVelocity(false, pos, motion[0].position);
			motion[0].velocity = vel;
		}
		double phaseDelta = trajectory[i](dof-1);
		motion.push_back(Frame(pos, vel, phaseDelta));
	}

	generateCycles(mMotionLength*TERMINAL_ITERATION, motion, mMotionGen, 
								  mBlend, mFixRootRot, mFixRootPos);
}
void 
ReferenceManager::
saveTrajectory(std::pair<std::vector<Eigen::VectorXd>, std::vector<double>> _trajectory, Fitness _f, Eigen::VectorXd _parameter)
{
	double start = (_trajectory.second)[0];
	
	_trajectory.first = align(_trajectory.first, getBVHFrame(start).position.segment<6>(0));
	mLock.lock();

	std::vector<Eigen::VectorXd> trajectory = normalizeTrajectory(_trajectory);
	std::vector<Eigen::VectorXd> displacement = getDisplacementFromTrajectory(trajectory);

	mEliteSet->updateEliteSet(displacement, _f, _parameter);

	mLock.unlock();

}
std::vector<Eigen::VectorXd>
ReferenceManager::
normalizeTrajectory(std::pair<std::vector<Eigen::VectorXd>, std::vector<double>> _trajectory)
{
	std::vector<Eigen::VectorXd> pos = _trajectory.first;
	std::vector<double> phase = _trajectory.second;

	std::vector<Eigen::VectorXd> trajectory;
	int dof = _trajectory.first[0].rows();

	int count = 0;
	for(int i = 0; i < mMotionLength; i++) {
		while(count + 1 < phase.size() && i >= phase[count+1])
			count += 1;
		
		Eigen::VectorXd p(dof + 1);

		if(i == 0) { // phase start
			int size = phase.size();

			double weight = (mMotionLength - phase[size-1]) / (mMotionLength - phase[size-1] + phase[0]);
			Eigen::VectorXd p_blend = weightedSumPos(pos[size-1], pos[0], weight);
			p_blend.segment<6>(0) = pos[0].segment<6>(0);

			double t0 = phase[size-1] - phase[size-2];
			double t1 = phase[1] - phase[0];
			double t_blend = (1 - weight) * t0 + weight * t1;
			p << p_blend, t_blend;

		} else if(count == phase.size() - 1) { // phase end
			double weight = (i - phase[count]) / (phase[0] + mMotionLength - phase[count]);
			Eigen::VectorXd p_blend = weightedSumPos(pos[0], pos[count], weight);
			p_blend.segment<6>(0) = pos[count].segment<6>(0);

			double t0 = phase[count] - phase[count-1];
			double t1 = phase[1] - phase[0];
			double t_blend = (1 - weight) * t0 + weight * t1;
			
			p << p_blend, t_blend;
		} else { //general case
			double weight = (i - phase[count]) / (phase[count+1] - phase[count]);
			Eigen::VectorXd p_blend = weightedSumPos(pos[count], pos[count+1], weight);
			double t0 = phase[count + 1] - phase[count];
			double t1;
			if(count == phase.size() - 2) {
				t1 = phase[count+1] - phase[count];
			} else {
				t1 = phase[count+2] - phase[count+1];
			}
			double t_blend = (1 - weight) * t0 + weight * t1;

			p << p_blend, t_blend;
		}
		trajectory.push_back(p);
	}	
	return trajectory;
}
std::vector<Eigen::VectorXd> 
ReferenceManager::
getDisplacementFromTrajectory(std::vector<Eigen::VectorXd> _trajectory)
{
	std::vector<Eigen::VectorXd> displacement;
	int dof = _trajectory[0].rows();

	for(int i = 0; i < _trajectory.size(); i++) {
		Eigen::VectorXd d(dof);
		
		Eigen::VectorXd p = _trajectory[i].head(dof);
		Eigen::VectorXd p_bvh = mMotionBVH[i].position;
		for(int j = 0; j < d.rows() - 1; j += 3) {			
			if(j == 3) {
				d.segment<3>(j) = p.segment<3>(j) - p_bvh.segment<3>(j);
			} else {
				d.segment<3>(j) = jointPositionDifferences(p.segment<3>(j), p_bvh.segment<3>(j));
			}
		}
		d(dof-1) = log(_trajectory[i](dof-1));
		displacement.push_back(d);
	}
	return displacement;
}
std::vector<Eigen::VectorXd> 
ReferenceManager::
getTrajectoryFromDisplacement(std::vector<Eigen::VectorXd> _displacement)
{
	std::vector<Eigen::VectorXd> trajectory;
	int dof = _displacement[0].rows();

	for(int i = 0; i < _displacement.size(); i++) {
		Eigen::VectorXd p(dof);

		Eigen::VectorXd p_bvh = mMotionBVH[i].position;
		Eigen::VectorXd d = _displacement[i];

		for(int j = 0; j < d.rows() - 1; j += 3) {
			if(j == 3) {
				p.segment<3>(j) = d.segment<3>(j) + p_bvh.segment<3>(j);
			} else {
				p.segment<3>(j) = rotate(p_bvh.segment<3>(j), d.segment<3>(j));
			} 
		}
		p(dof-1) = exp(d(dof-1));
		trajectory.push_back(p);
	}
	return trajectory;
}
};
