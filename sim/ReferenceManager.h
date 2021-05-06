#ifndef __SIM_REFERENCE_MANAGER_H__
#define __SIM_REFERENCE_MANAGER_H__

#include <tuple>
#include <mutex>
#include "dart/dart.hpp"
#include "EliteSet.h"
namespace SIM
{
struct Frame
{
	Frame(Eigen::VectorXd _p, Eigen::VectorXd _v): position(_p), velocity(_v), phaseDelta(1) {}
	Frame(Eigen::VectorXd _p, Eigen::VectorXd _v, double _t): position(_p), velocity(_v), phaseDelta(_t) {}
	Frame(const SIM::Frame& _mn): position(_mn.position), velocity(_mn.velocity), phaseDelta(_mn.phaseDelta) {}
	Eigen::VectorXd position;
	Eigen::VectorXd velocity;
	double phaseDelta;
};
class ReferenceManager
{
public:
	ReferenceManager(dart::dynamics::SkeletonPtr _skel);

	Eigen::VectorXd computeVelocity(bool _smooth, Eigen::VectorXd _p2, Eigen::VectorXd _p1, Eigen::VectorXd _p0=Eigen::VectorXd::Zero(1));
	void loadMotionFromBVH(std::string _bvh);
	void generateCycles(int _numFrames, std::vector<Frame>& _clip, std::vector<Frame>& _gen,
									   bool _blend=false, bool _fixRootRot=false, bool _fixRootPos=false);
	Frame getFrame(double _t);
	Frame getBVHFrame(double _t);
	int getMotionLength() { return mMotionLength; }
	double getNumDof() {return mMotionBVH[0].position.rows() + 1; }

	// parametric
	void setEliteSet(EliteSet* _eliteset) { mEliteSet = _eliteset;}
	void loadMotionFromDisplacement(std::vector<Eigen::VectorXd> _displacement);
	void saveTrajectory(std::pair<std::vector<Eigen::VectorXd>, std::vector<double>> _trajectory, Fitness _f, Eigen::VectorXd _parameter);

	std::vector<Eigen::VectorXd> normalizeTrajectory(std::pair<std::vector<Eigen::VectorXd>, std::vector<double>> _trajectory);
	std::vector<Eigen::VectorXd> getDisplacementFromTrajectory(std::vector<Eigen::VectorXd> _trajectory);
	std::vector<Eigen::VectorXd> getTrajectoryFromDisplacement(std::vector<Eigen::VectorXd> _displacement);

protected:
	dart::dynamics::SkeletonPtr mSkel;
	int mMotionLength;

	int mBlendingInterval;
	bool mBlend;
	bool mFixRootRot;
	bool mFixRootPos;

	std::vector<Frame> mMotionBVH;
	std::vector<Frame> mMotionGen;

	EliteSet* mEliteSet;
	std::mutex mLock;

};
}

#endif