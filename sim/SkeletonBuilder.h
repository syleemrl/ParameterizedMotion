#ifndef __SIM_SKELETON_BUILDER_H__
#define __SIM_SKELETON_BUILDER_H__
#include "dart/dart.hpp"

namespace SIM
{
class XMLNode
{
public:
	XMLNode(std::string _name, Eigen::Vector3d _offset);
	void setParent(XMLNode* _parent);
	void setBodyTranslation(Eigen::Vector3d _bt);
	void setSize(Eigen::Vector3d _size);
	XMLNode* getParent();
	std::string getName();
	Eigen::Vector3d getBodyTranslation();
	Eigen::Vector3d getJointTranslation();
	Eigen::Vector3d getSize();
private:
	XMLNode* mParent;
	std::string mName;
	Eigen::Vector3d mJTfromParentJoint;
	Eigen::Vector3d mBTfromJoint;
	Eigen::Vector3d mSize;
};
class SkeletonBuilder
{
public:
	static void generateNewSkeleton(std::string _motion, std::string _path);
	static std::pair<dart::dynamics::SkeletonPtr, std::map<std::string, double>> buildFromFile(std::string _xml);
    static dart::dynamics::BodyNode* makeFreeJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d _jointPosition,
		Eigen::Isometry3d _bodyPosition);

	static dart::dynamics::BodyNode* makeBallJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d _jointPosition,
		Eigen::Isometry3d _bodyPosition,
		bool _isLimitEnforced,
		Eigen::Vector3d _upperLimit,
		Eigen::Vector3d _lowerLimit);
	static dart::dynamics::BodyNode* makeWeldJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d& _jointPosition,
		Eigen::Isometry3d& _bodyPosition);
};
}

#endif