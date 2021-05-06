#ifndef __SIM_BVH_PARSER_H__
#define __SIM_BVH_PARSER_H__

#include <Eigen/Core>
#include <vector>
#include <tuple>

namespace SIM
{
class BVHNode
{
public:
	BVHNode(BVHNode* _parent, std::string _name);
	~BVHNode();
	void addChild(BVHNode* _child);
	void setIsEnd(bool _isEnd);
	void setOffset(double _x, double _y, double _z);
	void setChannel(std::vector<std::string> _ch);
	std::vector<BVHNode*> getChildren();
	std::vector<std::string> getChannels();
	Eigen::Vector3d getOffset();
	std::string getName();
	bool isRoot();
	bool isEnd();

private:
	BVHNode* mParent;
	std::vector<BVHNode*> mChildren;
	std::vector<std::string> mChannels;
	Eigen::Vector3d mOffset;
	std::string mName;

	bool mIsRoot;
	bool mIsEndEffector;
};
class BVHParser
{
public:
	static BVHNode* parseHierarchy(std::string _bvh);
	static std::pair<double, std::vector<Eigen::VectorXd>> parseMotion(std::string _bvh, BVHNode* _root);
	static std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>> parse(std::string _bvh);
	static std::vector<std::pair<std::string, int>> getNodeIdx(BVHNode* _current);
};
}

#endif