#include <iostream>
#include <fstream>
#include "BVHParser.h"
#include "Functions.h"
namespace SIM
{
BVHNode::
BVHNode(BVHNode* _parent, std::string _name) 
: mParent(_parent), mName(_name) 
{
	if(mParent)
		mIsRoot = false;
	else
		mIsRoot = true;
}
BVHNode::
~BVHNode() 
{
	while(!mChildren.empty()) {
		BVHNode* c = mChildren.back();
		mChildren.pop_back();

		delete c;
	}
}
void 
BVHNode::
addChild(BVHNode* _child) 
{
	mChildren.push_back(_child);
}
void 
BVHNode::
setIsEnd(bool _isEnd) 
{
	mIsEndEffector = _isEnd;
}
void 
BVHNode::
setOffset(double _x, double _y, double _z) 
{
	mOffset << _x, _y, _z;
}
void 
BVHNode::
setChannel(std::vector<std::string> _ch) 
{
	mChannels = _ch;
}
std::vector<std::string> 
BVHNode::
getChannels() 
{
	return mChannels;
}
std::vector<BVHNode*> 
BVHNode::
getChildren()
{
	return mChildren;
}
Eigen::Vector3d 
BVHNode::
getOffset()
{
	return mOffset;
}
std::string 
BVHNode::
getName()
{
	return mName;
}
bool 
BVHNode::
isRoot()
{
	return mIsRoot;
}
bool 
BVHNode::
isEnd()
{
	return mIsEndEffector;
}
void
parseHierarchyRecursive(BVHNode* _current, std::ifstream& _is)
{
	char buffer[256];

	_is >> buffer; //{
	while(_is >> buffer)
	{
		std::string str = buffer;

		if(!strcmp(buffer,"}")) {
			break;
		}
		if(!strcmp(buffer,"OFFSET"))
		{
			double x,y,z;

			_is >> x;
			_is >> y;
			_is >> z;
			_current->setOffset(x*0.01, y*0.01, z*0.01);
		} else if(!strcmp(buffer,"CHANNELS"))
		{
			_is >> buffer;
			int n;
			n = atoi(buffer);
			std::vector<std::string> channels;
			if(_current->isRoot() && n != 6) {
				std::cout << "Supports 6 dof only for root joints" << std::endl;
			} else if(!_current->isRoot() && n != 3) {
				std::cout << "Supports 3 dof only for non-root joints" << std::endl;
			}
			for(int i = 0; i < n; i++)
			{
				_is >> buffer;
				channels.push_back(std::string(buffer));
			}
			_current->setChannel(channels);

		} else if(!strcmp(buffer,"JOINT"))
		{
			_is >> buffer;
			BVHNode* child = new BVHNode(_current, std::string(buffer));
			child->setIsEnd(false);
			_current->addChild(child);

			parseHierarchyRecursive(child, _is);
		} else if(!strcmp(buffer,"End"))
		{
			_is >> buffer;
			BVHNode* child = new BVHNode(_current, _current->getName() + std::string("End"));
			child->setIsEnd(true);
			_current->addChild(child);

			parseHierarchyRecursive(child, _is);
		} 
	}
}
BVHNode* 
BVHParser::
parseHierarchy(std::string _bvh) 
{
	std::ifstream is(_bvh);
	char buffer[256];

	if(!is)
	{
		std::cout << "Can't open file: " << _bvh << std::endl;
		return 0;
	}
	BVHNode* root;
	while(is >> buffer)
	{
		if(!strcmp(buffer,"HIERARCHY"))
		{
			is>>buffer;//Root
			is>>buffer;//Name
			root = new BVHNode(0, std::string(buffer));
			root->setIsEnd(false);

			parseHierarchyRecursive(root, is);
			break;
		}
	}
	is.close();

	return root;
}
Eigen::VectorXd
parseFrameRecursive(BVHNode* _current, std::ifstream& _is)
{
	std::vector<std::string> ch = _current->getChannels();
	Eigen::VectorXd p(3);
	char buffer[256];

	if(_current->isRoot()) {
		p.resize(6);
		Eigen::VectorXd dof(6);
		for(int i = 0; i < 6; i++) {
			_is >> buffer;
			dof(i) = atof(buffer);
		}
		
		Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
		for(int i = 0; i < 6; i++) {
			if(_current->getChannels()[i] == "Xrotation")
			{
			   rot *= eulerX2Rot(dof(i));
			} else if(_current->getChannels()[i] == "Yrotation")
			{
			   rot *= eulerY2Rot(dof(i));
			} else if(_current->getChannels()[i] == "Zrotation")
			{
			   rot *= eulerZ2Rot(dof(i));
			} else if(_current->getChannels()[i] == "Xposition")
			{
			   p(0) = dof(i)*0.01;
			} else if(_current->getChannels()[i] == "Yposition")
			{
			   p(1) = dof(i)*0.01;
			} else if(_current->getChannels()[i] == "Zposition")
			{
			   p(2) = dof(i)*0.01;
			}
		}
		p.segment<3>(3) = logMap(rot);
	} else {
		Eigen::Vector3d dof;
		for(int i = 0; i < 3; i++) {
			_is >> buffer;
			dof(i) = atof(buffer);
		}

		Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
		for(int i = 0; i < 3; i++) {
			if(_current->getChannels()[i] == "Xrotation")
			{
			   rot *= eulerX2Rot(dof(i));
			} else if(_current->getChannels()[i] == "Yrotation")
			{
			   rot *= eulerY2Rot(dof(i));
			} else if(_current->getChannels()[i] == "Zrotation")
			{
			   rot *= eulerZ2Rot(dof(i));
			}
		}
		p = logMap(rot);
	}

	for(int i = 0; i < _current->getChildren().size(); i++)
	{
		BVHNode* child = _current->getChildren()[i];
		if(!child->isEnd()) {
			Eigen::VectorXd pChild = parseFrameRecursive(_current->getChildren()[i], _is);	
			Eigen::VectorXd p_(p.rows() + pChild.rows());
			p_.head(p.rows()) = p;
			p_.tail(pChild.rows()) = pChild;
			p = p_;

		}
	}

	return p;
}
std::pair<double,std::vector<Eigen::VectorXd>>
BVHParser::
parseMotion(std::string _bvh, BVHNode* _root)
{
	std::ifstream is(_bvh);
	char buffer[256];
	double timestep;
	std::vector<Eigen::VectorXd> positions;
	while(is >> buffer)
	{
		if(!strcmp(buffer,"MOTION"))
		{
			is >> buffer; //Frames:
			is >> buffer; //num_frames
			int nFrames = atoi(buffer);
			is >> buffer; //Frame
			is >> buffer; //Time:
			is >> buffer; //time step
			timestep = atof(buffer);
			for(int i = 0; i < nFrames; i++)
			{
				Eigen::VectorXd p = parseFrameRecursive(_root, is);
				positions.push_back(p);
			}
		}
	}
	is.close();
	return std::pair<double,std::vector<Eigen::VectorXd>>(timestep, positions);
}
std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>> 
BVHParser::
parse(std::string _bvh){
	BVHNode* root = parseHierarchy(_bvh);
	std::pair<double, std::vector<Eigen::VectorXd>> tp = parseMotion(_bvh, root);
	return std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>>(root, tp.first, tp.second);
}
std::vector<std::pair<std::string, int>> 
BVHParser::
getNodeIdx(BVHNode* _current) {
	std::vector<std::pair<std::string, int>> idxList;
	idxList.push_back(std::pair<std::string, int>(_current->getName(), 0));
	int idxMax = _current->getChannels().size();

	for(int i = 0 ; i < _current->getChildren().size(); i++) {
		BVHNode* child = _current->getChildren()[i];
		if(!child->isEnd()) {
			std::vector<std::pair<std::string, int>> idxListChild = getNodeIdx(child);
			for(int j = 0; j < idxListChild.size(); j++) {
				idxListChild[j].second += idxMax;
				idxList.push_back(idxListChild[j]);
			}
			idxMax = idxList.back().second + 3;
		}
	}
	return idxList;
}
};