#include <tinyxml2.h>
#include <fstream>
#include <cmath>
#include "Functions.h"
#include "SkeletonBuilder.h"
#include "BVHParser.h"
#include "Configurations.h"

typedef tinyxml2::XMLElement TiXmlElement;
typedef tinyxml2::XMLDocument TiXmlDocument;

namespace SIM
{
XMLNode::
XMLNode(std::string _name, Eigen::Vector3d _offset)
:mJTfromParentJoint(_offset), mName(_name)
{}
void 
XMLNode::
setParent(XMLNode* _parent)
{
	mParent = _parent;
}
void 
XMLNode::
setBodyTranslation(Eigen::Vector3d _bt)
{
	mBTfromJoint = _bt;
}
void 
XMLNode::
setSize(Eigen::Vector3d _size)
{
	mSize = _size;	
}
XMLNode* 
XMLNode::
getParent()
{
	return mParent;
}
std::string 
XMLNode::
getName()
{
	return mName;
}
Eigen::Vector3d 
XMLNode::
getBodyTranslation()
{
	return mBTfromJoint;
}
Eigen::Vector3d 
XMLNode::
getJointTranslation()
{
	return mJTfromParentJoint;
}
Eigen::Vector3d 
XMLNode::
getSize()
{
	return mSize;
}
std::vector<XMLNode*>
generateXMLnodesRecursive(BVHNode* _current)
{
	std::vector<XMLNode*> bodynodes;
	XMLNode* bn = new XMLNode(_current->getName(), _current->getOffset());
	bodynodes.push_back(bn);
	
	std::vector<Eigen::Vector3d> childOffsets;
	for(int i = 0; i < _current->getChildren().size(); i++) {
		BVHNode* child = _current->getChildren()[i];
		childOffsets.push_back(child->getOffset());

		if(!child->isEnd()) {
			std::vector<XMLNode*> bodynodesChild = generateXMLnodesRecursive(child);
			bodynodesChild[0]->setParent(bn);
			for(int j = 0; j < bodynodesChild.size(); j++) {
				bodynodes.push_back(bodynodesChild[j]);
			}
		}
	}

	Eigen::Vector3d flag;
	Eigen::Vector3d min;
	Eigen::Vector3d max;

	flag.setZero();
	min.setZero();
	max.setZero();

	for(int i = 0; i < childOffsets.size(); i++) {
		for(int j = 0; j < 3; j++) {
			if(childOffsets[i][j] != 0) {
				if(childOffsets[i][j] < min[j]) {
					min[j] = childOffsets[i][j];
				} else if(childOffsets[i][j] > max[j]) {
					max[j] = childOffsets[i][j];
				}
				flag[j] = 1;
			}
		}
	}

	Eigen::Vector3d size = - min + max;
	for(int j = 0; j < 3; j++) {
		if(flag(j) == 0 || size(j) < 0.04) {
			min(j) += -0.06;
			max(j) += 0.06;
			size(j) = - min(j) + max(j);
		}
	}
	Eigen::Vector3d bodyTranslation = min + size / 2.0;
	bn->setSize(size);
	bn->setBodyTranslation(bodyTranslation);

	return bodynodes;
}
void 
SkeletonBuilder::
generateNewSkeleton(std::string _motion, std::string _path) 
{
	//torque limits and joint pos limits should be written manually.
	if(boost::filesystem::exists(_path)) {
		std::cout << "file exists. generation stopped." << std::endl;
		return;
	}
	BVHNode* root = BVHParser::parseHierarchy(_motion);
	std::vector<XMLNode*> bodynodes = generateXMLnodesRecursive(root);
	
	std::ofstream ofs(_path);
	ofs << "<Skeleton name=\"Humanoid\">" << std::endl;
	for(int i = 0; i < bodynodes.size(); i++) {
		if(i == 0) {
			ofs << "<Joint type=\"FreeJoint\" name=\"" 
				<< bodynodes[i]->getName() << "\" parent_name=\"None\" size=\"" 
				<< vec2Str(bodynodes[i]->getSize()) << "\" mass=\"5\">" << std::endl;
			ofs << "<BodyPosition translation=\""
				<< vec2Str(bodynodes[i]->getBodyTranslation()) << "\" />" << std::endl;
			ofs << "<JointPosition translation=\"0 0 0\" />" << std::endl;
		} else {
			ofs << "<Joint type=\"BallJoint\" name=\"" 
				<< bodynodes[i]->getName() << "\" parent_name=\""
				<< bodynodes[i]->getParent()->getName() << "\" size=\"" 
				<< vec2Str(bodynodes[i]->getSize()) << "\" mass=\"5\">" << std::endl;
			ofs << "<BodyPosition translation=\""
				<< vec2Str(bodynodes[i]->getBodyTranslation()) << "\" />" << std::endl;
			ofs << "<JointPosition translation=\"" 
				<< vec2Str(bodynodes[i]->getJointTranslation()) << "\" />" << std::endl;
		}
		ofs << "</Joint>" << std::endl;
	}
	ofs << "</Skeleton>" << std::endl;
	ofs.close();
}
std::pair<dart::dynamics::SkeletonPtr, std::map<std::string, double>> 
SkeletonBuilder::
buildFromFile(std::string _xml) {
	TiXmlDocument doc;
	if(doc.LoadFile(_xml.c_str())){
		std::cout << "Can't open file : " << _xml << std::endl;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
	
	std::string skelname = skeldoc->Attribute("name");
	dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create(skelname);

	std::map<std::string, double> torqueMap;

	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// type
		std::string jointType = body->Attribute("type");
		std::string name = body->Attribute("name");
		std::string parentName = body->Attribute("parent_name");
		dart::dynamics::BodyNode *parent;
		if(!parentName.compare("None"))
			parent = nullptr;
		else
			parent = skel->getBodyNode(parentName);
		// size
		Eigen::Vector3d size = str2Vec3d(std::string(body->Attribute("size")));

		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		if(bodyPosElem->Attribute("linear")!=nullptr)
			bodyPosition.linear() = orthonormalize(str2Mat3d(bodyPosElem->Attribute("linear")));
		bodyPosition.translation() = str2Vec3d(bodyPosElem->Attribute("translation"));

		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = orthonormalize(str2Mat3d(jointPosElem->Attribute("linear")));
		jointPosition.translation() = str2Vec3d(jointPosElem->Attribute("translation"));

		double torquelim = 1e6;
		TiXmlElement *torquelimElem = body->FirstChildElement("TorqueLimit");
		if(torquelimElem != nullptr) {
			torquelim = std::stod(torquelimElem->Attribute("norm"));
		}
		torqueMap.insert(std::pair<std::string, double>(name, torquelim));

		// mass
		double mass = atof(body->Attribute("mass"));
		
		if(!jointType.compare("FreeJoint") ){
			SkeletonBuilder::makeFreeJointBody(skel, parent,
											   name, size, mass,
											   jointPosition, bodyPosition);
		} else if(!jointType.compare("BallJoint")){
			// joint limit
			bool isLimitEnforced = false;
			Eigen::Vector3d upperLimit(1E6,1E6,1E6), lowerLimit(-1E6,-1E6,-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				isLimitEnforced = true;
				upperLimit = str2Vec3d(jointPosElem->Attribute("upper"));
			}
			if(jointPosElem->Attribute("lower")!=nullptr)
			{
				isLimitEnforced = true;
				lowerLimit = str2Vec3d(jointPosElem->Attribute("lower"));
			}

			SkeletonBuilder::makeBallJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition,
											   isLimitEnforced, upperLimit, lowerLimit);
		} else if(!jointType.compare("WeldJoint")){
			SkeletonBuilder::makeWeldJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition);			
		}
	}
	return std::pair<dart::dynamics::SkeletonPtr, std::map<std::string, double>> (skel, torqueMap);
}
dart::dynamics::BodyNode* 
SkeletonBuilder::
makeFreeJointBody(
	const dart::dynamics::SkeletonPtr& _skel,
	dart::dynamics::BodyNode* const _parent,
	std::string _name,
	Eigen::Vector3d _size,
	double _mass,
	Eigen::Isometry3d _jointPosition,
	Eigen::Isometry3d _bodyPosition)
{
	dart::dynamics::ShapePtr shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(_size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(_mass);
	inertia.setMoment(shape->computeInertia(_mass));

	dart::dynamics::BodyNode* bn;
	dart::dynamics::FreeJoint::Properties props;
	props.mName = _name;
	if(_parent!=nullptr)
		props.mT_ParentBodyToJoint = _parent->getParentJoint()->getTransformFromChildBodyNode()*_jointPosition;
	props.mT_ChildBodyToJoint = _bodyPosition.inverse();

	bn = _skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
		_parent, props, dart::dynamics::BodyNode::AspectProperties(_name)).second;

	bn->createShapeNodeWith<dart::dynamics::CollisionAspect,dart::dynamics::DynamicsAspect>(shape);
	bn->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);	
	bn->setInertia(inertia);

	return bn;
}

dart::dynamics::BodyNode* 
SkeletonBuilder::
makeBallJointBody(
	const dart::dynamics::SkeletonPtr& _skel,
	dart::dynamics::BodyNode* const _parent,
	std::string _name,
	Eigen::Vector3d _size,
	double _mass,
	Eigen::Isometry3d _jointPosition,
	Eigen::Isometry3d _bodyPosition,
	bool _isLimitEnforced,
	Eigen::Vector3d _upperLimit,
	Eigen::Vector3d _lowerLimit)
{
	dart::dynamics::ShapePtr shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(_size));
	
	dart::dynamics::Inertia inertia;
	inertia.setMass(_mass);
	inertia.setMoment(shape->computeInertia(_mass));

	dart::dynamics::BodyNode* bn;
	dart::dynamics::BallJoint::Properties props;
	props.mName = _name;
	if(_parent!=nullptr)
		props.mT_ParentBodyToJoint = _parent->getParentJoint()->getTransformFromChildBodyNode()*_jointPosition;
	props.mT_ChildBodyToJoint = _bodyPosition.inverse();

	bn = _skel->createJointAndBodyNodePair<dart::dynamics::BallJoint>(
		_parent, props, dart::dynamics::BodyNode::AspectProperties(_name)).second;

	dart::dynamics::JointPtr jn = bn->getParentJoint();
	for(int i = 0; i < jn->getNumDofs(); i++){
		jn->getDof(i)->setDampingCoefficient(JOINT_DAMPING);
	}

	bn->createShapeNodeWith<dart::dynamics::CollisionAspect,dart::dynamics::DynamicsAspect>(shape);
	bn->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
	bn->setInertia(inertia);

	if(_isLimitEnforced){
		dart::dynamics::JointPtr joint = bn->getParentJoint();
		joint->setLimitEnforcement(_isLimitEnforced);

		for(int i = 0; i < 3; i++)
		{
			joint->setPositionUpperLimit(i, _upperLimit[i]);
			joint->setPositionLowerLimit(i, _lowerLimit[i]);
		}
	}

	return bn;
}
dart::dynamics::BodyNode* 
SkeletonBuilder::
makeWeldJointBody(
	const dart::dynamics::SkeletonPtr& _skel,
	dart::dynamics::BodyNode* const _parent,
	std::string _name,
	Eigen::Vector3d _size,
	double _mass,
	Eigen::Isometry3d& _jointPosition,
	Eigen::Isometry3d& _bodyPosition)
{
	dart::dynamics::ShapePtr shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(_size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(_mass);
	inertia.setMoment(shape->computeInertia(_mass));

	dart::dynamics::BodyNode* bn;
	dart::dynamics::FreeJoint::Properties props;
	props.mName = _name;
	
	if(_parent!=nullptr)
		props.mT_ParentBodyToJoint = _parent->getParentJoint()->getTransformFromChildBodyNode()*_jointPosition;
	props.mT_ChildBodyToJoint = _bodyPosition.inverse();

	bn = _skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
		_parent, props, dart::dynamics::BodyNode::AspectProperties(_name)).second;
	
	bn->createShapeNodeWith<dart::dynamics::VisualAspect,dart::dynamics::CollisionAspect,dart::dynamics::DynamicsAspect>(shape);
	bn->setInertia(inertia);

	return bn;
}
};