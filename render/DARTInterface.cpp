#include "DARTInterface.h"
using namespace dart::dynamics;
using namespace dart::simulation;

void
GUI::
drawSkeleton(const dart::dynamics::SkeletonPtr& _skel, int _type)
{
	for(int i = 0; i < _skel->getNumBodyNodes(); i++)
	{
		auto bn = _skel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
		auto jn = bn->getParentJoint();
		Eigen::Isometry3d jn_transform = bn->getTransform()*jn->getTransformFromChildBodyNode();

		Eigen::Vector3d jn_com = jn_transform.translation();

		std::string name = bn->getName();
		Eigen::Vector4d color = shapeNodes[_type]->getVisualAspect()->getRGBA();
		color.head<3>() *= 0.5;
		if(name == "LeftUpLeg" || name == "RightUpLeg" || name == "LeftLeg" || name == "RightLeg"){
			glPushMatrix();
			glColor4f(color[0], color[1], color[2], color[3]);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glTranslatef(jn_com[0], jn_com[1], jn_com[2]);
			GUI::drawSphere(0.045);
			glPopMatrix();
		}

		if(name == "LeftForeArm" || name == "RightForeArm"){
			glPushMatrix();
			glColor4f(color[0], color[1], color[2], color[3]);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glTranslatef(jn_com[0], jn_com[1], jn_com[2]);
			GUI::drawSphere(0.035);
			glPopMatrix();
		}

		if(name == "LeftHand" || name == "RightHand"){
			glPushMatrix();
			glColor4f(color[0], color[1], color[2], color[3]);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glTranslatef(jn_com[0], jn_com[1], jn_com[2]);
			GUI::drawSphere(0.03);
			glPopMatrix();
		}

		if(name == "LeftFoot" || name == "RightFoot"){
			glPushMatrix();
			glColor4f(color[0], color[1], color[2], color[3]);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glTranslatef(jn_com[0], jn_com[1], jn_com[2]);
			GUI::drawSphere(0.03);
			glPopMatrix();

		}
		
		if(name == "Ground_dummy")
			continue;

		auto T = shapeNodes[_type]->getTransform();
		drawShape(T, shapeNodes[_type]->getShape().get(), shapeNodes[_type]->getVisualAspect()->getRGBA());
	}
}
void 
GUI::
drawFootContact(
	const dart::dynamics::SkeletonPtr& _skel, std::pair<bool, bool> _contact)
{
	if(_contact.first) {
		drawBodyNode(_skel, Eigen::Vector4d(0.2, 0.2, 0.2, 1), "RightFoot");
		drawBodyNode(_skel, Eigen::Vector4d(0.2, 0.2, 0.2, 1), "RightToe");
	}
	if(_contact.second) {
		drawBodyNode(_skel, Eigen::Vector4d(0.2, 0.2, 0.2, 1), "LeftFoot");
		drawBodyNode(_skel, Eigen::Vector4d(0.2, 0.2, 0.2, 1), "LeftToe");
	}
}
void
GUI::
drawShape(Eigen::Isometry3d _T,
	const dart::dynamics::Shape* _shape,
	Eigen::Vector4d _color)
{
	glEnable(GL_LIGHTING);

    float ground_mat_shininess[] = {128.0};
    float ground_mat_specular[]  = {0.01, 0.01, 0.01, 0.35};
    float ground_mat_diffuse[]   = {0.05, 0.05, 0.05, 0.35};
    float ground_mat_ambient[]  = {0.05, 0.05, 0.05, 0.35};
    for(int i = 0; i < 4; i++){
    	ground_mat_specular[i] = _color[i]*0.1;
    	ground_mat_diffuse[i] = _color[i];
    	ground_mat_ambient[i] = _color[i];
    }

  
	glColor4f(_color[0], _color[1], _color[2], _color[3]);
	glPushMatrix();

	Eigen::Vector3d translation = _T.translation();
	Eigen::Matrix3d linear = _T.linear();
	Eigen::AngleAxisd aa(linear);
	glTranslatef(translation[0], translation[1], translation[2]);

	if(_shape->is<SphereShape>())
	{
		const auto* sphere = dynamic_cast<const SphereShape*>(_shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		GUI::drawSphere(sphere->getRadius());
	}
	glRotatef(aa.angle()/M_PI*180.0, aa.axis()[0], aa.axis()[1], aa.axis()[2]);

	if(_shape->is<BoxShape>())
	{
		const auto* box = dynamic_cast<const BoxShape*>(_shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    	GUI::drawRoundedBox(box->getSize(), 0.02);
	}
	else if(_shape->is<CapsuleShape>())
	{
		auto* capsule = dynamic_cast<const CapsuleShape*>(_shape);
    	GUI::drawCapsule(capsule->getRadius(),capsule->getHeight());

	}
	else if(_shape->is<CylinderShape>())
	{
		auto* cylinder = dynamic_cast<const CylinderShape*>(_shape);
    	GUI::drawCylinder(cylinder->getRadius(),cylinder->getHeight());
	}

	glPopMatrix();
}
void
GUI::
drawBodyNode(const dart::dynamics::SkeletonPtr& _skel, Eigen::Vector4d _color, std::string _name, int _type)
{
	auto bn = _skel->getBodyNode(_name);
	auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
	auto T = shapeNodes[_type]->getTransform();
	drawShape(T, shapeNodes[_type]->getShape().get(), _color);
}
void 
GUI::
setSkeletonColor(const dart::dynamics::SkeletonPtr& _object, Eigen::Vector3d _color)
{
	// Set the color of all the shapes in the object
	for(std::size_t i=0; i < _object->getNumBodyNodes(); ++i)
	{
		Eigen::Vector3d c = _color;
		dart::dynamics::BodyNode* bn = _object->getBodyNode(i);
		if(bn->getName() == "Neck")
			c.head<3>() *= 0.5;
		auto visualShapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
		for(auto visualShapeNode : visualShapeNodes)
			visualShapeNode->getVisualAspect()->setColor(c);
	}
}

void
GUI::
setSkeletonColor(const dart::dynamics::SkeletonPtr& _object, Eigen::Vector4d _color)
{
	// Set the color of all the shapes in the object
	for(std::size_t i=0; i < _object->getNumBodyNodes(); ++i)
	{
		Eigen::Vector4d c = _color;
		dart::dynamics::BodyNode* bn = _object->getBodyNode(i);
		if(bn->getName() == "Neck")
			c.head<3>() *= 0.5;
		auto visualShapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
		for(auto visualShapeNode : visualShapeNodes)
			visualShapeNode->getVisualAspect()->setRGBA(c);
	}
}
