#ifndef __RENDER_DART_INTERFACE_H__
#define __RENDER_DART_INTERFACE_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "GLFunctions.h"
namespace GUI
{
	void drawSkeleton(const dart::dynamics::SkeletonPtr& _skel, int _type=0);
	void drawFootContact(const dart::dynamics::SkeletonPtr& _skel, std::pair<bool, bool> _contact);
	void drawShape(Eigen::Isometry3d _T,
		const dart::dynamics::Shape* _shape,
		Eigen::Vector4d _color);
	void drawBodyNode(const dart::dynamics::SkeletonPtr& skel, 
		Eigen::Vector4d _color,
		std::string _name, 
		int type=0);
	void setSkeletonColor(const dart::dynamics::SkeletonPtr& _object, Eigen::Vector3d _color);
	void setSkeletonColor(const dart::dynamics::SkeletonPtr& _object, Eigen::Vector4d _color);	

};

#endif