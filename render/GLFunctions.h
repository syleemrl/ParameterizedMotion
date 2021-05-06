#ifndef __RENDER_GL_FUNCTIONS_H__
#define __RENDER_GL_FUNCTIONS_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <assimp/cimport.h>
#include <assimp/scene.h>
namespace GUI
{
	void drawSphere(double _r);
	void drawCapsule(double _radius, double _height);
	void drawCylinder(double _radius, double _height);
	void drawCube(Eigen::Vector3d _size);
	void drawTriangle(Eigen::Vector3d _p0, Eigen::Vector3d _p1, Eigen::Vector3d _p2, Eigen::Vector3d _color = Eigen::Vector3d(0.8,0.8,0.8));
	void drawLine(Eigen::Vector3d _p0,Eigen::Vector3d _p1, Eigen::Vector3d _color = Eigen::Vector3d(0.8,0.8,0.8));
	void drawPoint(Eigen::Vector3d _p0,Eigen::Vector3d _color = Eigen::Vector3d(0.8,0.8,0.8), double _scale = 1.0);

	void drawRoundedBoxPlanes(Eigen::Vector3d _size, double _radius);
    void drawRoundedBoxCylinder(Eigen::Vector2d _size, double _height, double _radius);
    void drawRoundedBoxSphere(Eigen::Vector3d _size, double _radius);
	void drawRoundedBox(Eigen::Vector3d _size, double _radius);

	void drawBezierCurve(
		Eigen::Vector3d _p0,
		Eigen::Vector3d _p1,
		Eigen::Vector3d _p2,
		Eigen::Vector3d _color = Eigen::Vector3d(0.8,0.8,0.8));
	void drawTrajectory(std::vector<Eigen::Vector3d> _points, int _idx, Eigen::Vector3d _color = Eigen::Vector3d(1.0,0.0,0.0), bool _line=true);
	void drawStringOnScreen(float _x, float _y, std::string _s, bool _bigFont, Eigen::Vector3d _color=Eigen::Vector3d(0.8,0.8,0.8));
	void drawGround(int _comX, int _comZ, double _groundHeight);
};

#endif