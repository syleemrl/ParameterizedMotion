#include <iostream>
#include <GL/glut.h>
#include "Camera.h"
Camera::
Camera(int _w, int _h)
	:mLookAt(Eigen::Vector3d(0,0.8,0)), mEye(Eigen::Vector3d(0,1.5, 3)), mUp(Eigen::Vector3d(0,1,0)), 
	mFovy(60.0), mW(_w), mH(_h)
{

}
void
Camera::
setSize(int _w,int _h)
{
	mW = _w;
	mH = _h;
}
void
Camera::
setCamera(Eigen::Vector3d _lookAt, Eigen::Vector3d _eye, Eigen::Vector3d _up)
{
	mLookAt = _lookAt;
	mEye = _eye;
	mUp = _up;
}
void
Camera::
apply()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(mFovy, (GLfloat)mW / (GLfloat)mH, 0.01, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(mEye.x(), mEye.y(), mEye.z(),
		mLookAt.x(), mLookAt.y(), mLookAt.z(),
		mUp.x(), mUp.y(), mUp.z());
}
void
Camera::
pan(int _x, int _y, int _prevX, int _prevY)
{
	double delta = ((double)_prevY - (double)_y)/15.0;
	Eigen::Vector3d vec = (mLookAt - mEye);
	double scale = vec.norm();
	scale = std::max((scale - delta), 1.0);
	Eigen::Vector3d vd = (scale - delta) * vec.normalized();	
	mEye = mLookAt - vd;
}
void
Camera::
zoom(int _x, int _y, int _prevX, int _prevY)
{
	double delta = (double)_prevY - (double)_y;
	mFovy += delta / 20.0;
}
void
Camera::
rotate(int _x, int _y, int _prevX, int _prevY)
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	double rad = std::min(mW, mH) / 2.0;
	double dx = (double)_x - (double)_prevX;
	double dy = (double)_y - (double)_prevY;
	double angleY = atan2(dx * 0.5, rad);
	double angleX = atan2(dy * 0.5, rad);

	Eigen::Vector3d n = mLookAt - mEye;
	Eigen::Vector3d axisX = Eigen::Vector3d::UnitY().cross(n.normalized());
	n = Eigen::Quaterniond(Eigen::AngleAxisd(-angleY, Eigen::Vector3d::UnitY()))._transformVector(n);
	n = Eigen::Quaterniond(Eigen::AngleAxisd(angleX, axisX))._transformVector(n);
	mEye = mLookAt - n;
}
void 
Camera::
setCenter(Eigen::Vector3d _c){
	Eigen::Vector3d delta = _c - mLookAt;

	mLookAt += delta; 
	mEye += delta;
}
void
Camera::
translate(int _x, int _y, int _prevX, int _prevY)
{
	Eigen::Vector3d delta((double)_x - (double)_prevX, (double)_y - (double)_prevY, 0);
	Eigen::Vector3d yvec = (mLookAt - mEye);
	yvec[1] = 0;
	double scale = yvec.norm()/1000.0;
	yvec.normalize();
	Eigen::Vector3d xvec = -yvec.cross(mUp);
	xvec.normalize();

	delta = delta[0] * xvec * scale + delta[1] * yvec * scale;

	mLookAt += delta; 
	mEye += delta;
}