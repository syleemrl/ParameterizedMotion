#ifndef __RENDER_CAMERA_H__
#define __RENDER_CAMERA_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
class Camera
{
public:
	Camera(int _w, int _h);
		
	void setSize(int _w, int _h);
	void setCamera(Eigen::Vector3d _lookAt, Eigen::Vector3d _eye, Eigen::Vector3d _up);
	void apply();

	void pan(int _x, int _y, int _prevX, int _prevY);
	void zoom(int _x, int _y, int _prevX, int _prevY);
	void rotate(int _x, int _y, int _prevX, int _prevY);
	void translate(int _x, int _y, int _prevX, int _prevY);
	void setCenter(Eigen::Vector3d _c);
private:
	Eigen::Vector3d mLookAt;
	Eigen::Vector3d mEye;
	Eigen::Vector3d mUp;
	double mFovy;
	int mW, mH;
};


#endif