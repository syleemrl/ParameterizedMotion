#include <string>
#include <iostream>
#include "Functions.h"
#include "dart/dart.hpp"

namespace SIM
{
Eigen::Matrix3d
eulerX2Rot(double _x)
{
	double cosa = cos(_x*M_PI/180.0);
	double sina = sin(_x*M_PI/180.0);
	Eigen::Matrix3d R;
	R<<	1,0		,0	  ,
		0,cosa	,-sina,
		0,sina	,cosa ;
	return R;
}
Eigen::Matrix3d 
eulerY2Rot(double _y)
{
	double cosa = cos(_y*M_PI/180.0);
	double sina = sin(_y*M_PI/180.0);
	Eigen::Matrix3d R;
	R <<cosa ,0,sina,
		0    ,1,   0,
		-sina,0,cosa;
	return R;	
}
Eigen::Matrix3d 
eulerZ2Rot(double _z)
{
	double cosa = cos(_z*M_PI/180.0);
	double sina = sin(_z*M_PI/180.0);
	Eigen::Matrix3d R;
	R<<	cosa,-sina,0,
		sina,cosa ,0,
		0   ,0    ,1;
	return R;		
}
//from Dart library
Eigen::Vector3d 
logMap(Eigen::Matrix3d _mat) 
{
    Eigen::AngleAxisd aa(_mat);
    return aa.angle()*aa.axis();
}
//from Dart library
Eigen::Matrix3d 
makeSkewSymmetric(Eigen::Vector3d _v) {
  Eigen::Matrix3d result = Eigen::Matrix3d::Zero();

  result(0, 1) = -_v(2);
  result(1, 0) =  _v(2);
  result(0, 2) =  _v(1);
  result(2, 0) = -_v(1);
  result(1, 2) = -_v(0);
  result(2, 1) =  _v(0);

  return result;
}
//from Dart library
Eigen::Matrix3d 
expMapRot(Eigen::Vector3d _q) {
  double theta = _q.norm();

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d qss = makeSkewSymmetric(_q);
  Eigen::Matrix3d qss2 = qss*qss;

  if (theta < 1e-3)
    R = Eigen::Matrix3d::Identity() + qss + 0.5*qss2;
  else
    R = Eigen::Matrix3d::Identity()
        + (sin(theta)/theta)*qss
        + ((1-cos(theta))/(theta*theta))*qss2;

  return R;
}
std::string
vec2Str(Eigen::VectorXd _vec)
{
	std::string str = "";
	for(int i = 0; i < _vec.rows(); i++) {
		str += std::to_string(_vec[i]);
		if(i != _vec.rows() - 1) 
			str += " ";
	}
	return str;
}
Eigen::Vector3d 
proj(Eigen::Vector3d _u, Eigen::Vector3d _v)
{
	Eigen::Vector3d proj;
	proj = _u.dot(_v)/_u.dot(_u)*_u;
	return proj;	
}
Eigen::Matrix3d
orthonormalize(Eigen::Matrix3d _mat)
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d v0,v1,v2;
	Eigen::Vector3d u0,u1,u2;
	v0 = _mat.col(0);
	v1 = _mat.col(1);
	v2 = _mat.col(2);

	u0 = v0;
	u1 = v1 - proj(u0,v1);
	u2 = v2 - proj(u0,v2) - proj(u1,v2);

	u0.normalize();
	u1.normalize();
	u2.normalize();

	rot.col(0) = u0;
	rot.col(1) = u1;
	rot.col(2) = u2;
	return rot;
}
std::vector<double> 
str2Double(std::string _str)
{
    std::vector<double> result;
    std::string::size_type sz = 0, nsz = 0;
    while(sz < _str.length()){
        result.push_back(std::stold(_str.substr(sz), &nsz));
        sz += nsz;
    }
    return result;
}
Eigen::Matrix3d
str2Mat3d(std::string _str)
{
	std::vector<double> v = str2Double(_str);
	Eigen::Matrix3d mat;
	mat << v[0], v[1], v[2],
			v[3], v[4], v[5],
			v[6], v[7], v[8];
	return mat;
}
Eigen::Vector3d
str2Vec3d(std::string _str)
{
	std::vector<double> v = str2Double(_str);
	Eigen::Vector3d vec;
	vec << v[0], v[1], v[2];
	return vec;
}
Eigen::Vector3d 
quat2Pos(Eigen::Quaterniond _q)
{
	Eigen::AngleAxisd aa(_q);
	double angle = aa.angle();
	angle = std::fmod(angle + M_PI, 2 * M_PI)-M_PI;
	return angle*aa.axis();
}
Eigen::VectorXd 
weightedSumPos(Eigen::VectorXd _source, Eigen::VectorXd _target, double _weight, bool _blendRoot)
{
	Eigen::VectorXd pos(_target.rows());
	pos = _target;

	for(int i = 0; i < pos.size(); i += 3) {
		if (i == 3) {
			if(_blendRoot)	pos.segment<3>(i) = (1 - _weight) * _source.segment<3>(i) + _weight * _target.segment<3>(i); 
			else pos[4] = (1 - _weight) * _source[4] + _weight * _target[4]; 
		} else if (i == 0 && !_blendRoot) {
			pos(0) = _source(0) * (1-_weight) + _target(0) * _weight;
			pos(1) = _source(1);
			pos(2) = _source(2) * (1-_weight) + _target(2) * _weight;
		} else {
			Eigen::AngleAxisd tAA(_target.segment<3>(i).norm(), _target.segment<3>(i).normalized());
			Eigen::AngleAxisd sAA(_source.segment<3>(i).norm(), _source.segment<3>(i).normalized());
					
			Eigen::Quaterniond tQ(tAA);
			Eigen::Quaterniond sQ(sAA);

			pos.segment<3>(i) = quat2Pos(sQ.slerp(_weight, tQ)); 
		}
	}

	return pos;
}
Eigen::VectorXd 
weightedSumVec(Eigen::VectorXd _source, Eigen::VectorXd _target, double _weight)
{
	Eigen::VectorXd vec = (1 - _weight) * _source + _weight * _target; 
	return vec;
}
Eigen::Vector3d 
getPosDiff(Eigen::Vector3d _p1, Eigen::Vector3d _p0)
{
	Eigen::Matrix3d R0 = expMapRot(_p0);
    Eigen::Matrix3d R1 = expMapRot(_p1);

  	return logMap(R0.transpose() * R1);
}
Eigen::Vector3d 
getPosDiffXZplane(Eigen::Vector3d _p1, Eigen::Vector3d _p0)
{
	_p1 = _p1.cwiseProduct(Eigen::Vector3d::UnitY());
	_p0 = _p0.cwiseProduct(Eigen::Vector3d::UnitY());
	return getPosDiff(_p1, _p0);
}
double 
expOfSquared(Eigen::VectorXd _vec,double _sigma)
{
	return exp(-1.0*_vec.dot(_vec)/(_sigma*_sigma)/_vec.rows());
}
Eigen::Vector3d 
rotate(Eigen::Vector3d _q0, Eigen::Vector3d _q1) {
	Eigen::AngleAxisd aa0 = Eigen::AngleAxisd(_q0.norm(), _q0.normalized());
	Eigen::AngleAxisd aa1 = Eigen::AngleAxisd(_q1.norm(), _q1.normalized());
  	Eigen::AngleAxisd aa;
  	aa = aa0 * aa1;
  	return aa.axis() * aa.angle();
}
Eigen::Quaterniond pos2Quat(Eigen::Vector3d _v) {
	if( _v.norm() < 1e-8 ){
		return Eigen::Quaterniond::Identity();
	}
	Eigen::AngleAxisd aa(_v.norm(), _v.normalized());
	Eigen::Quaterniond q(aa);
	if(q.w() < 0){
		q.coeffs() *= -1;
	}
	return q;
}
Eigen::Vector3d NearestOnGeodesicCurve3d(Eigen::Vector3d targetAxis, Eigen::Vector3d targetPosition, Eigen::Vector3d position) {
	Eigen::Quaterniond v1_q = pos2Quat(position);
	Eigen::Quaterniond q = pos2Quat(targetPosition);
	Eigen::Vector3d axis = targetAxis.normalized();
	double ws = v1_q.w();
	Eigen::Vector3d vs = v1_q.vec();
	double w0 = q.w();
	Eigen::Vector3d v0 = q.vec();

	double a = ws*w0 + vs.dot(v0);
	double b = w0*(axis.dot(vs)) - ws*(axis.dot(v0)) + vs.dot(axis.cross(v0));

	double alpha = atan2( a,b );

	double t1 = -2*alpha + M_PI;
	Eigen::Quaterniond t1_q(Eigen::AngleAxisd(t1, axis));
	double t2 = -2*alpha - M_PI;
	Eigen::Quaterniond t2_q(Eigen::AngleAxisd(t2, axis));

	if (v1_q.dot(t1_q) > v1_q.dot(t2_q))
	{	
		return quat2Pos(t1_q);
	} else {
		return quat2Pos(t2_q);
	}
}
Eigen::Vector3d projectToXZ(Eigen::Vector3d v) {

	Eigen::Vector3d nearest = NearestOnGeodesicCurve3d(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0), v);
	return nearest;

}
std::vector<Eigen::VectorXd> 
align(std::vector<Eigen::VectorXd> _data, Eigen::VectorXd _target) {
	std::vector<Eigen::VectorXd> result = _data;
	Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(_data[0].head<6>());
	Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(_target.head<6>());

	Eigen::Isometry3d T01 = T1*T0.inverse();

	Eigen::Vector3d p01 = dart::math::logMap(T01.linear());			
	T01.linear() =  dart::math::expMapRot(projectToXZ(p01));
	T01.translation()[1] = 0;
	Eigen::Isometry3d T0Updated = T01*T0;

	for(int i = 0; i < _data.size(); i++) {
		Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(_data[i].head<6>());
		TCurrent = T0.inverse()*TCurrent;
		TCurrent = T0Updated*TCurrent;

		result[i].head<6>() = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
	}

	return result;
}
Eigen::Vector3d 
jointPositionDifferences(Eigen::Vector3d _q1, Eigen::Vector3d _q0)
{
	Eigen::Matrix3d R0 = expMapRot(_q0);
    Eigen::Matrix3d R1 = expMapRot(_q1);

  	return logMap(R0.transpose() * R1);
}
np::ndarray 
toNumPyArray(std::vector<double> _vec)
{
	int n = _vec.size();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i=0;i<n;i++)
	{
		dest[i] = _vec[i];
	}

	return array;
}
np::ndarray 
toNumPyArray(Eigen::VectorXd _vec)
{
	int n = _vec.rows();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i =0; i<n; i++)
	{
		dest[i] = _vec[i];
	}

	return array;
}
np::ndarray 
toNumPyArray(Eigen::MatrixXd _mat)
{
	int n = _mat.rows();
	int m = _mat.cols();

	p::tuple shape = p::make_tuple(n, m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			dest[index++] = _mat(i,j);
		}
	}

	return array;
}
np::ndarray 
toNumPyArray(std::vector<Eigen::VectorXd> _mat)
{
	int n = _mat.size();
	int m;
	if(n == 0)
		m = 0;
	else  
		m = _mat[0].rows();

	p::tuple shape = p::make_tuple(n, m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			dest[index++] = _mat[i][j];
		}
	}

	return array;
}
np::ndarray 
toNumPyArray(std::vector<std::vector<double>> _mat)
{
	int n = _mat.size();
	int m = _mat[0].size();

	p::tuple shape = p::make_tuple(n, m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			dest[index++] = _mat[i][j];
		}
	}

	return array;
}
Eigen::VectorXd 
toEigenVector(np::ndarray _array, int _n)
{
	Eigen::VectorXd vec(_n);

	float* srcs = reinterpret_cast<float*>(_array.get_data());

	for(int i=0; i<_n; i++)
	{
		vec[i] = srcs[i];
	}
	return vec;
}
Eigen::MatrixXd 
toEigenMatrix(np::ndarray _array, int _n, int _m)
{
	Eigen::MatrixXd mat(_n, _m);

	float* srcs = reinterpret_cast<float*>(_array.get_data());

	int index = 0;
	for(int i=0; i<_n; i++)
	{
		for(int j=0; j<_m; j++)
		{
			mat(i,j) = srcs[index++];
		}
	}
	return mat;
}
bool 
isSame(Eigen::VectorXd _v1, Eigen::VectorXd _v2)
{
	for(int i = 0; i < _v1.rows(); i++) {
		if(abs(_v1[i] - _v2[i]) > 1e-4)
			return false;
	}
	return true;
}
template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
       	*(result++) = item;
    }
}

std::vector<std::string> 
split(const std::string &s, char delim=' ') {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}
};