#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);
	Eigen::Matrix3d GetRotation() { return Tout.rotation();}
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);
	void RotateInSystem(const Eigen::Matrix3d& mat, const Eigen::Quaterniond rot);
	void MyRotate(const Eigen::Quaterniond rot);
	void MyRotate(const Eigen::Matrix3d &rot);
	void MyScale(Eigen::Vector3d amt);
	void SetCenterOfRotation(Eigen::Vector3d center);
	Eigen::Vector3d GetCenterOfRotation() { return -Tout.translation(); };

	Eigen::Matrix3d GetRotation() const{ return Tout.rotation().matrix(); }
	void SetRotation(Eigen::Matrix3d toSet) { Tout.rotate(toSet); }

	virtual ~Movable() {}
private:
	Eigen::Affine3d Tout,Tin;
};

