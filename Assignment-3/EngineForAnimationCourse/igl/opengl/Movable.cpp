#include "Movable.h"
#include <iostream>
Movable::Movable()
{
	Tout = Eigen::Affine3d::Identity();
	Tin = Eigen::Affine3d::Identity();
}

Movable::Movable(const Movable &mov)
{
	Tout = mov.Tout;
	Tin = mov.Tin;
}

Eigen::Matrix4f Movable::MakeTransScale()
{
	return (Tout.matrix() * Tin.matrix()).cast<float>();
}

Eigen::Matrix4d Movable::MakeTransScaled()
{
	return (Tout.matrix() * Tin.matrix());
}

Eigen::Matrix4d Movable::MakeTransd()
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;

	return (Tout.matrix() * mat);
}

void Movable::MyTranslate(Eigen::Vector3d amt, bool preRotation)
{

	if (preRotation)
		Tout.pretranslate(amt);
	else
		Tout.translate(amt);
}

void Movable::TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt)
{
	Tout.pretranslate(rot.transpose() * amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
{
	Tin.pretranslate(-amt);
	Tout.pretranslate(amt);
}

// angle in radians
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, rotAxis.normalized()));
}

void Movable::RotateInSystem(Eigen::Matrix3d preRot, Eigen::Vector3d rotAxis, double angle)
{
	// counter rotate a third party rotation, and then counter previous self rotation
	Tout.rotate(Eigen::AngleAxisd(angle, Tout.rotation().transpose() * preRot.transpose() * rotAxis.normalized()));
}

double calcAngle(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
	double dot_prod = v1.dot(v2);
	dot_prod /= v1.norm() * v2.norm();
	dot_prod = std::max(std::min(dot_prod, 1.0), -1.0);
	return acos(dot_prod);
}

// Rotates angle around rotAxis in euler manner
void Movable::EulerRotation(Eigen::Vector3d rotAxis, double angle, bool limit)
{
	Eigen::Vector3d ea = GetRotation().eulerAngles(2, 0, 2);

	Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
	Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
	Eigen::Matrix3d Z1 = Eigen::AngleAxisd(ea[0], z_axis).matrix();
	Eigen::Matrix3d X = Eigen::AngleAxisd(ea[1], x_axis).matrix();
	Eigen::Matrix3d Z2 = Eigen::AngleAxisd(ea[2], z_axis).matrix();

	Eigen::Matrix3d R = Eigen::AngleAxisd(angle, rotAxis.normalized()).matrix();

	Eigen::Matrix3d res = Z1 * R * X * Z2;
	if (limit)
	{
		Eigen::Vector3d parent_z_axis = -(res.transpose() * z_axis);
		double alpha = calcAngle(parent_z_axis, z_axis);
		if (alpha < (M_PI / 6))
		{
			angle = angle > 0.0 ? angle - ((M_PI / 6) - alpha) : angle + ((M_PI / 6) - alpha);
			R = Eigen::AngleAxisd(angle, rotAxis.normalized()).matrix();
			res = Z1 * R * X * Z2;
		}
	}
	res = GetRotation().transpose() * res;
	Tout.rotate(res);
}

void Movable::MyRotate(const Eigen::Matrix3d &rot)
{
	Tout.rotate(rot);
}

void Movable::MyScale(Eigen::Vector3d amt)
{
	Tin.scale(amt);
}

// void Movable::TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preRotation)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * amt; //transpose instead of inverse
//	MyTranslate(v, preRotation);
// }
//
// void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * rotAxis; //transpose instead of inverse
//	MyRotate(v.normalized(), angle);
// }
//
//
// void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
//{
//	Tout.pretranslate(Tout.rotation().matrix().block<3, 3>(0, 0) * amt);
//	Tin.translate(-amt);
// }
//
// Eigen::Vector3d Movable::GetCenterOfRotation()
//{
//	return Tin.translation();
// }
