//Copyright 2016 - 2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon

#include "Utility.h"
#include "ICP.h"

using namespace Basis;

Utility::Utility(){

}

Utility::~Utility(){

}

void Utility::computeTransformation(const std::vector<EVector3> &source, const std::vector<EVector3> &target, Eigen::Matrix4d &mat){
	Eigen::Matrix3Xd source_vertices;
	Eigen::Matrix3Xd target_vertices;

	source_vertices.resize(3, source.size());
	target_vertices.resize(3, target.size());

	for (int i = 0; i < source.size(); i++){
		source_vertices(0, i) = (double)source[i].x();
		source_vertices(1, i) = (double)source[i].y();
		source_vertices(2, i) = (double)source[i].z();
	}

	for (int i = 0; i < target.size(); i++){
		target_vertices(0, i) = (double)target[i].x();
		target_vertices(1, i) = (double)target[i].y();
		target_vertices(2, i) = (double)target[i].z();
	}

	Eigen::Affine3d affine = RigidMotionEstimator::point_to_point(source_vertices, target_vertices);
	mat = affine.matrix().cast<double>();;
}

Eigen::Matrix4d Utility::inverseTransformation(const Eigen::Matrix4d& mat)
{
	Eigen::Matrix3d rot = mat.block<3, 3>(0, 0);
	EVector3 t(mat(0, 3), mat(1, 3), mat(2, 3));

	Eigen::Matrix3d rot_1 = rot.inverse();
	EVector3 newT = -rot_1 * t;

	Eigen::Matrix4d inverseMat;

	inverseMat.block<3, 3>(0, 0) = rot_1;
	inverseMat.block<3, 1>(0, 3) = newT;

	return inverseMat;
}

Matrix3f Utility::createRotation(const Vector3f & _axis, float angle)
{
	Vector3f axis = normalize(_axis);
	float si = sinf(angle);
	float co = cosf(angle);

	Matrix3f ret;
	ret.setIdentity();

	ret *= co;
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ret.at(c, r) += (1.0f - co) * axis[c] * axis[r];

	Matrix3f skewmat;
	skewmat.setZeros();
	skewmat.at(1, 0) = -axis.z;
	skewmat.at(0, 1) = axis.z;
	skewmat.at(2, 0) = axis.y;
	skewmat.at(0, 2) = -axis.y;
	skewmat.at(2, 1) = axis.x;
	skewmat.at(1, 2) = -axis.x;
	skewmat *= si;
	ret += skewmat;

	return ret;
}
