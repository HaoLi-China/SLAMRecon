/**
* This file defines some types about vector and matrix based on Eigen.
*
* Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
*/

#ifndef _EIGENDEFINE_H
#define _EIGENDEFINE_H

#include <Eigen/Core>

typedef Eigen::Vector2i EVector2i;
typedef Eigen::Vector2d EVector2d;
typedef Eigen::Vector2d EVector2;
typedef Eigen::Vector2f EVector2f;
typedef Eigen::Matrix< short, 2, 1> EVector2s;
typedef Eigen::Vector3f EVector3f;
typedef Eigen::Vector3d EVector3d;
typedef Eigen::Vector3d EVector3;
typedef Eigen::Vector3i EVector3i;
typedef Eigen::Matrix< unsigned int, 3, 1> EVector3u;
typedef Eigen::Matrix< unsigned char, 3, 1> EVector3uc;
typedef Eigen::Vector4d EVector4d;
typedef Eigen::Vector4d EVector4;
typedef Eigen::Vector4f EVector4f;
typedef Eigen::Vector4i EVector4i;
typedef Eigen::Matrix< unsigned int, 4, 1> EVector4u;
typedef Eigen::Matrix< unsigned char, 4, 1> EVector4uc;

#endif //_EIGENDEFINE_H