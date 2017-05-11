/**
* This file defines some functions to deal with transformation matrix.
*
* Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
*/

#ifndef _UTILITY_H
#define _UTILITY_H

#include <vector>
#include "EigenDefine.h"
#include "Define.h"

namespace Basis
{
	class Utility
	{
	public:
		Utility();
		~Utility();


		//************************************
		// Method:    computeTransformation
		// FullName:  Basis::Utility::computeTransformation
		// Access:    public static 
		// Returns:   void
		// Qualifier: compute transformation matrix based on a set of points and their corre points
		// Parameter: const std::vector<EVector3> & source
		// Parameter: const std::vector<EVector3> & target
		// Parameter: Eigen::Matrix4d & mat
		//************************************
		static void computeTransformation(const std::vector<EVector3> &source, const std::vector<EVector3> &target, Eigen::Matrix4d &mat);
		
		//************************************
		// Method:    inverseTransformation
		// FullName:  Basis::Utility::inverseTransformation
		// Access:    public static 
		// Returns:   Eigen::Matrix4d
		// Qualifier: inverse transformation matrix
		// Parameter: const Eigen::Matrix4d & mat
		//************************************
		static Eigen::Matrix4d inverseTransformation(const Eigen::Matrix4d& mat);
		
		//************************************
		// Method:    createRotation
		// FullName:  Basis::Utility::createRotation
		// Access:    public static 
		// Returns:   Matrix3f
		// Qualifier: compute rotation matrix given a rotation axis and angle
		// Parameter: const Vector3f & axis
		// Parameter: float angle
		//************************************
		static Matrix3f createRotation(const Vector3f & axis, float angle);
	};
}
#endif