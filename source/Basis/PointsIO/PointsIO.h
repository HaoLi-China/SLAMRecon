/**
* This file is about reading and writing point cloud.
*
* Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
*/

#ifndef _POINTSIO_H
#define _POINTSIO_H

#include <stdio.h>
#include <iostream>
#include <vector>

#include "rply.h"
#include "../Define.h"

class PointsIO
{
public:
	PointsIO();
	~PointsIO();

	static bool savePLYfile(const std::string& filename, const std::vector<Vector3f>& points, const std::vector<Vector3f>& normals, const std::vector<Vector3u>& colors);
	static bool savePLYfile(const std::string& filename, const std::vector<Vector3f>& points, const std::vector<Vector3f>& normals, const Vector3u &color);
};

#endif // POINTSIO_H
