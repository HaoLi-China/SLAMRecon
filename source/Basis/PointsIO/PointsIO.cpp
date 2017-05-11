//Copyright 2016 - 2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon

#include "PointsIO.h"

PointsIO::PointsIO(){

}

PointsIO::~PointsIO(){

}

bool PointsIO::savePLYfile(const std::string& filename, const std::vector<Vector3f> &points, const std::vector<Vector3f>& normals, const std::vector<Vector3u>& colors){
	if (points.size() == 0) {
		std::cout << "points is null" << std::endl;
		return false;
	}

	p_ply ply = ply_create(filename.c_str(), PLY_LITTLE_ENDIAN, NULL, 0, NULL);

	if (ply == NULL) {
		std::cout << "could not open the file" << std::endl;
		return false;
	}

	//////////////////////////////////////////////////////////////////////////

	if (!ply_add_comment(ply, "ply file")) {
		std::cout << "unable to add comment" << std::endl;
		ply_close(ply);
		return false;
	}

	if (!ply_add_element(ply, "vertex", points.size())) {
		std::cout << "unable to add element \'vertex\'" << std::endl;
		ply_close(ply);
		return false;
	}

	e_ply_type length_type, value_type;
	length_type = value_type = static_cast<e_ply_type>(-1);
	std::string pos[3] = { "x", "y", "z" };
	for (unsigned int i = 0; i < 3; ++i) {
		if (!ply_add_property(ply, pos[i].c_str(), PLY_FLOAT, length_type, value_type)) {
			std::cout << "unable to add property \'" << pos[i].c_str() << "\'" << std::endl;
			ply_close(ply);
			return false;
		}
	}

	if (normals.size() == points.size()) {
		std::string normal[3] = { "nx", "ny", "nz" };
		for (unsigned int i = 0; i < 3; ++i) {
			if (!ply_add_property(ply, normal[i].c_str(), PLY_FLOAT, length_type, value_type)) {
				std::cout << "unable to add property \'" << normal[i].c_str() << "\'" << std::endl;
				ply_close(ply);
				return false;
			}
		}
	}

	if (colors.size() == points.size()) {
		std::string color[4] = { "red", "green", "blue" };
		for (unsigned int i = 0; i < 3; ++i) {
			if (!ply_add_property(ply, color[i].c_str(), PLY_UCHAR, length_type, value_type)) {
				std::cout << "unable to add property \'" << color[i].c_str() << "\'" << std::endl;
				ply_close(ply);
				return false;
			}
		}
	}

	if (!ply_write_header(ply)) {
		std::cout << filename.c_str() << ": invalid PLY file" << std::endl;
		ply_close(ply);
		return false;
	}

	//////////////////////////////////////////////////////////////////////////

	for (int i = 0; i < points.size(); i++){
		ply_write(ply, points[i].x);
		ply_write(ply, points[i].y);
		ply_write(ply, points[i].z);

		if (normals.size() == points.size()) {
			ply_write(ply, normals[i].x);
			ply_write(ply, normals[i].y);
			ply_write(ply, normals[i].z);
		}

		if (colors.size() == points.size()) {
			ply_write(ply, colors[i].x);
			ply_write(ply, colors[i].y);
			ply_write(ply, colors[i].z);
		}
	}

	ply_close(ply);
	return true;
}

bool PointsIO::savePLYfile(const std::string& filename, const std::vector<Vector3f>& points, const std::vector<Vector3f>& normals, const Vector3u &color){
	std::vector<Vector3u> colors;
	for (int i = 0; i < points.size(); i++){
		colors.push_back(color);
	}

	bool res = savePLYfile(filename, points, normals, colors);

	return res;
}