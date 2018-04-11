#include "PrimoMeshViewer.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <glm/gtc/quaternion.hpp>

float PrimoMeshViewer::calc_face_area(Mesh::FaceHandle _fh) const
{
	// TODO: Implement 
	if (!_fh.is_valid())
	{
		std::cout << "Calc face area received invalid face handle, returning 1" << std::endl;
		return 1.0f;
	}
	Mesh::FaceHalfedgeCCWIter fh_ccwit = mesh_.cfh_ccwbegin(_fh);
	Mesh::VertexHandle v0 = mesh_.from_vertex_handle(*fh_ccwit);
	Mesh::VertexHandle v1 = mesh_.to_vertex_handle(*fh_ccwit);
	fh_ccwit++;
	Mesh::VertexHandle v2 = mesh_.to_vertex_handle(*fh_ccwit);

	Vec3f v0v1 = mesh_.point(v1) - mesh_.point(v0);
	Vec3f v0v2 = mesh_.point(v2) - mesh_.point(v0);

	float area = (v0v1 % v0v2).norm() * 0.5f;
	return area;
}