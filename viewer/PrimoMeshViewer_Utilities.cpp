#include "PrimoMeshViewer.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <glm/gtc/quaternion.hpp>

const LinearColor LinearColor::RED(1.f,0.f,0.f);
const LinearColor LinearColor::BLUE(0.f,0.f,1.0f);
const LinearColor LinearColor::GREEN(0.f,1.f,0.f);
const LinearColor LinearColor::BLACK(0.f,0.f,0.f);
const LinearColor LinearColor::GREY(0.5f,0.5f,0.5f);
const LinearColor LinearColor::WHITE(1.f,1.f,1.f);
const LinearColor LinearColor::YELLOW(1.f,1.f,0.f);
const LinearColor LinearColor::CYAN(0.f,1.f,1.f);
const LinearColor LinearColor::MAGENTA(1.f,0.f,1.f);
const LinearColor LinearColor::ORANGE(1.f,0.5f,0.f);
const LinearColor LinearColor::PURPLE(0.5f, 0.f,1.f);
const LinearColor LinearColor::TURQUOISE(0.f, 0.5f, 0.5f);
const LinearColor LinearColor::SILVER(0.8f,0.8f,0.8f);
const LinearColor LinearColor::EMERALD(0.2f,0.8f,0.4f);

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

void PrimoMeshViewer::draw_debug_lines()
{
	for (DebugLine& line : debug_lines_)
	{
		glLineWidth(line.width_);
		glColor4fv(line.color_.rgba_);
		glBegin(GL_LINES);
		glVertex3f(line.from_[0], line.from_[1], line.from_[2]);
		glVertex3f(line.to_[0], line.to_[1], line.to_[2]);
		glEnd();
	}
	debug_lines_.clear();
}


void PrimoMeshViewer::add_debug_arrow(Vec3f& from, Vec3f& to, float arrow_size, LinearColor color)
{
	DebugLine line(from, to, arrow_size, color);
}

void PrimoMeshViewer::add_debug_coordinate(Transformation& world_transform, float size)
{

}