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
	for (Transformation& t : g_debug_transformations_to_draw_local_optimization)
	{
		add_debug_coordinate(t, 100);
	}

	for (Arrow& a : g_debug_arrows_to_draw_local_optimizations)
	{
		add_debug_arrow(a.from, a.to, a.color, a.arrow_size);
	}

	for (DebugLine& line : debug_lines_)
	{
		glLineWidth(line.width_);
		glColor4fv(line.color_.rgba_);
		glBegin(GL_LINES);
		glVertex3f(line.from_[0], line.from_[1], line.from_[2]);
		glVertex3f(line.to_[0], line.to_[1], line.to_[2]);
		glEnd();
	}
	// flush the debug line buffer
	debug_lines_.clear();
}


void PrimoMeshViewer::add_debug_arrow(const Vector3d& from,const Vector3d& to, LinearColor color, double arrow_size)
{
	DebugLine line(from, to, 1.0f, color);
	debug_lines_.emplace_back(line);
	Vector3d dir = (to - from).normalize();
	Vector3d up(0, 1, 0);
	Vector3d left = dir % up;
	if (length(left) != 1.0f)
	{
		dir.find_best_axis_vectors(up, left);
	}

	Vector3d origin(0, 0, 0);
	// Make a transform
	Transformation tm(left,up,dir,origin);
	
	float arrow_sqrt = sqrt(arrow_size);

	DebugLine arrow1(to, to + tm.transformVector(Vector3d(-arrow_sqrt , 0, -arrow_sqrt)), 1.0f, color);
	debug_lines_.emplace_back(arrow1);
	DebugLine arrow2(to, to + tm.transformVector(Vector3d(arrow_sqrt, 0, -arrow_sqrt)), 1.0f, color );
	debug_lines_.emplace_back(arrow2);


}

void PrimoMeshViewer::add_debug_coordinate(Transformation& world_transform, double size, Transformation base_transform)
{
	add_debug_arrow(world_transform.translation_, world_transform.translation_ + world_transform.x_axis() * size, LinearColor::RED, 0.1f * size);
	add_debug_arrow(world_transform.translation_, world_transform.translation_ + world_transform.y_axis() * size, LinearColor::GREEN, 0.1f * size);
	add_debug_arrow(world_transform.translation_, world_transform.translation_ + world_transform.z_axis() * size, LinearColor::BLUE, 0.1f * size);
}


void PrimoMeshViewer::add_debug_line(Vector3d& from, Vector3d& to, LinearColor color, double width /*= 1.0f*/)
{
	DebugLine line(from, to, width, color);
	debug_lines_.emplace_back(line);
}


void PrimoMeshViewer::print_quaternion(Eigen::Quaternion<double>& Q)
{
	float half_theta = acos(Q.w());
	float sin_half_theta = sin(half_theta);
	float axis_x = Q.x() / sin_half_theta;
	float axis_y = Q.y() / sin_half_theta;
	float axis_z = Q.z() / sin_half_theta;

	std::cout << "== Begin Quaternion Log == " << std::endl;
	std::cout << "w = " << Q.w() << std::endl;
	std::cout << "x = " << Q.x() << std::endl;
	std::cout << "y = " << Q.y() << std::endl;
	std::cout << "z = " << Q.z() << std::endl;
	std::cout << "theta = " << half_theta * 2 << std::endl;
	std::cout << "around axis = " << std::endl;
	std::cout << axis_x << std::endl;
	std::cout << axis_y << std::endl;
	std::cout << axis_z << std::endl;
	std::cout << "== End Quaternion Log == " << std::endl;


}