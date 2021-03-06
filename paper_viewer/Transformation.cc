//=============================================================================
//
//   Code framework for the lecture
//
//   "Surface Representation and Geometric Modeling"
//
//   Mark Pauly, Mario Botsch, Balint Miklos, and Hao Li
//
//   Copyright (C) 2007 by  Applied Geometry Group and
//                          Computer Graphics Laboratory, ETH Zurich
//
//-----------------------------------------------------------------------------
//
//                                License
//
//   This program is free software; you can redistribute it and/or
//   modify it under the terms of the GNU General Public License
//   as published by the Free Software Foundation; either version 2
//   of the License, or (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program; if not, write to the Free Software
//   Foundation, Inc., 51 Franklin Street, Fifth Floor,
//   Boston, MA  02110-1301, USA.
//
//=============================================================================
//=============================================================================
//
//  CLASS Transformation - IMPLEMENTATION
//
//=============================================================================

// Modified by Qingyou Zhao and Zejian Wang for better utility

//== INCLUDES =================================================================

#include "Transformation.hh"
#include <cmath>
#include <cstring>
#include "gl.hh"
#include <Eigen/Geometry>
// #include <glm/gtc/quaternion.hpp>

//== IMPLEMENTATION ==========================================================

Transformation::Transformation() { set_identity(); }



//=============================================================================
Transformation::Transformation(float tx, float ty, float tz) {
	set_identity();
	translation_ = Vector3d(tx, ty, tz);
}

//=============================================================================
Transformation::Transformation(float angle, Vector3f axis) {
	set_identity();

	double l = length(axis);
	if (l > 0) {
		double l1 = 1.0 / l;
		double x = axis[0] * l1;
		double y = axis[1] * l1;
		double z = axis[2] * l1;
		double s = sin(angle), c = cos(angle);
		double xs = x * s, ys = y * s, zs = z * s, c1 = 1.0 - c;
		double xx = c1 * x * x, yy = c1 * y * y, zz = c1 * z * z;
		double xy = c1 * x * y, xz = c1 * x * z, yz = c1 * y * z;
		rotation_[0][0] = xx + c;
		rotation_[0][1] = xy - zs;
		rotation_[0][2] = xz + ys;
		rotation_[1][0] = xy + zs;
		rotation_[1][1] = yy + c;
		rotation_[1][2] = yz - xs;
		rotation_[2][0] = xz - ys;
		rotation_[2][1] = yz + xs;
		rotation_[2][2] = zz + c;
	}
}

Transformation::Transformation(const Eigen::Quaternion<double>& Q, const Vector3d& T)
{
	set_identity();
	//

	assert(fabs(Q.norm() - 1.0f) < 1E-7);
	//{
		//std::cout << "Quaternion " << " is not normalized" << std::endl;
	//}
	// bool bUseUnrealVersion = false;

	// // Set up 3x3 rot from quat
	// const float x2 = Q.x() + Q.x();  const float y2 = Q.y() + Q.y();  const float z2 = Q.z() + Q.z();
	// const float xx = Q.x() * x2;   const float xy = Q.x() * y2;   const float xz = Q.x() * z2;
	// const float yy = Q.y() * y2;   const float yz = Q.y() * z2;   const float zz = Q.y() * z2;
	// const float wx = Q.w() * x2;   const float wy = Q.w() * y2;   const float wz = Q.w() * z2;

	Eigen::Matrix3d eigenR;
	eigenR = Q;
	Matrix3x3d& M = rotation_;
	for(int r = 0;r < 3;++r){
		for(int c = 0;c <3;++c){
			M(r, c) = eigenR(r, c);
		}
	}
	// Heck, just rewrite this from scratch to make sure I know what is happening 


	// M(0, 0) = 1.0f - (yy + zz);	M(0, 1) = xy - wz;				M(0, 2) = xz + wy;
	// M(1, 0) = xy + wz;			M(1, 1) = 1.0f - (xx + zz);		M(1, 2) = yz - wx;
	// M(2, 2) = xz - wy;			M(2, 1) = yz + wx;				M(2, 2) = 1.0f - (xx + yy);

	

	// set up translation

	translation_[0] = T[0];
	translation_[1] = T[1];
	translation_[2] = T[2];

}

Transformation::Transformation(const Vector3d& x_axis, const Vector3d& y_axis, const Vector3d& z_axis, const Vector3d& t)
{
	Matrix3x3d& R = rotation_;
	R(0, 0) = x_axis[0];
	R(0, 1) = x_axis[1];
	R(0, 2) = x_axis[2];

	R(1, 0) = y_axis[0];
	R(1, 1) = y_axis[1];
	R(1, 2) = y_axis[2];

	R(2, 0) = z_axis[0];
	R(2, 1) = z_axis[1];
	R(2, 2) = z_axis[2];

	translation_[0] = t[0];
	translation_[1] = t[1];
	translation_[2] = t[2];
}

//=============================================================================

void Transformation::set_identity() {
  rotation_.set_identity();
  translation_.fill(0);
}

//=============================================================================

Transformation Transformation::operator*(const Transformation& o) const {
  Transformation t;

  t.rotation_ = rotation_ * o.rotation_;
  t.translation_ = rotation_ * o.translation_ + translation_;

  return t;
}

OpenMesh::Vec3f Transformation::operator*(const OpenMesh::Vec3f& p) const
{
	return transformPoint(p);
}

//=============================================================================

// inverse rigid motion
Transformation Transformation::inverse() {
  Transformation t;
  t.rotation_ = rotation_.transpose();
  t.translation_ = -t.rotation_ * translation_;
  return t;
}

//=============================================================================

// Transform point
Vector3d Transformation::transformPoint(const Vector3d& p) const {
  return rotation_ * p + translation_;
}

//=============================================================================

// Transform vector
Vector3d Transformation::transformVector(const Vector3d& v) const {
  return rotation_ * v;
}

Vector3d Transformation::x_axis() const
{
	return Vector3d(rotation_(0, 0), rotation_(1, 0), rotation_(2, 0));
}

Vector3d Transformation::y_axis() const
{
	return Vector3d(rotation_(0, 1), rotation_(1, 1), rotation_(2, 1));
}

Vector3d Transformation::z_axis() const
{
	return Vector3d(rotation_(0, 2), rotation_(1, 2), rotation_(2, 2));
}

Vector3d Transformation::get_forward_vector() const
{
	return z_axis();
}

Vector3d Transformation::get_up_vector() const
{
	return y_axis();
}

Vector3d Transformation::get_left_vector() const
{
	return x_axis();
}

//=============================================================================

// Transform points
std::vector<Vector3d> Transformation::transformPoints(
    const std::vector<Vector3d>& ps) {
  std::vector<Vector3d> ps_out = ps;
  for (int i = 0; i < (int)ps.size(); i++) ps_out[i] = transformPoint(ps[i]);
  return ps_out;
}

//=============================================================================

// Transform vectors
std::vector<Vector3d> Transformation::transformVectors(
    const std::vector<Vector3d>& vs) {
  std::vector<Vector3d> vs_out = vs;
  for (int i = 0; i < (int)vs.size(); i++) vs_out[i] = transformVector(vs[i]);
  return vs_out;
}

std::string Transformation::to_string()
{
	std::string s;
	std::ostringstream oss;
	oss << "Rotation \n";
	oss << rotation_(0, 0)<< ' ' << rotation_(0, 1) << ' ' << rotation_(0, 2) <<  '\n';
	oss << rotation_(1, 0) << ' ' << rotation_(1, 1) << ' ' << rotation_(1, 2) << '\n';
	oss << rotation_(2, 0) << ' ' << rotation_(2, 1) << ' ' << rotation_(2, 2) << '\n';
	oss << "Translation \n";
	oss << translation_[0] << ' ' << translation_[1] << ' ' << translation_[2] << '\n';
	return oss.str();
}

//=============================================================================

/// apply transformation to current OpenGL matrix
void Transformation::apply_gl() {
  float data[16];
  memset(data, 0, sizeof(float) * 16);
  data[15] = 1;

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) data[4 * j + i] = rotation_[i][j];
  for (int i = 0; i < 3; i++) data[12 + i] = translation_[i];

  glMultMatrixf(data);
}

//=============================================================================

/// retrieve current OpenGL transformation
Transformation Transformation::retrieve_gl() {
  Transformation tr;

  double data[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, data);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) tr.rotation_[i][j] = data[4 * j + i];
  for (int i = 0; i < 3; i++) tr.translation_[i] = data[12 + i];

  return tr;
}

//=============================================================================
OpenMesh::Vec3f Transformation::transformPoint(const OpenMesh::Vec3f& p) const{
  return rotation_ * p + OpenMesh::Vec3f(translation_[0], translation_[1], translation_[2]);
}

OpenMesh::Vec3f Transformation::transformVector(const OpenMesh::Vec3f& v) const{
  return rotation_ * v;
}
