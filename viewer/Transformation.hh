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
//  CLASS Transformation
//
//=============================================================================

#ifndef TRANSFORMATION_HPP_
#define TRANSFORMATION_HPP_

#include <vector>
#include "Matrix.hh"
#include "Vector.hh"
#include <Eigen/Geometry>


/**
 * Transformation class
 *
 * contains a rotation and translation defining a rigid transformation
 */
class Transformation {
 public:
  /// constructor: identity transformation
  Transformation();

  /// constructor: translation
  Transformation(float tx, float ty, float tz);

  /// constructor: rotation around axis
  Transformation(float angle, Vector3f axis);

  /// constructor: rotation and translation, assume we just get the eigen quaternion directly
  Transformation(const Eigen::Quaternion<float>& Q, const Vector3f& T);


  /// constructor: with a rotation matrix and translation vector

  /// set identity transformation
  void set_identity();

  /// apply transformation to current OpenGL Matrix
  void apply_gl();

  /// retrieve curren OpenGL transformation
  static Transformation retrieve_gl();

  /// concatenate two transformations
  Transformation operator*(const Transformation& o) const;

  /// return inverse transformation
  Transformation inverse();

  /// Transform point
  Vector3d transformPoint(const Vector3d& p) const;

  OpenMesh::Vec3f transformPoint(const OpenMesh::Vec3f& p) const;
  OpenMesh::Vec3f operator*(const OpenMesh::Vec3f& p) const;

  /// Transform vector
  Vector3d transformVector(const Vector3d& v) const;

  OpenMesh::Vec3f transformVector(const OpenMesh::Vec3f& p) const;


  /// Transform points
  std::vector<Vector3d> transformPoints(const std::vector<Vector3d>& ps);

  /// Transform vectors
  std::vector<Vector3d> transformVectors(const std::vector<Vector3d>& vs);

  /// A nicedly formated string form of this rotaion
  std::string to_string();

  Matrix3x3d rotation_;
  Vector3d translation_;
};

#endif /* TRANSFORMATION_HPP_ */
