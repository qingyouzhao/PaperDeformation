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
//  CLASS Vector
//
//=============================================================================

#ifndef VECTOR_HH_
#define VECTOR_HH_
#include <cmath>
#include <OpenMesh/Core/Geometry/Vector11T.hh>
#include <OpenMesh/Core/Geometry/VectorT_inc.hh>
template <class T, unsigned int C>
struct Vector;

typedef Vector<double, 1> Vector1d;
typedef Vector<double, 2> Vector2d;
typedef Vector<double, 3> Vector3d;
typedef Vector<double, 4> Vector4d;

typedef Vector<float, 1> Vector1f;
typedef Vector<float, 2> Vector2f;
typedef Vector<float, 3> Vector3f;
typedef Vector<float, 4> Vector4f;
// only for save 3 ints
typedef Vector<int, 3> Vector3i;

/**
 * simple Vector class
 */
template <class T, unsigned int C>
struct Vector {
  T v[C];

  Vector() {}

  Vector(T v0, T v1) {
    v[0] = v0;
    v[1] = v1;
  }

  Vector(T v0, T v1, T v2) {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }

  Vector(const OpenMesh::VectorT<double, C>& open_mesh_v)
  {
	  for (unsigned i = 0; i < C; i++) v[i] = open_mesh_v[i];
  }

  Vector(const OpenMesh::VectorT<float, C>& open_mesh_v)
  {
	  for (unsigned i = 0; i < C; i++) v[i] = open_mesh_v[i];
  }

  Vector& operator=(const OpenMesh::VectorT<double, C>& open_mesh_v)
  {
	  for (unsigned i = 0; i < C; i++) v[i] = open_mesh_v[i];
	  return *this;
  }
  Vector& operator=(const OpenMesh::VectorT<float, C>& open_mesh_v)
  {
	  for (unsigned i = 0; i < C; i++) v[i] = open_mesh_v[i];
	  return *this;
  }

  T& operator[](unsigned int i) { return v[i]; }

  const T& operator[](unsigned int i) const { return v[i]; }

  Vector& operator+=(const Vector& o) {
    for (unsigned i = 0; i < C; i++) v[i] += o.v[i];
    return *this;
  }

  Vector& operator-=(const Vector& o) {
    for (unsigned i = 0; i < C; i++) v[i] -= o.v[i];
    return *this;
  }

  Vector& operator*=(T o) {
    for (unsigned i = 0; i < C; i++) v[i] *= o;
    return *this;
  }

  Vector& operator/=(T o) {
    for (unsigned i = 0; i < C; i++) v[i] /= o;
    return *this;
  }

  Vector& normalize() { return (*this) /= length(*this); }

  void fill(T f) {
    for (unsigned i = 0; i < C; i++) v[i] = f;
  }

  void find_best_axis_vectors(Vector<T, 3>& axis1, Vector<T, 3>& axis2) const
  {
	  const T nx = abs(v[0]);
	  const T ny = abs(v[1]);
	  const T nz = abs(v[2]);

	  if (ny > nx && ny > nz) axis1 = Vector<T, 3>(0, 0, 1); // up is dominant in this vector, we try projecting up system in the world forward direction(z).
	  else					axis1 = Vector<T, 3>(0, 1, 0); // up is not dominant, we try to project up to up still

	  axis1 = (axis1 - *this * (axis1 | *this)).normalize();
	  axis2 = axis1 % *this;
  }
};

template <class T, unsigned int C>
inline Vector<T, C> operator-(const Vector<T, C>& v1) {
  Vector<T, C> res;
  for (unsigned i = 0; i < C; i++) res.v[i] = -v1.v[i];
  return res;
}

template <class T, unsigned int C>
inline Vector<T, C> operator+(const Vector<T, C>& v1, const Vector<T, C>& v2) {
  Vector<T, C> res;
  for (unsigned i = 0; i < C; i++) res.v[i] = v1.v[i] + v2.v[i];
  return res;
}

template <class T, unsigned int C>
inline Vector<T, C> operator-(const Vector<T, C>& v1, const Vector<T, C>& v2) {
  Vector<T, C> res;
  for (unsigned i = 0; i < C; i++) res.v[i] = v1.v[i] - v2.v[i];
  return res;
}

template <class T, unsigned int C>
inline Vector<T, C> operator/(const Vector<T, C>& v, T s) {
  Vector<T, C> res;
  for (unsigned i = 0; i < C; i++) res.v[i] = v.v[i] / s;
  return res;
}

template <class T, unsigned int C>
inline Vector<T, C> operator*(const Vector<T, C>& v, T s) {
  Vector<T, C> res;
  for (unsigned i = 0; i < C; i++) res.v[i] = v.v[i] * s;
  return res;
}

template <class T, unsigned int C>
inline Vector<T, C> operator*(T s, const Vector<T, C>& v) {
  Vector<T, C> res;
  for (unsigned i = 0; i < C; i++) res.v[i] = v.v[i] * s;
  return res;
}

template <class T>
Vector<T, 3> operator%(const Vector<T, 3>& a, const Vector<T, 3>& b)
{
	return cross_product(a, b);
}

template <class T, unsigned int C>
inline T operator|(const Vector<T, C>& a, const Vector<T, C>& b)
{
	return dot_product(a, b);
}

template <class T>
Vector<T, 3> cross_product(const Vector<T, 3>& a, const Vector<T, 3>& b) {
  Vector<T, 3> r;
  r.v[0] = a.v[1] * b.v[2] - a.v[2] * b.v[1];
  r.v[1] = a.v[2] * b.v[0] - a.v[0] * b.v[2];
  r.v[2] = a.v[0] * b.v[1] - a.v[1] * b.v[0];
  return r;
}

template <class T, unsigned int C>
inline T dot_product(const Vector<T, C>& a, const Vector<T, C>& b) {
  T sum = a.v[0] * b.v[0];
  for (unsigned i = 1; i < C; i++) sum += a.v[i] * b.v[i];
  return sum;
}

template <class T, unsigned int C>
T length2(const Vector<T, C>& a) {
  return (T)dot_product(a, a);
}

template <class T, unsigned int C>
T length(const Vector<T, C>& a) {
  return (T)sqrt(length2(a));
}



#endif /*VECTOR_HH_*/
