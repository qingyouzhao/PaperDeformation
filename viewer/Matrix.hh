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
//  CLASS Matrix
//
//=============================================================================

#ifndef MATRIX_HH_
#define MATRIX_HH_

#include "Vector.hh"
#include <OpenMesh/Core/Geometry/VectorT.hh>

template <class T, unsigned int R, unsigned int C>
struct Matrix;

typedef Matrix<double, 3, 3> Matrix3x3d;
typedef Matrix<double, 4, 4> Matrix4x4d;

typedef Matrix<float, 3, 3> Matrix3x3f;
typedef Matrix<float, 4, 4> Matrix4x4f;

/**
 * simple Matrix class
 */
template <class T, unsigned int R, unsigned int C>
struct Matrix {
  /// data
  T v[C][R];
  Matrix() { set_identity(); }

  /// access column
  T* operator[](unsigned int i) { return v[i]; }

  /// access column
  const T* operator[](unsigned int i) const { return v[i]; }

  /// access element (r,c)
  T& operator()(unsigned int i, unsigned int j) { return v[j][i]; }

  /// access element (r,c)
  const T& operator()(unsigned int i, unsigned int j) const { return v[j][i]; }

  /// add matrix
  Matrix& operator+=(const Matrix& o) {
    for (unsigned int i = 0; i < C; i++) {
      for (unsigned int j = 0; j < R; j++) {
        v[i][j] += o.v[i][j];
      }
    }
    return *this;
  }

  /// subtract matrix
  Matrix& operator-=(const Matrix& o) {
    for (unsigned int i = 0; i < C; i++) {
      for (unsigned int j = 0; j < R; j++) {
        v[i][j] -= o.v[i][j];
      }
    }
    return *this;
  }

  /// fill matrix with element
  void fill(T f) {
    for (unsigned int i = 0; i < C; i++) {
      for (unsigned int j = 0; j < R; j++) {
        v[i][j] = f;
      }
    }
  }

  /// set identity matrix
  void set_identity() {
    for (unsigned int i = 0; i < C; i++) {
      for (unsigned int j = 0; j < R; j++) {
        v[i][j] = ((i == j) ? T(1.0) : T(0.0));
      }
    }
  }

  /// calculate transpose of matrix
  Matrix<T, C, R> transpose() const {
    Matrix<T, C, R> mat;
    for (unsigned i = 0; i < C; i++) {
      for (unsigned j = 0; j < R; j++) {
        mat[j][i] = v[i][j];
      }
    }
    return mat;
  }
};

/// subtract two matrices
template <class T, unsigned int R, unsigned int C>
Matrix<T, R, C> operator-(const Matrix<T, R, C>& o) {
  Matrix<T, R, C> mat;
  for (unsigned int i = 0; i < C; i++) {
    for (unsigned int j = 0; j < R; j++) {
      mat.v[i][j] = -o.v[i][j];
    }
  }
  return mat;
}

/// retrieve column vector
template <class T, unsigned int R, unsigned int C>
Vector<T, R> get_col(const Matrix<T, R, C>& m, unsigned i) {
  Vector<T, R> res;
  for (unsigned j = 0; j < R; j++) res.v[j] = m.v[i][j];
  return res;
}

/// set column vector
template <class T, unsigned int C, unsigned int R>
void set_col(Matrix<T, R, C>& m, unsigned i, const Vector<T, R>& a) {
  for (unsigned j = 0; j < R; j++) m.v[i][j] = a.v[j];
}

/// retrieve row vector
template <class T, unsigned int R, unsigned int C>
Vector<T, C> get_row(const Matrix<T, R, C>& m, unsigned i) {
  Vector<T, C> res;
  for (unsigned j = 0; j < C; j++) res.v[j] = m.v[j][i];
  return res;
}

/// set row vector
template <class T, unsigned int R, unsigned int C>
void set_row(Matrix<T, R, C>& m, unsigned i, const Vector<T, C>& a) {
  for (unsigned j = 0; j < C; j++) m.v[j][i] = a.v[j];
}

/// multiply matrix and vector
template <class T, unsigned int R, unsigned int C>
inline Vector<T, C> operator*(const Matrix<T, R, C>& m, const Vector<T, C>& a) {
  Vector<T, R> res;

  for (unsigned int i = 0; i < R; i++) {
    res.v[i] = m.v[0][i] * a.v[0];
    for (unsigned int j = 1; j < C; j++) {
      res.v[i] += m.v[j][i] * a.v[j];
    }
  }

  return res;
}
/// multiply matrix and vector
template <class T, unsigned int R, unsigned int C>
inline OpenMesh::Vec3f operator*(const Matrix<T, R, C>& m, const OpenMesh::Vec3f& a) {
  assert(R == 3 && C == 3);
  OpenMesh::Vec3f res;

  for (unsigned int i = 0; i < R; i++) {
    res[i] = m.v[0][i] * a[0];
    for (unsigned int j = 1; j < C; j++) {
      res[i] += m.v[j][i] * a[j];
    }
  }

  return res;
}

// multiply two matrices
template <class T, unsigned int R, unsigned int C1, unsigned int C2>
Matrix<T, R, C2> operator*(const Matrix<T, R, C1>& m1,
                           const Matrix<T, C1, C2>& m2) {
  Matrix<T, R, C2> res;

  for (unsigned int i = 0; i < R; i++) {
    for (unsigned int j = 0; j < C2; j++) {
      res.v[j][i] = m1.v[0][i] * m2.v[j][0];
      for (unsigned int k = 1; k < C1; k++) {
        res.v[j][i] += m1.v[k][i] * m2.v[j][k];
      }
    }
  }

  return res;
}

template <class T, unsigned int R, unsigned int C>
inline Matrix<T, R, C> operator*(T s, const Matrix<T, R, C>& v) {
  Matrix<T, R, C> res;
  for (unsigned i = 0; i < C; i++) {
    for (unsigned j = 0; j < R; j++) {
      res.v[i][j] = v.v[i][j] * s;
    }
  }
  return res;
}

template <class T, unsigned int R, unsigned int C>
inline Matrix<T, R, C> operator*(const Matrix<T, R, C>& v, T s) {
  Matrix<T, R, C> res;
  for (unsigned i = 0; i < C; i++) {
    for (unsigned j = 0; j < R; j++) {
      res.v[i][j] = v.v[i][j] * s;
    }
  }
  return res;
}

template <class T, unsigned int R, unsigned int C>
inline Matrix<T, R, C> operator+(const Matrix<T, R, C>& v1,
                                 const Matrix<T, R, C>& v2) {
  Matrix<T, R, C> res;
  for (unsigned i = 0; i < C; i++) {
    for (unsigned j = 0; j < R; j++) {
      res.v[i][j] = v1.v[i][j] + v2.v[i][j];
    }
  }
  return res;
}

template <class T, unsigned int R, unsigned int C>
inline Matrix<T, R, C> operator-(const Matrix<T, R, C>& v1,
                                 const Matrix<T, R, C>& v2) {
  Matrix<T, R, C> res;
  for (unsigned i = 0; i < C; i++) {
    for (unsigned j = 0; j < R; j++) {
      res.v[i][j] = v1.v[i][j] - v2.v[i][j];
    }
  }
  return res;
}

// returns the inverse of matrix
template <class T>
inline T det(const Matrix<T, 3, 3>& mat) {
  return mat.v[0][0] * mat.v[1][1] * mat.v[2][2] +
         mat.v[1][0] * mat.v[2][1] * mat.v[0][2] +
         mat.v[2][0] * mat.v[0][1] * mat.v[1][2] -
         mat.v[0][0] * mat.v[2][1] * mat.v[1][2] -
         mat.v[1][0] * mat.v[0][1] * mat.v[2][2] -
         mat.v[2][0] * mat.v[1][1] * mat.v[0][2];
}

// returns the inverse of matrix
template <class T>
inline Matrix<T, 3, 3> inverse(const Matrix<T, 3, 3>& mat) {
  Matrix<T, 3, 3> inv;

  T det = mat.v[0][0] * mat.v[1][1] * mat.v[2][2] +
          mat.v[1][0] * mat.v[2][1] * mat.v[0][2] +
          mat.v[2][0] * mat.v[0][1] * mat.v[1][2] -
          mat.v[0][0] * mat.v[2][1] * mat.v[1][2] -
          mat.v[1][0] * mat.v[0][1] * mat.v[2][2] -
          mat.v[2][0] * mat.v[1][1] * mat.v[0][2];

  T idet = 1. / det;

  inv.v[0][0] = idet * (mat.v[1][1] * mat.v[2][2] - mat.v[1][2] * mat.v[2][1]);
  inv.v[1][0] = idet * (mat.v[2][0] * mat.v[1][2] - mat.v[2][2] * mat.v[1][0]);
  inv.v[2][0] = idet * (mat.v[1][0] * mat.v[2][1] - mat.v[1][1] * mat.v[2][0]);
  inv.v[0][1] = idet * (mat.v[2][1] * mat.v[0][2] - mat.v[2][2] * mat.v[0][1]);
  inv.v[1][1] = idet * (mat.v[0][0] * mat.v[2][2] - mat.v[0][2] * mat.v[2][0]);
  inv.v[2][1] = idet * (mat.v[2][0] * mat.v[0][1] - mat.v[2][1] * mat.v[0][0]);
  inv.v[0][2] = idet * (mat.v[0][1] * mat.v[1][2] - mat.v[0][2] * mat.v[1][1]);
  inv.v[1][2] = idet * (mat.v[1][0] * mat.v[0][2] - mat.v[1][2] * mat.v[0][0]);
  inv.v[2][2] = idet * (mat.v[0][0] * mat.v[1][1] - mat.v[0][1] * mat.v[1][0]);

  return inv;
}

#endif /*MATRIX_HH_*/
