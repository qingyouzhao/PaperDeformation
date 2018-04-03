/*===========================================================================*\
 *                                                                           *
 *                                IsoEx                                      *
 *        Copyright (C) 2002 by Computer Graphics Group, RWTH Aachen         *
 *                         www.rwth-graphics.de                              *
 *                                                                           *
 *---------------------------------------------------------------------------* 
 *                                                                           *
 *                                License                                    *
 *                                                                           *
 *  This library is free software; you can redistribute it and/or modify it  *
 *  under the terms of the GNU Library General Public License as published   *
 *  by the Free Software Foundation, version 2.                              *
 *                                                                           *
 *  This library is distributed in the hope that it will be useful, but      *
 *  WITHOUT ANY WARRANTY; without even the implied warranty of               *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU        *
 *  Library General Public License for more details.                         *
 *                                                                           *
 *  You should have received a copy of the GNU Library General Public        *
 *  License along with this library; if not, write to the Free Software      *
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                *
 *                                                                           *
\*===========================================================================*/

//=============================================================================
//
//  CLASS RegularGrid - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include <IsoEx/Grids/RegularGrid.hh>

//== NAMESPACES ===============================================================

namespace IsoEx {

//== IMPLEMENTATION ========================================================== 


void
RegularGrid::
initialize(const OpenMesh::Vec3f&  _origin,
	   const OpenMesh::Vec3f&  _x_axis,
	   const OpenMesh::Vec3f&  _y_axis,
	   const OpenMesh::Vec3f&  _z_axis,
	   unsigned int            _x_res,
	   unsigned int            _y_res,
	   unsigned int            _z_res)
{
  origin_ = _origin;
  x_axis_ = _x_axis;
  y_axis_ = _y_axis;
  z_axis_ = _z_axis;
  x_res_ = _x_res;
  y_res_ = _y_res;
  z_res_ = _z_res;

  // CubeIdx and PointIdx have 32 bits 
  // -> its 3 components should require <= 10 bits
  assert(x_res_ < 1024);
  assert(y_res_ < 1024);
  assert(z_res_ < 1024);

  n_cubes_  = (_x_res-1) * (_y_res-1) * (_z_res-1);
  n_points_ = _x_res * _y_res * _z_res;

  dx_ = x_axis_ / (float)(x_res_-1);
  dy_ = y_axis_ / (float)(y_res_-1);
  dz_ = z_axis_ / (float)(z_res_-1);

  offsets_[0] = 0;
  offsets_[1] = 1;
  offsets_[2] = 1 + x_res_;
  offsets_[3] =     x_res_;
  offsets_[4] =              x_res_*y_res_;
  offsets_[5] = 1          + x_res_*y_res_;
  offsets_[6] = 1 + x_res_ + x_res_*y_res_;
  offsets_[7] =     x_res_ + x_res_*y_res_;
}


//-----------------------------------------------------------------------------


RegularGrid::PointIdx
RegularGrid::
point_idx(CubeIdx _idx, unsigned char _corner) const
{
  assert(_corner < 8);

  // get cube coordinates
  unsigned int X(x_res_-1), Y(y_res_-1);
  unsigned int x = _idx % X;  _idx /= X;
  unsigned int y = _idx % Y;  _idx /= Y;
  unsigned int z = _idx;

  // transform to point coordinates
  _idx = x + y*x_res_ + z*x_res_*y_res_;

  // add offset
  return _idx + offsets_[_corner];
}


//-----------------------------------------------------------------------------


OpenMesh::Vec3f
RegularGrid::
point(PointIdx _idx) const
{
  unsigned int x = _idx % x_res_;  _idx /= x_res_;
  unsigned int y = _idx % y_res_;  _idx /= y_res_;
  unsigned int z = _idx;

  return origin_ + dx_*static_cast<float>(x)
	             + dy_*static_cast<float>(y)
				 + dz_*static_cast<float>(z);
}


//-----------------------------------------------------------------------------


OpenMesh::Vec3f
RegularGrid::
point(unsigned int _x,
			unsigned int _y,
			unsigned int _z) const
{
  return origin_ + dx_*static_cast<float>(_x)
		+ dy_*static_cast<float>(_y)
		+ dz_*static_cast<float>(_z);
}


//=============================================================================
} // namespace IsoEx
//=============================================================================
