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
//  CLASS RegularGrid
//
//=============================================================================


#ifndef ISOEX_REGULARGRID_HH
#define ISOEX_REGULARGRID_HH


//== INCLUDES =================================================================

#include <IsoEx/Grids/Grid.hh>

//== NAMESPACES ===============================================================

namespace IsoEx {

//== CLASS DEFINITION =========================================================

	      
/** \class RegularGrid RegularGrid.hh <IsoEx/Grids/RegularGrid.hh>
    This class implements a regular 3D grid.
    \ingroup grids
*/	      
class RegularGrid : public Grid
{
public:
   
  /** Constructor: given the implicit to be sampled, the grids extend
      in 3-space (origin and 3 axes) as well as the resolution (number
      of steps) of the axes. The grid will contain
      _x_res*_y_res*_z_res points and 
      (_x_res-1)*(_y_res-1)*(_z_res-1) cubes. 

      \note The resolution of each axis has to be less than 1024. This is
      to make sure that a cube or point can be represented by one
      integer.
  */
  RegularGrid(const OpenMesh::Vec3f&  _origin,
	      const OpenMesh::Vec3f&  _x_axis,
	      const OpenMesh::Vec3f&  _y_axis,
	      const OpenMesh::Vec3f&  _z_axis,
	      unsigned int            _x_res,
	      unsigned int            _y_res,
	      unsigned int            _z_res)
  { initialize(_origin, _x_axis, _y_axis, _z_axis, _x_res, _y_res, _z_res); }

  void initialize(const OpenMesh::Vec3f&  _origin,
		  const OpenMesh::Vec3f&  _x_axis,
		  const OpenMesh::Vec3f&  _y_axis,
		  const OpenMesh::Vec3f&  _z_axis,
		  unsigned int            _x_res,
		  unsigned int            _y_res,
		  unsigned int            _z_res);



  //------------------------------------------------------- mandatory interface

  /// Return number of cubes
  unsigned int n_cubes() const { return n_cubes_; }

  /// Return number of points
  unsigned int n_points() const { return n_points_; }

  /// Return the PointIdx of the \b _corners'th corner of the cube \b _idx
  PointIdx point_idx(CubeIdx _idx, unsigned char _corner) const;

  /// Return the 3D point refered to by \b _idx.
  OpenMesh::Vec3f  point(PointIdx _idx) const;

  /// Return the 3D point refered to by x,y,z.
  OpenMesh::Vec3f  point(unsigned int _x,
												 unsigned int _y,
												 unsigned int _z) const;

  const OpenMesh::Vec3f& origin() const { return origin_; }
  const OpenMesh::Vec3f& x_axis() const { return x_axis_; }
  const OpenMesh::Vec3f& y_axis() const { return y_axis_; }
  const OpenMesh::Vec3f& z_axis() const { return z_axis_; }
  unsigned int x_resolution() const { return x_res_; }
  unsigned int y_resolution() const { return y_res_; }
  unsigned int z_resolution() const { return z_res_; }


private:

  OpenMesh::Vec3f   origin_, x_axis_, y_axis_, z_axis_, dx_, dy_, dz_;
  unsigned int      x_res_, y_res_, z_res_, n_cubes_, n_points_;
  CubeIdx           offsets_[8];
};


//=============================================================================
} // namespace IsoEx
//=============================================================================
#endif // ISOEX_REGULARGRID_HH defined
//=============================================================================

