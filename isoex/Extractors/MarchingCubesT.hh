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
//  CLASS MarchingCubesT
//
//=============================================================================

#ifndef ISOEX_MARCHINGCUBEST_HH
#define ISOEX_MARCHINGCUBEST_HH

//== INCLUDES =================================================================

#include <IsoEx/Extractors/Edge2VertexMapT.hh>
#include <IsoEx/Grids/Grid.hh>
#include <map>

//== NAMESPACES ===============================================================

namespace IsoEx {

//== CLASS DEFINITION =========================================================


/** \class MarchingCubesT MarchingCubesT.hh <IsoEx/Extractors/MarchingCubesT.hh>
    This class implements the well known Marching Cubes algorithm.
    The 0-level iso-surface is extracted in the constructor. Use it through 
    the convenience function 
    <b> IsoEx::marching_cubes() </b>.
    \ingroup extractors
*/	      
template <class Mesh>
class MarchingCubesT
{
public:
   
  MarchingCubesT(const Grid& _grid, Mesh& _mesh);

  
private:

  typedef Grid::PointIdx      PointIdx;
  typedef Grid::CubeIdx       CubeIdx;
  typedef Grid::CubeIterator  CubeIterator;

  typedef typename Mesh::Point         Point;
  typedef typename Mesh::VertexHandle  VertexHandle;
  typedef typename Mesh::FaceHandle    FaceHandle;


  void process_cube(CubeIdx _idx);
  VertexHandle add_vertex(PointIdx _p0, PointIdx _p1);


  const Grid&      grid_;
  Mesh&            mesh_;

  // maps an edge to the sample vertex generated on it
  Edge2VertexMapT<PointIdx, VertexHandle> edge2vertex_;
};


//-----------------------------------------------------------------------------


/** Convenience wrapper for the Marching Cubes algorithm.
    \see IsoEx::MarchingCubesT
    \ingroup extractors
*/	      
template <class Mesh>
void marching_cubes(const Grid& _grid, Mesh& _mesh)
{
  MarchingCubesT<Mesh> mc(_grid, _mesh);
}


//=============================================================================
} // namespace IsoEx
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(ISOEX_MARCHINGCUBEST_C)
#define ISOEX_MARCHINGCUBEST_TEMPLATES
#include "MarchingCubesT.cc"
#endif
//=============================================================================
#endif // ISOEX_MARCHINGCUBEST_HH defined
//=============================================================================
