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
//  CLASS MarchingCubesT - IMPLEMENTATION
//
//=============================================================================

#define ISOEX_MARCHINGCUBEST_C

//== INCLUDES =================================================================

#include <IsoEx/Extractors/MarchingCubesT.hh>
#include <IsoEx/Extractors/MCTables.hh>
#include <vector>

//== NAMESPACES ===============================================================

namespace IsoEx {

//== IMPLEMENTATION ==========================================================


template <class Mesh>
MarchingCubesT<Mesh>::
MarchingCubesT(const Grid& _grid, Mesh& _mesh)
  : grid_(_grid),
    mesh_(_mesh)
{
  CubeIterator cube_it(grid_.begin()), cube_end(grid_.end());
  for (; cube_it!=cube_end; ++cube_it)
    process_cube(*cube_it);
}


//-----------------------------------------------------------------------------


template <class Mesh>
void
MarchingCubesT<Mesh>::
process_cube(CubeIdx _cidx)
{
  PointIdx           corner[8];
  VertexHandle       samples[12];
  unsigned char      cubetype(0);
  unsigned int       i;


  // get point indices of corner vertices
  for (i=0; i<8; ++i)
    corner[i] = grid_.point_idx(_cidx, i);


  // determine cube type
  for (i=0; i<8; ++i)
    if (grid_.scalar_distance(corner[i]) > 0.0)
      cubetype |= (1<<i);


  // trivial reject ?
  if (cubetype == 0 || cubetype == 255)
    return;


  // compute samples on cube's edges
  if (edgeTable[cubetype]&1)    samples[0]  = add_vertex(corner[0], corner[1]);
  if (edgeTable[cubetype]&2)    samples[1]  = add_vertex(corner[1], corner[2]);
  if (edgeTable[cubetype]&4)    samples[2]  = add_vertex(corner[3], corner[2]);
  if (edgeTable[cubetype]&8)    samples[3]  = add_vertex(corner[0], corner[3]);
  if (edgeTable[cubetype]&16)   samples[4]  = add_vertex(corner[4], corner[5]);
  if (edgeTable[cubetype]&32)   samples[5]  = add_vertex(corner[5], corner[6]);
  if (edgeTable[cubetype]&64)   samples[6]  = add_vertex(corner[7], corner[6]);
  if (edgeTable[cubetype]&128)  samples[7]  = add_vertex(corner[4], corner[7]);
  if (edgeTable[cubetype]&256)  samples[8]  = add_vertex(corner[0], corner[4]);
  if (edgeTable[cubetype]&512)  samples[9]  = add_vertex(corner[1], corner[5]);
  if (edgeTable[cubetype]&1024) samples[10] = add_vertex(corner[2], corner[6]);
  if (edgeTable[cubetype]&2048) samples[11] = add_vertex(corner[3], corner[7]);



  // connect samples by triangles
  for (i=0; triTable[cubetype][0][i] != -1; i+=3 )
    mesh_.add_face(samples[triTable[cubetype][0][i  ]],
		   samples[triTable[cubetype][0][i+1]],
		   samples[triTable[cubetype][0][i+2]]);
}


//-----------------------------------------------------------------------------


template <class Mesh>
typename MarchingCubesT<Mesh>::VertexHandle
MarchingCubesT<Mesh>::
add_vertex(PointIdx _p0, PointIdx _p1)
{
  // find vertex if it has been computed already
  VertexHandle   vh = edge2vertex_.find(_p0, _p1);
  if (vh.is_valid())  return vh;



  // generate new vertex
  const OpenMesh::Vec3f&  p0(grid_.point(_p0));
  const OpenMesh::Vec3f&  p1(grid_.point(_p1));

  float s0 = fabs(grid_.scalar_distance(_p0));
  float s1 = fabs(grid_.scalar_distance(_p1));
  float t  = s0 / (s0+s1);

  vh = mesh_.add_vertex((1.0f-t)*p0 + t*p1);
  edge2vertex_.insert(_p0, _p1, vh);

  return vh;
}


//=============================================================================
} // namespace IsoEx
//=============================================================================
