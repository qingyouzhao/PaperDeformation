//=============================================================================
//                                                
//   Code framework for the lecture
//
//   "Surface Representation and Geometric Modeling"
//
//   Mark Pauly, Mario Botsch, Balint Miklos, and Hao Li
//
//   Copyright (C) 2007 by  Applied Geometry Group and 
//							Computer Graphics Laboratory, ETH Zurich
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
#include <igl/triangle/triangulate.h>
// Input polygon
Eigen::MatrixXd V;
Eigen::MatrixXi E;
Eigen::MatrixXd H;

// Triangulated interior
Eigen::MatrixXd V2;
Eigen::MatrixXi F2;

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;

  // Create the boundary of a square
  V.resize(8,2);
  E.resize(8,2);
  H.resize(1,2);

  V << -1,-1, 1,-1, 1,1, -1, 1,
       -2,-2, 2,-2, 2,2, -2, 2;

  E << 0,1, 1,2, 2,3, 3,0,
       4,5, 5,6, 6,7, 7,4;

  H << 0,0;

  // Triangulate the interior
  igl::triangle::triangulate(V,E,H,"a0.005q",V2,F2);

  // Plot the generated mesh
}