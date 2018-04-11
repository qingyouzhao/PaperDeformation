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
#include "ValenceViewer.hh"
#include "PrimoMeshViewer.h"



int main(int argc, char **argv)
{
  glutInit(&argc, argv);
  printf("Usage:\na: Toggle prism visualization\n+: Add prism height\n-: Minus prism height\no: switch optimization method (default: local)\n\n");
  printf("Legend:\nOrange: dynamic faces\nBlue: optimizable faces\nDark Green: static faces\n-------"
          "---------------------------------------------------\n\n");
  printf("Logging:\n");

  // ValenceViewer window("Valence Viewer", 512, 512);
  PrimoMeshViewer window("Primo Demo", 1024, 1024);

  if (argc>1)
    window.open_mesh(argv[1]);

  glutMainLoop();
}
