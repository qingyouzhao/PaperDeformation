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
//=============================================================================
//
//  CLASS ValenceViewer - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "ValenceViewer.hh"
#include <vector>
#include <float.h>



//== IMPLEMENTATION ========================================================== 


ValenceViewer::
ValenceViewer(const char* _title, int _width, int _height)
  : MeshViewer(_title, _width, _height)
{ 
  mesh_.request_vertex_colors();

  mesh_.add_property(vvalence_);

  add_draw_mode("Vertex Valences");

}


//-----------------------------------------------------------------------------


ValenceViewer::
~ValenceViewer()
{
}

//-----------------------------------------------------------------------------

bool
ValenceViewer::
open_mesh(const char* _filename)
{
  // load mesh
  if (MeshViewer::open_mesh(_filename))
  {
    // compute valence stuff
    calc_valences();
    color_coding();

    glutPostRedisplay();
    return true;
  }
  return false;
}


//-----------------------------------------------------------------------------


void 
ValenceViewer::
calc_valences()
{
  Mesh::VertexIter        v_it, v_end(mesh_.vertices_end());

  for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it)
  {
    valence(v_it) = mesh_.valence(v_it);
  }
}


//-----------------------------------------------------------------------------


void 
ValenceViewer::
color_coding()
{

  Mesh::VertexIter  v_it, v_end(mesh_.vertices_end());
  Mesh::Scalar      valen, min_valence(FLT_MAX), max_valence(-FLT_MAX);
  Mesh::Color       col;


  // put all valence values into one array
  std::vector<Mesh::Scalar> valence_values;
  valence_values.reserve(mesh_.n_vertices());
  for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it)
    valence_values.push_back(valence(v_it));

  // ignoring the extreme 1%
  unsigned int n = valence_values.size()-1;
  unsigned int i = valence_values.size() / 100.0;
  std::sort(valence_values.begin(), valence_values.end());
  min_valence = valence_values[i];
  max_valence = valence_values[n-1-i];
  std::cout << "Blue color for valences: " << valence_values[0] << " - " << min_valence << std::endl;
  std::cout << "Red color for valence: " << max_valence << " - " << valence_values[n-1] << std::endl;


  // define uniform color intervalls [v0,v1,v2,v3,v4]
  Mesh::Scalar v0, v1, v2, v3, v4;
  v0 = min_valence + 0.0/4.0 * (max_valence - min_valence);
  v1 = min_valence + 1.0/4.0 * (max_valence - min_valence);
  v2 = min_valence + 2.0/4.0 * (max_valence - min_valence);
  v3 = min_valence + 3.0/4.0 * (max_valence - min_valence);
  v4 = min_valence + 4.0/4.0 * (max_valence - min_valence);



  // map valences to colors
  for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it)
  {
    valen = valence(v_it);
    col = Mesh::Color(255,255,255);
    
    unsigned char u;

    if (valen < v0)
    {
      col = Mesh::Color(0, 0, 255);
    }
    else if (valen > v4) 
    {
      col = Mesh::Color(255, 0, 0);
    }

    else if (valen <= v2) 
    {
      if (valen <= v1) // [v0, v1]
      {
	u = (unsigned char) (255.0 * (valen - v0) / (v1 - v0));
	col = Mesh::Color(0, u, 255);
      }      
      else // ]v1, v2]
      {
	u = (unsigned char) (255.0 * (valen - v1) / (v2 - v1));
	col = Mesh::Color(0, 255, 255-u);
      }
    }
    else 
    {
      if (valen <= v3) // ]v2, v3]
      {
	u = (unsigned char) (255.0 * (valen - v2) / (v3 - v2));
	col = Mesh::Color(u, 255, 0);
      }
      else // ]v3, v4]
      {
	u = (unsigned char) (255.0 * (valen - v3) / (v4 - v3));
	col = Mesh::Color(255, 255-u, 0);
      }
    }

    mesh_.set_color(v_it, col);
  }
}


//-----------------------------------------------------------------------------


void 
ValenceViewer::
draw(const std::string& _draw_mode)
{

  if (indices_.empty())
  {
    MeshViewer::draw(_draw_mode);
    return;
  }

  if (_draw_mode == "Vertex Valences")
  {

	  glDisable(GL_LIGHTING);
	  glShadeModel(GL_SMOOTH);
	  glEnableClientState(GL_VERTEX_ARRAY);
	  glEnableClientState(GL_NORMAL_ARRAY);
	  glEnableClientState(GL_COLOR_ARRAY);
	  GL::glVertexPointer(mesh_.points());
	  GL::glNormalPointer(mesh_.vertex_normals());
	  GL::glColorPointer(mesh_.vertex_colors());
	  glDepthRange(0.01, 1.0);
	  glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	  glDisableClientState(GL_VERTEX_ARRAY);
	  glDisableClientState(GL_NORMAL_ARRAY);
	  glDisableClientState(GL_COLOR_ARRAY);
	  glColor3f(0.1, 0.1, 0.1);
	  glEnableClientState(GL_VERTEX_ARRAY);
	  GL::glVertexPointer(mesh_.points());
	  glDrawBuffer(GL_BACK);
	  glDepthRange(0.0, 1.0);
	  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	  glDepthFunc(GL_LEQUAL);
	  glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	  glDisableClientState(GL_VERTEX_ARRAY);
	  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	  glDepthFunc(GL_LESS);
  }

  else MeshViewer::draw(_draw_mode);
}


//=============================================================================
