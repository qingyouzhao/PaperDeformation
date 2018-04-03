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
//  CLASS Implicit
//
//=============================================================================


#ifndef ISOEX_IMPLICIT_HH
#define ISOEX_IMPLICIT_HH


//== INCLUDES =================================================================

#include <OpenMesh/Core/Geometry/VectorT.hh>

//== NAMESPACES ===============================================================

namespace IsoEx {

//== CLASS DEFINITION =========================================================

/** \class Implicit Implicit.hh <IsoEx/Implicits/Implicit.hh>
    
    This is the abstract base class for all objects representing implicit
    function. The three abstract virtual functions have to be overridden
    by derived classes.

    \ingroup implicits
*/	      
class Implicit
{
public:
   
  /// \name Constructor & destructor
  //@{

  /// constructor
  Implicit() {}
  /// destructor
  virtual ~Implicit() {}

  //@}



  /// \name Abstract interface of implicit objects
  //@{

  /** Is the point \b _point inside or outside w.r.t. the implicit
      function? The result should be the same as scalar_distance(_point)
      < 0, but it can be implemented more efficiently.
  */
  virtual bool is_inside(const OpenMesh::Vec3f& _point) const = 0;


  /** Returns the <em>scalar distance</em> value of \b _point. Points
      inside the object should have negative distances, points outside
      positive distance.
  */
  virtual float scalar_distance(const OpenMesh::Vec3f& _point) const = 0;


  /** This method returns the <em>directed distance</em> of \b _p0 in
      the direction <b>_p1-_p0</b>. The resulting intersection point
      (casting a ray from _p0 in direction (_p1-_p0)) is stored in \b
      _point, the corresponding normal vector at this point is stored
      in \b _normal, the distance value is stored in _distance (again
      negative distance means \b _p0 is inside). Since the ray
      intersection may fail (because of precision problems) it is
      returned whether an intersection was found or not.
   */
  virtual bool directed_distance(const OpenMesh::Vec3f&  _p0,
				 const OpenMesh::Vec3f&  _p1,
				 OpenMesh::Vec3f&        _point,
				 OpenMesh::Vec3f&        _normal,
				 float&                  _distance) const = 0;
  //@}
};


//=============================================================================
} // namespace IsoEx
//=============================================================================
#endif // ISOEX_IMPLICITSPHERE_HH defined
//=============================================================================

