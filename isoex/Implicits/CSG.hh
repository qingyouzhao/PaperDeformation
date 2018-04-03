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
//  CLASS CSGUnion, CSGIntersection, CSGDifference
//
//=============================================================================


#ifndef ISOEX_CSG_HH
#define ISOEX_CSG_HH


//== INCLUDES =================================================================

#include <IsoEx/Implicits/Implicit.hh>

//== NAMESPACES ===============================================================

namespace IsoEx {
namespace CSG {

//== CLASS DEFINITION =========================================================


/** \class Union CSG.hh <IsoEx/Implicits/CSG.hh>
    This class builds a CSG union of two other implicits.
    \see IsoEx::Implicit
    \ingroup implicits
*/	      
class Union : public Implicit
{
public:
   
  /// Constructor: given the two implicit objects to unite
  Union(const Implicit& _implicit1, const Implicit& _implicit2)
    : implicit1_(_implicit1), implicit2_(_implicit2)
  {}


  /// \name Mandatory interface of implicit objects, see also IsoEx::Implicit.
  //@{

  bool   is_inside(const OpenMesh::Vec3f& _p) const {
    return implicit1_.is_inside(_p) || implicit2_.is_inside(_p);
  }

  float  scalar_distance(const OpenMesh::Vec3f& _p) const {
    return std::min(implicit1_.scalar_distance(_p), 
		    implicit2_.scalar_distance(_p));
  }

  bool   directed_distance(const OpenMesh::Vec3f&  _p0,
			   const OpenMesh::Vec3f&  _p1,
			   OpenMesh::Vec3f&        _point,
			   OpenMesh::Vec3f&        _normal,
			   float&                  _distance) const
  {
    bool             ok1, ok2;
    OpenMesh::Vec3f  p1, n1, p2, n2;
    float            d1, d2;

    ok1 = implicit1_.directed_distance(_p0, _p1, p1, n1, d1);
    ok2 = implicit2_.directed_distance(_p0, _p1, p2, n2, d2);

    if (ok1 && ok2) { if (d1 < d2) ok2 = false; else ok1 = false; }
    
    if      (ok1)  { _point = p1; _normal = n1; _distance = d1; return true; }
    else if (ok2)  { _point = p2; _normal = n2; _distance = d2; return true; }
    
    return false;
  }

  //@}

private:

  const Implicit& implicit1_;
  const Implicit& implicit2_;
};



//=============================================================================



/** \class Intersection CSG.hh <IsoEx/Implicits/CSG.hh>
    This class builds a CSG intersection of two other implicits.
    \see IsoEx::Implicit
    \ingroup implicits
*/	      
class Intersection : public Implicit
{
public:
   
  /// Constructor: given the two implicit objects to intersect
  Intersection(const Implicit& _implicit1, const Implicit& _implicit2)
    : implicit1_(_implicit1), implicit2_(_implicit2)
  {}


  /// \name Mandatory interface of implicit objects, see also IsoEx::Implicit.
  //@{

  bool   is_inside(const OpenMesh::Vec3f& _p) const {
    return (implicit1_.is_inside(_p) && implicit2_.is_inside(_p));
  }

  float  scalar_distance(const OpenMesh::Vec3f& _p) const {
    return std::max(implicit1_.scalar_distance(_p), 
		    implicit2_.scalar_distance(_p));
  }

  bool   directed_distance(const OpenMesh::Vec3f&  _p0,
			   const OpenMesh::Vec3f&  _p1,
			   OpenMesh::Vec3f&        _point,
			   OpenMesh::Vec3f&        _normal,
			   float&                  _distance) const
  {
    bool             ok1, ok2;
    OpenMesh::Vec3f  p1, n1, p2, n2;
    float            d1, d2;

    ok1 = implicit1_.directed_distance(_p0, _p1, p1, n1, d1);
    ok2 = implicit2_.directed_distance(_p0, _p1, p2, n2, d2);

    if (ok1 && ok2) { if (d1 > d2) ok2 = false; else ok1 = false; }
    
    if      (ok1)  { _point = p1; _normal = n1; _distance = d1; return true; }
    else if (ok2)  { _point = p2; _normal = n2; _distance = d2; return true; }
    
    return false;
  }
  
  //@}


private:

  const Implicit& implicit1_;
  const Implicit& implicit2_;
};



//=============================================================================



/** \class Difference CSG.hh <IsoEx/Implicits/CSG.hh>
    This class builds a CSG difference of two other implicits.
    \see IsoEx::Implicit
    \ingroup implicits
*/	      
class Difference : public Implicit
{
public:
   
  /** Constructor: given the two implicit objects, builds \b _implicit1
      minus \b _implicit2.
  */
  Difference(const Implicit& _implicit1, const Implicit& _implicit2)
    : implicit1_(_implicit1), implicit2_(_implicit2)
  {}


  /// \name Mandatory interface of implicit objects, see also IsoEx::Implicit.
  //@{

  bool   is_inside(const OpenMesh::Vec3f& _p) const {
    return implicit1_.is_inside(_p) && !implicit2_.is_inside(_p);
  }

  float  scalar_distance(const OpenMesh::Vec3f& _p) const {
    return std::max(implicit1_.scalar_distance(_p), 
		    -implicit2_.scalar_distance(_p));
  }

  bool   directed_distance(const OpenMesh::Vec3f&  _p0,
			   const OpenMesh::Vec3f&  _p1,
			   OpenMesh::Vec3f&        _point,
			   OpenMesh::Vec3f&        _normal,
			   float&                  _distance) const
  {
    bool             ok1, ok2;
    OpenMesh::Vec3f  p1, n1, p2, n2;
    float            d1, d2;

    ok1 = implicit1_.directed_distance(_p0, _p1, p1, n1, d1);
    ok2 = implicit2_.directed_distance(_p0, _p1, p2, n2, d2);
    d2 = -d2;

    if (ok1 && ok2) { if (d1 > d2) ok2 = false; else ok1 = false; }
    
    if      (ok1)  { _point = p1; _normal = n1; _distance = d1; return true; }
    else if (ok2)  { _point = p2; _normal = n2; _distance = d2; return true; }
    
    return false;
  }
  
  //@}

private:

  const Implicit& implicit1_;
  const Implicit& implicit2_;
};



//=============================================================================
} // namespace CSG
} // namespace IsoEx
//=============================================================================
#endif // ISOEX_CSG_HH defined
//=============================================================================

