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
//  CLASS ImplicitSphere
//
//=============================================================================


#ifndef ISOEX_IMPLICITSPHERE_HH
#define ISOEX_IMPLICITSPHERE_HH


//== INCLUDES =================================================================

#include <IsoEx/Implicits/Implicit.hh>

//== NAMESPACES ===============================================================

namespace IsoEx {

//== CLASS DEFINITION =========================================================

	      
/** \class ImplicitSphere ImplicitSphere.hh <IsoEx/Implicits/ImplicitSphere.hh>
    This class implements a very simple implicit object: a sphere given its
    center and its radius.
    \see IsoEx::Implicit
    \ingroup implicits
*/	      
class ImplicitSphere : public Implicit
{
public:
   
  /// \name Constructor & destructor
  //@{

  /// Constructor: given sphere center and radius
  ImplicitSphere(const OpenMesh::Vec3f& _center, float _radius)
    : center_(_center), 
      radius_(_radius), 
      sqr_radius_(_radius*_radius)
  {}

  /// Empty destructor
  ~ImplicitSphere() {}

  //@}



  /// \name Abstract interface of implicit objects, see also IsoEx::Implicit.
  //@{

  bool is_inside(const OpenMesh::Vec3f& _point) const 
  {
    return (center_ - _point).sqrnorm() <= sqr_radius_;
  }

  float scalar_distance(const OpenMesh::Vec3f& _point) const 
  {
    return (center_ - _point).norm() - radius_;
  }

  bool directed_distance(const OpenMesh::Vec3f&  _p0,
			 const OpenMesh::Vec3f&  _p1,
			 OpenMesh::Vec3f&        _point,
			 OpenMesh::Vec3f&        _normal,
			 float&                  _distance) const
  {
    OpenMesh::Vec3f orig(_p0), dir(_p1-_p0);

    double a = dir.sqrnorm();
    double b = 2.0*(dir | (orig - center_));
    double c = (orig - center_).sqrnorm() - radius_*radius_;
    double d = b*b - 4.0*a*c;

    if (d >= 0)
    {
      d = sqrt(d);

      double t1 = (-b-d) / (2.0*a);
      double t2 = (-b+d) / (2.0*a);
      double t  = 1.00001;
      if (t1 >= 0.0 && t1 < t) t = t1;
      if (t2 >= 0.0 && t2 < t) t = t2;

      if (t != 1.00001)
      {
	_point    = orig + dir*t;
	_normal   = (_point - center_) / radius_;
	_distance = ((dir | _normal) < 0.0) ? dir.norm()*t : -dir.norm()*t;
	return true;
      }
    }

    return false;
  }
  
  //@}


private:

  OpenMesh::Vec3f  center_;
  float            radius_;
  float            sqr_radius_;
};


//=============================================================================
} // namespace IsoEx
//=============================================================================
#endif // ISOEX_IMPLICITSPHERE_HH defined
//=============================================================================

