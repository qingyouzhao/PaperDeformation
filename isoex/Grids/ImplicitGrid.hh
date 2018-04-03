//=============================================================================
//
//  CLASS ImplicitGrid
//
//=============================================================================


#ifndef ISOEX_IMPLICITGRID_HH
#define ISOEX_IMPLICITGRID_HH


//== INCLUDES =================================================================

#include <IsoEx/Grids/RegularGrid.hh>
#include <IsoEx/Implicits/Implicit.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <vector>

//== NAMESPACES ===============================================================

namespace IsoEx {

//== CLASS DEFINITION =========================================================

	      
/** \class ImplicitGrid ImplicitGrid.hh <IsoEx/Grids/ImplicitGrid.hh>
    
    This is the base class for all grids representing implicit
    objects, i.e. they store a reference to an implicit.  All
    inside/outside tests and distance queries will be passed on to the
    implicit.

    In addition, this grid also provides caching of inside/outside tests and
    scalar distance queries.

    \ingroup grids
*/	      
class ImplicitGrid : public RegularGrid
{
public:
   
  //------------------------------------------------------------ public methods


  /// Default constructor
  ImplicitGrid(const Implicit& _implicit,
	       const OpenMesh::Vec3f&  _origin,
	       const OpenMesh::Vec3f&  _x_axis,
	       const OpenMesh::Vec3f&  _y_axis,
	       const OpenMesh::Vec3f&  _z_axis,
	       unsigned int            _x_res,
	       unsigned int            _y_res,
	       unsigned int            _z_res) 
    : RegularGrid(_origin, _x_axis, _y_axis, _z_axis, _x_res, _y_res, _z_res),
      implicit_(_implicit) 
  {}

  /// Destructor
  virtual ~ImplicitGrid() {}



  /// \name This function calls will be passed to the implicit object.
  //@{

  /// See IsoEx::Implicit::is_inside()
  virtual bool is_inside(PointIdx _pidx) const {
    return (is_inside_cache_.empty() ? 
	    implicit_.is_inside(point(_pidx)) :
	    is_inside_cache_[_pidx]);
  }

  /// See IsoEx::Implicit::scalar_distance()
  virtual float scalar_distance(PointIdx _pidx) const {
    return (scalar_distance_cache_.empty() ? 
	    implicit_.scalar_distance(point(_pidx)) :
	    scalar_distance_cache_[_pidx]);
  }

  /// See IsoEx::Implicit::directed_distance()
  virtual bool directed_distance(const OpenMesh::Vec3f&  _p0,
				 const OpenMesh::Vec3f&  _p1,
				 OpenMesh::Vec3f&        _point,
				 OpenMesh::Vec3f&        _normal,
				 float&                  _distance) const {
    return implicit_.directed_distance(_p0, _p1, _point, _normal, _distance);
  }
  
  //@}



  /// \name Enable caching of inside/outside/distance computations
  //@{

  /// Cache results of is_inside()
  void build_is_inside_cache() const
  {
    int i, np(n_points());
    
    is_inside_cache_.clear();
    is_inside_cache_.resize(n_points());

    for (i=0; i<np; ++i)
      is_inside_cache_[i] = implicit_.is_inside(point(i));
  }

  /// Cache results of scalar_distance()
  void build_scalar_distance_cache() const
  {
    int i, np(n_points());

    scalar_distance_cache_.clear();
    scalar_distance_cache_.resize(np);

    for (i=0; i<np; ++i)
      scalar_distance_cache_[i] = implicit_.scalar_distance(point(i));
  }

  //@}


 
protected:

  const Implicit&     implicit_;

  mutable std::vector<bool>   is_inside_cache_;
  mutable std::vector<float>  scalar_distance_cache_;
};


//=============================================================================
} // namespace IsoEx
//=============================================================================
#endif // ISOEX_IMPLICITGRID_HH defined
//=============================================================================
