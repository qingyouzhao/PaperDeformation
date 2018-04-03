//=============================================================================

#ifndef RBF_HH
#define RBF_HH

//=============================================================================

#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <vector>
#include <gmm.h>

//=============================================================================

class ImplicitRBF
{
public:

    typedef OpenMesh::Vec3f            Vec3f;
    typedef OpenMesh::Vec3d            Vec3d;
    typedef gmm::dense_matrix<double>  gmmMatrix;
    typedef std::vector<double>        gmmVector;

    
    // fit RBF to given constraints
    ImplicitRBF( 
        const std::vector<Vec3f>& _points, 
        const std::vector<Vec3f>& _normals );

    // evaluate RBF at position _p
    double operator()(const Vec3f& _p) const;

private:

    // evaluate basis function of RBF
    static double kernel(const Vec3d& _c, const Vec3d& _x)
    {
        double r = (_x-_c).norm();
        return r*r*r;
    }

	// evaluate basis function of RBF
	static double kernel(const Vec3f& _c, const Vec3f& _x)
	{
		double r = (_x - _c).norm();
		return r*r*r;
	}

    // solve linear system _A * _x = _b
    void solve_linear_system( 
        gmmMatrix& _A, 
        gmmVector& _b, 
        gmmVector& _x );
  
private:

    std::vector<Vec3d>   centers_;
    std::vector<double>  weights_;
};

//=============================================================================
#endif // RBF_HH defined
//=============================================================================
