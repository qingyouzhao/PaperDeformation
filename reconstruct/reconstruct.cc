//=============================================================================

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Tools/Utils/Timer.hh>

#include <IsoEx/Grids/ScalarGridT.hh>
#include <IsoEx/Extractors/MarchingCubesT.hh>

#include <iostream>
#include <fstream>
#include <vector>

#include "ImplicitRBF.hh"
#include "ImplicitHoppe.hh"

//=============================================================================

// set to 1 for RBF fitting
#define USE_RBF    1

// else set this to 1 for HOPPE
#define USE_HOPPE  1

// resolution of Marching Cubes grid
#define MC_RESOLUTION  50

//=============================================================================

typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
typedef Mesh::Point                       Point;
typedef Mesh::Scalar                      Scalar;
typedef OpenMesh::Vec3d                   Vec3d;


//=============================================================================

int main(int argc, char **argv)
{
    // parse command line
    if (argc<2)
    {
        std::cerr << "Usage:\n"
            << argv[0] 
            << "  <input-points>  <output-mesh>\n";
        exit(1);
    }

    // load sample points with normals
    std::cout << "Load points\n" << std::flush;

    std::ifstream ifs(argv[1]);
    if (!ifs) 
    {
        std::cerr << "Cannot open file\n";
        exit(1);
    }

    std::vector<Point>   points, normals;
    Point                p, n;

    while (ifs && !ifs.eof())
    {
        ifs >> p[0] >> p[1] >> p[2];
        ifs >> n[0] >> n[1] >> n[2];
        points.push_back(p);
        normals.push_back(n);
    }
    std::cout << points.size() << " sample points\n";
    ifs.close();

    // fit RBF to constraints
#if USE_RBF
    std::cout << "Fit RBF\n" << std::flush;
    ImplicitRBF   implicit( points, normals );
#elif USE_HOPPE
    std::cout << "Use Hoppe's method\n" << std::flush;
    ImplicitHoppe implicit( points, normals );
#else
#error(You have to set either USE_RBF or USE_HOPPE to 1)
#endif

    // compute bounding cube for Marching Cubes grid
    std::cout << "Bounding Box\n" << std::flush;
    Point bb_min( points[0]), bb_max( points[0]);

    for (unsigned int i=1; i<points.size(); ++i)
    {
        bb_min.minimize( points[i] );
        bb_max.maximize( points[i] );
    }

    Point  bb_center = (bb_max+bb_min)*0.5f;
    Scalar bb_size   = (bb_max-bb_min).max();
    bb_min = bb_center - 0.6f * Point(bb_size, bb_size, bb_size);
    bb_max = bb_center + 0.6f * Point(bb_size, bb_size, bb_size);

    // setup Marching Cubes grid by sampling RBF
    std::cout << "Setup grid\n" << std::flush;
    int res = MC_RESOLUTION;
    IsoEx::ScalarGridT<Scalar>  grid(bb_min,
        Point(bb_max[0]-bb_min[0], 0, 0),
        Point(0, bb_max[1]-bb_min[1], 0),
        Point(0, 0, bb_max[2]-bb_min[2]),
        res, res, res);

    for (unsigned int x=0; x<res; ++x)
        for (unsigned int y=0; y<res; ++y)
            for (unsigned int z=0; z<res; ++z)
                grid.value(x,y,z) = implicit( grid.point(x,y,z) );

    // isosurface extraction by Marching Cubes
    std::cout << "Marching Cubes\n" << std::flush;
    Mesh mesh;
    marching_cubes(grid, mesh); 

    // write mesh
     std::cout << "Write mesh\n" << std::flush;
    if (!OpenMesh::IO::write_mesh(mesh, argv[2]))
    {
        std::cerr << "Cannot write mesh\n";
        exit(1);
    }

    std::cout << "Done\n" << std::flush;
    return 0;
}


//=============================================================================
