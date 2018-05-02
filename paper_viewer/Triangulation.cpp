
#include "libigl/include/igl/delaunay_triangulation.h"
#include "libigl/include/igl/triangle/triangulate.h"
#include "Triangulation.h"
#include <vector>
#include <iostream>

void Triangulation::igl_example() const
{
	using namespace Eigen;
	using namespace std;
	// Input polygon
	Eigen::MatrixXd V;
	Eigen::MatrixXi E;
	Eigen::MatrixXd H;

	// Triangulated interior
	Eigen::MatrixXd V2;
	Eigen::MatrixXi F2;
	// Create the boundary of a square
	V.resize(8, 2);
	E.resize(8, 2);
	H.resize(1, 2);

	V << -1, -1, 1, -1, 1, 1, -1, 1,
		-2, -2, 2, -2, 2, 2, -2, 2;

	E << 0, 1, 1, 2, 2, 3, 3, 0,
		4, 5, 5, 6, 6, 7, 7, 4;

	H << 0, 0;

	// Triangulate the interior
	igl::triangle::triangulate(V, E, H, "a0.005q", V2, F2);
}

void Triangulation::triangulate_crease_pattern(const Triangulation::Mesh& mesh_)
{
	// just let it run it self first
	igl_example();
	// we should be able to get all the mesh points
	Eigen::MatrixXd V;
	Eigen::MatrixXi E;
	Eigen::MatrixXd H;

	// Triangulated interior
	Eigen::MatrixXd V2;
	Eigen::MatrixXi F2;

	int nv = mesh_.n_vertices();
	int ne = mesh_.n_edges();

	V.resize(nv, 2);
	E.resize(ne, 2);
	int v_c = 0;
	for (Mesh::VertexIter v_it = mesh_.vertices_begin(); v_it!= mesh_.vertices_end(); v_it++)
	{
		Mesh::Point p = mesh_.point(v_it);
		std::cout << "processed v index " << (*v_it) << " v_c = " << v_c << " at " << p << std::endl;
		// store xz of v
		V(v_c, 0) = p[0];
		V(v_c, 1) = p[2];
		v_c++;
	}

	int e_c = 0;
	for (Mesh::EdgeIter e_it = mesh_.edges_begin(); e_it != mesh_.edges_end(); e_it++)
	{
		Mesh::Edge e = mesh_.edge(e_it);
		Mesh::HalfedgeHandle heh = mesh_.halfedge_handle(*e_it, 0);
		Mesh::VertexHandle from_vh = mesh_.from_vertex_handle(heh);
		Mesh::VertexHandle to_vh = mesh_.to_vertex_handle(heh);
		std::cout << "processed e index " << e_c << " v_idx  = " << from_vh.idx()  << "," << to_vh.idx() << std::endl;
		std::cout << "processed e from " << mesh_.point(from_vh) << " to " << mesh_.point(to_vh) << std::endl;

		E(e_c, 0) = from_vh.idx();
		E(e_c, 1) = to_vh.idx();
		e_c++;
	}

	// Todo what does the config look like
	// https://www.cs.cmu.edu/~quake/triangle.html

	igl::triangle::triangulate(V, E, H, "a0.05q", V2, F2);


	
}

// Project.
// Read cpx

// Create points
// IGL triangulate
// IGL -> obj

// Paper viewer -> read obj
// Get points
// Half edge handles
// Triangulation

// 当你用这个function的时候
// 我哦建议你说在。hh不要include gil的东西
// 当我在promo viewer的时候
// 在那个里头不能有igl的东西
// 然后呢，得到mesh view里的mesh

// get point 可以留在view乐然