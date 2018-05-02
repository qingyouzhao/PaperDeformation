#pragma once
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

// This class handles the triangulation of a mesh
class Triangulation {
	typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;

public:
	void igl_example() const;

	// The ultimate goal is we take an open mesh mesh, triangulate that.
	
	void triangulate_crease_pattern(const Triangulation::Mesh& mesh_);

	Mesh new_mesh;
private:
};