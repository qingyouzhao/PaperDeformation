#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "Vector.hh"
#include <vector>

extern const int CONTOUR;
extern const int MOUNTAIN;
extern const int VALLEY;
// This class reads in the crease pattern file
// Then triangulates the data structure based on https://github.com/libigl/libigl/blob/master/include/igl/triangle/triangulate.h
// Then create a mesh http://www.openmesh.org/Documentation/OpenMesh-2.0-Documentation/tutorial_01.html
struct CreaseEdge
{
	Vector3f start;
	Vector3f end;
	std::vector<Vector3f> control_points;
	int segments;
	int type; // 0, 1, 2, 3
};

class CreasePatternParser {

	typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
public:
	// read in the crease pattern
	void read_crease_pattern(const std::string& filename);
	// parse generated and triangulated data to out mesh
	void crease_pattern_to_open_mesh(Mesh& out_mesh, std::vector<std::vector<Mesh::HalfedgeHandle>>& out_hehs, std::vector<int>& out_crease_types);
private:
	// triangulate based on crease_edge_data
	void triangulate(Mesh& out_mesh);
	void igl_example() const;

	// returns if the point is on the segment
	bool is_point_on_segment(const Vector3f& point, const Vector3f& start, const Vector3f& end) const;
	CreaseEdge read_line(const std::string& line);
	// A plain vector of all the things we read from lines
	std::vector<CreaseEdge> crease_edge_data;
};