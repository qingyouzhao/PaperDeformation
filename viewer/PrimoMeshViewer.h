#pragma once

#include "MeshViewer.hh"
#include "nanort.h"
#include <vector>

enum class EPrismExtrudeMode {
	VERT_NORMAL,
	FACE_NORMAL,
	CUSTOM
};

struct PrismProperty {
	Vec3f FromVertPrismDir;
	float FromVertPrismSize;
	Vec3f ToVertPrimsDir;
	float ToVertPrismSize;

	// A simple illustration of how the prism is stored
	/*
	10---------11 ^       10------------11   ^
	| f_ij     |  |       |  f_ji       |    |
	f-he_ij-->to  |       to<--he_ji---from  |
	00---------01 normal  00------------01   normal
	*/
	Vec3f calc_f_uv(int u, int v, bool is_i_j) const
	{
		Vec3f result;
		// We only care uv edges
		assert(u == 0 || u == 1);
		assert(v == 0 || v == 1);
		if (is_i_j)
		{
			result = v == 0 ? (FromVertPrismDir * FromVertPrismSize) : (ToVertPrimsDir * ToVertPrismSize);
			result *= (u == 0 ? -1 : 1);
		}
		else
		{
			result = v == 0 ? (ToVertPrimsDir * ToVertPrismSize) : (FromVertPrismDir * FromVertPrismSize);
			result *= (u == 0 ? -1 : 1);
		}
		return result;
	}
};

class PrimoMeshViewer :public MeshViewer
{
public:
	/// default constructor
	PrimoMeshViewer(const char* _title, int _width, int _height);

	// destructor
	~PrimoMeshViewer();

	/// open mesh
	virtual bool open_mesh(const char* _filename);

protected:
	// Draw with Open GL utilities, update color coding stuff here.
	virtual void draw(const std::string& _draw_mode);

	// GLut overload, inputs
	virtual void keyboard(int key, int x, int y);
	virtual void motion(int x, int y);
	virtual void mouse(int button, int state, int x, int y);

	// Setup prisms for the meshes
	// default to face normals, this makes the prism very flat
	virtual void setup_prisms(EPrismExtrudeMode PrismExtrudeMode = EPrismExtrudeMode::FACE_NORMAL);
	
	// Move this vertex to the targeted handles
	virtual void manipulate(Mesh::VertexHandle vh_, Mesh::Point target_location);

	// Locally optimize for one prism
	virtual void local_optimize(int iterations);

	// Locally optimize for one prism faces 
	virtual void local_optimize_face(Mesh::FaceHandle _fh);

	// globally solve for all prism faces
	virtual void global_optimize_all_faces(int iterations);

	float calc_face_area(Mesh::FaceHandle _fh) const;

	/// get average vertex distance
    float get_average_vertex_distance(const Mesh& mesh) const;
	// For primo hierarchical, the solution should be based on a, say randomly sampled points, 
	// then find the shortest path along the edges

	enum class EViewMode{VIEW, MOVE} viewMode_;
	enum class ESelectMode{STATIC, DYNAMIC, NONE} selectMode_;
private:
	// Normalized direction
	OpenMesh::HPropHandleT<PrismProperty>  P_PrismProperty;
	// press p to visualize prisms
	bool drawPrisms_;
	// colors of faces of prisms
	GLfloat staticFacesColor_[3];
	GLfloat dynamicFacesColor_[3];
	GLfloat optimizedFacesColor_[3];
	// prism' height (homogeneous: all prisms' height are same now)
	float prismHeight_;
	float averageVertexDisance_;
	// 3 types of face handles
	// only optimize the optimizedFaces
	std::vector<OpenMesh::FaceHandle> optimizedFaceHandles_;
	// static faces(prisms) as hard constraints
	std::vector<OpenMesh::FaceHandle> staticFaceHandles_;
	// dynamic faces(prisms) could be moved by UI, also hard constraints when doing optimization
	std::vector<OpenMesh::FaceHandle> dynamicFaceHandles_;

	// used for ray-casting, from prim_id to faceHandle
	std::vector<OpenMesh::FaceHandle> allFaceHandles_;
	nanort::BVHAccel<float> allFaces_BVH_;
	void update_allFace_handles();
	void build_allFace_BVH();

};