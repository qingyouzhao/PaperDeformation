#pragma once

#include "MeshViewer.hh"
#include "nanort.h"
#include "Transformation.hh"
#include <vector>
#include <unordered_map>
enum class EPrismExtrudeMode {
	VERT_NORMAL,
	FACE_NORMAL,
	CUSTOM
};

/// This struct store the prism data structure and provides basic functionalities for retrieval and claculation
struct PrismProperty {
	Vec3f FromVertPrismDir_DEPRECATED;
	float FromVertPrismSize_DEPRECATED;
	Vec3f ToVertPrimsDir_DEPRECATED;
	float ToVertPrismSize_DEPRECATED;

	Vec3f FromVertPrismUp;
	Vec3f FromVertPrismDown;
	Vec3f ToVertPrismUp;
	Vec3f ToVertPrismDown;

	Vec3f FromVertNormal;
	Vec3f ToVertNormal;

	// A simple illustration of how the prism is stored
	/*
	10---------11 ^       10------------11   ^
	| f_ij     |  |       |  f_ji       |    |
	f-he_ij-->to  |       to<--he_ji---from  |
	00---------01 normal  00------------01   normal
	*/

	// A simple illustration of how the prism is stored
	/*
	From up		
	01---------11 ^       01------------11   ^
	| f_ij     |  |       |  f_ji       |    |
	f-he_ij-->to  |       to<--he_ji---from  |
	00---------10 normal  00------------10   normal
	*/

	Vec3f f_uv(int u, int v, bool is_i_j)
	{
		Vec3f result;
		// We only care uv edges
		assert(u == 0 || u == 1);
		assert(v == 0 || v == 1);
		if (is_i_j)
		{
			result = (v == 1) ? ( u == 0 ? FromVertPrismUp : ToVertPrismUp)
							  : ( u == 0 ? FromVertPrismDown : ToVertPrismDown);
		}
		else
		{
			result = (v == 1) ? (u == 0 ? ToVertPrismUp: FromVertPrismUp)
							  : (u == 0 ? ToVertPrismDown: FromVertPrismDown);
		}
		return result;
	}

	Vec3f TargetPosFrom()
	{
		return (FromVertPrismUp + FromVertPrismDown) * 0.5f;
	}
	Vec3f TargetPosTo()
	{	
		return (ToVertPrismDown + ToVertPrismUp) * 0.5f;

	}

	void TransformPrism(const Transformation& Transform)
	{
		FromVertPrismUp = Transform * (FromVertPrismUp);
		FromVertPrismDown = Transform * (FromVertPrismDown);
		ToVertPrismUp = Transform * (ToVertPrismUp);
		ToVertPrismDown = Transform * (ToVertPrismDown);
		Transform.transformVector(FromVertNormal);
		Transform.transformVector(ToVertNormal);
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
	virtual void setup_prisms(std::vector<OpenMesh::FaceHandle> &face_handles, 
								EPrismExtrudeMode PrismExtrudeMode = EPrismExtrudeMode::FACE_NORMAL);
	
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
	enum class ESelectMode{STATIC, DYNAMIC, OPTIMIZED, NONE} selectMode_;
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
	int local_optimize_iterations_;
	// 3 types of face handles
	// only optimize the optimizedFaces
	std::vector<OpenMesh::FaceHandle> optimizedFaceHandles_;
	std::vector<unsigned int> optimizedVertexIndices_;
	// static faces(prisms) as hard constraints
	std::vector<OpenMesh::FaceHandle> staticFaceHandles_;
	std::vector<unsigned int> staticVertexIndices_;
	// dynamic faces(prisms) could be moved by UI, also hard constraints when doing optimization
	std::vector<OpenMesh::FaceHandle> dynamicFaceHandles_;
	std::vector<unsigned int> dynamicVertexIndices_;

	// used for ray-casting, from prim_id to faceHandle
	std::vector<OpenMesh::FaceHandle> allFaceHandles_;
	nanort::BVHAccel<float> allFaces_BVH_;
	// face_handles is cleared and filled with all face handles in mesh_
	void get_allFace_handles(std::vector<OpenMesh::FaceHandle> &face_handles);
	void build_allFace_BVH();
	
	// 1. ray cast allfaces
	// 2. add to STATIC/DYNAMIC faces based on selectMode_
	// 3. update face STATIC/DYNAMIC indices for drawing
	void raycast_faces(int x, int y);
	void update_1typeface_indices(const std::vector<OpenMesh::FaceHandle> &face_handles, 
										std::vector<unsigned int> &indices);
	
	// delete a face_handle(fh) from face_handles, where fh.idx() == faceId
	// O(n), need optimize if this is raycast bottleneck
	void delete_faceHandle(unsigned int faceId, std::vector<OpenMesh::FaceHandle> &face_handles);
	// each face could only have one type of STATIC/DYNAMI/NONE
	std::unordered_map<unsigned int, ESelectMode> faceIdx_to_selType_;
	// draw prisms for all faces in array(vector)
	void draw_prisms(const std::vector<OpenMesh::FaceHandle> &face_handles) const;
	// transformation for all dynamic faces. changed in ESelectMode::NONE(press 3)
	// 1 transformation for all dynamic faces, just for simplicity
	Transformation dynamic_faces_transform_;
	
	// it is avg of all dynamic faces' normals. 
	OpenMesh::Vec3f dynamic_rotation_axis_;
	OpenMesh::Vec3f dynamic_rotation_centroid_;
	void update_dynamic_rotation_axis_and_centroid();
	void rotate_faces_and_prisms_around_centroid(const OpenMesh::Vec3f &rotation_centroid, const OpenMesh::Vec3f &rotation_axis
										, float angle, std::vector<OpenMesh::FaceHandle> &face_handles);
	void translate_faces_and_prisms_along_axis(const OpenMesh::Vec3f &axis, float dist, std::vector<OpenMesh::FaceHandle> &face_handles);

};