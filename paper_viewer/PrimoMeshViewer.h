#pragma once

#include "MeshViewer.hh"
#include "nanort.h"
#include "Transformation.hh"
#include <thread>
#include <vector>
#include <unordered_map>
#include <list>


enum class EPrismExtrudeMode {
	VERT_NORMAL,
	FACE_NORMAL,
	CUSTOM
};

extern const std::string test_crease_file;

// A wrapper for GL color functionality
struct LinearColor
{
	GLfloat rgba_[4];
	LinearColor() { rgba_[0] = 0; }
	LinearColor(float r, float g, float b, float a = 1.0f) {
		rgba_[0] = r; rgba_[1] = g; rgba_[2] = b; rgba_[3] = a;
	}

	float& r() { return rgba_[0]; }
	float& g() { return rgba_[1]; }
	float& b() { return rgba_[2]; }
	float& a() { return rgba_[3]; }
	GLfloat& operator[](uint i) { return i < 4 ? rgba_[i] : rgba_[0]; }
	float v() {
		return (rgba_[0] + rgba_[1] + rgba_[2]) / 3.0f;
	}
	LinearColor operator*(double c) const {
		LinearColor nc;
		nc[0] = this->rgba_[0] * c;
		nc[1] = this->rgba_[1] * c;
		nc[2] = this->rgba_[2] * c;
		nc[3] = this->rgba_[3] * c;
		return *this;
	}

	static const LinearColor RED;
	static const LinearColor BLUE;
	static const LinearColor GREEN;
	static const LinearColor BLACK;
	static const LinearColor GREY;
	static const LinearColor WHITE;
	static const LinearColor YELLOW;
	static const LinearColor CYAN;
	static const LinearColor MAGENTA;
	static const LinearColor ORANGE;
	static const LinearColor PURPLE;
	static const LinearColor TURQUOISE;
	static const LinearColor SILVER;
	static const LinearColor EMERALD;
};


struct DebugLine
{
	DebugLine(const Vector3d& _from, const Vector3d& _to, float _width, const LinearColor& _color, int _frames_alive = 1) : from_(_from), to_(_to), width_(_width), color_(_color), frames_alive(_frames_alive) {}

	Vector3d from_;
	Vector3d to_;
	float width_;
	LinearColor color_;
	float time_alive; // Do we have a frame rate?
	int frames_alive;
};

// A simple Arrow data
struct Arrow
{
	Arrow(Vector3d _from, Vector3d _to, const LinearColor& _color, float _size = 5.0f, float _width = 1.0) :from(_from), to(_to), color(_color), arrow_size(_size), width(_width) {}
	Vector3d from;
	Vector3d to;
	float width;
	LinearColor color;
	float arrow_size;
};

/// This struct store the prism data structure and provides basic functionalities for retrieval and claculation
struct PrismProperty {
	// Vec3f FromVertPrismDir_DEPRECATED;
	// float FromVertPrismSize_DEPRECATED;
	// Vec3f ToVertPrimsDir_DEPRECATED;
	// float ToVertPrismSize_DEPRECATED;

	Vec3f FromVertPrismUp;
	Vec3f FromVertPrismDown;
	Vec3f ToVertPrismUp;
	Vec3f ToVertPrismDown;


	// This weight is set up in setup_prisms() in the beginning,
	// and it is NEVER changed when deformation happens
	float weight_ij;

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

	inline const Vec3f& f_uv(int u, int v, bool is_i_j) const
	{
		// We only care uv edges
		assert(u == 0 || u == 1);
		assert(v == 0 || v == 1);
		if (is_i_j)
		{
			return (v == 1) ? ( u == 0 ? FromVertPrismUp : ToVertPrismUp)
							  : ( u == 0 ? FromVertPrismDown : ToVertPrismDown);
		}
		else
		{
			return (v == 1) ? (u == 0 ? ToVertPrismUp: FromVertPrismUp)
							  : (u == 0 ? ToVertPrismDown: FromVertPrismDown);
		}
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
	virtual void setup_prisms(const std::vector<OpenMesh::FaceHandle> &face_handles, 
								EPrismExtrudeMode PrismExtrudeMode = EPrismExtrudeMode::FACE_NORMAL);
	virtual void update_prisms_height_uniform(const std::vector<OpenMesh::FaceHandle> &face_handles, const float dh);

	// optimize prisms based on optimizeMode_
	virtual void optimize_faces(const std::vector<OpenMesh::FaceHandle> &face_handles, 
										const std::unordered_map<int,int> &face_idx_2_i, const int max_iterations);
	// Locally optimize for one prism
	virtual void local_optimize(const std::vector<OpenMesh::FaceHandle> &face_handles, const int max_iterations);
	virtual void update_vertices_based_on_prisms();

	// Locally optimize for one prism faces 
	virtual void local_optimize_face(Mesh::FaceHandle _fh, const OpenMesh::HPropHandleT<PrismProperty> &, bool is_ij = false);


	// the wrapper for calculating the final rotation
	Transformation compute_optimal_face_transform(const Eigen::Matrix3f& S, const Vector3d &c_i, const Vector3d &c_star) const;

	// globally solve for all prism faces
	// mostly face_handles should be optimizedFaceHandles_
	virtual void global_optimize_faces(const std::vector<OpenMesh::FaceHandle> &face_handles, 
										const std::unordered_map<int,int> &face_idx_2_i, const int max_iterations);
	void project_v_and_update_prisms(const Eigen::VectorXf &C, const std::vector<OpenMesh::FaceHandle> &face_handles, float lambda);

	float calc_face_area(Mesh::FaceHandle _fh) const;

	/// get average vertex distance
    float get_average_vertex_distance(const Mesh& mesh) const;
	// For primo hierarchical, the solution should be based on a, say randomly sampled points, 
	// then find the shortest path along the edges

	enum class EViewMode{VIEW, MOVE} viewMode_;
	enum class ESelectMode{STATIC, DYNAMIC, OPTIMIZED, NONE} selectMode_;
	enum class EOptimizeMode{LOCAL = 0, GLOBAL = 1} optimizeMode_ ;
private:
	// Normalized direction
	OpenMesh::HPropHandleT<PrismProperty>  P_PrismProperty;
	OpenMesh::HPropHandleT<PrismProperty>  P_globalPrism_intermediate;

	//OpenMesh::FPropHandleT<Transformation> P_FaceTransformationCache;
	// press p to visualize prisms
	bool drawPrisms_;
	// press x to visualize other stuff
	bool drawDebugInfo_;
	// colors of faces of prisms
	GLfloat staticFacesColor_[3];
	GLfloat dynamicFacesColor_[3];
	GLfloat optimizedFacesColor_[3];
	// prism' height (homogeneous: all prisms' height are same now)
	float prismHeight_;
	float averageVertexDisance_;
	int global_optimize_iterations_;
	// 3 types of face handles
	// only optimize the optimizedFaces
	std::vector<OpenMesh::FaceHandle> optimizedFaceHandles_;
	// maintain a set of optimized faces' idx only for global optimization
	
	std::unordered_map<int, int> optimizedFaceIdx_2_i_;
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
	void delete_faceHandle(unsigned int faceId, std::vector<OpenMesh::FaceHandle> &face_handles,
							std::unordered_map<int, int> *face_idx_2_i = nullptr);
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

public:
	void read_mesh_and_cp(const std::string& mesh_filename, const std::string& crease_pattern_filename);
	void test_read_crease_pattern();
	void read_crease_pattern(const std::string& filename);

	// I might need t a beizer wrapper.
	
	//QYZ's version of the creases marked here.
	std::vector<std::vector<Mesh::HalfedgeHandle>> creases;

public:
	// Debug Utilities, these arrows will be added by local optimize. Every draw flushes the debug lines once. SHould be more optimized 
	void add_debug_arrow(const Vector3d& from, const Vector3d& to, LinearColor color, double arrow_size);
	void add_debug_coordinate(Transformation& world_transform, double size, Transformation base_transform = Transformation());
	void add_debug_line(Vector3d& from, Vector3d& to, LinearColor color, double width = 1.0f);

	std::list<Transformation> g_debug_transformations_to_draw_local_optimization;
	std::list<Arrow> g_debug_arrows_to_draw_local_optimizations;

public:
	void get_points_from_line(std::string& line, std::vector<Vector3f>& out_points, int& segments);
	// Get the handled of the closes vert on this mesh from the point
	Mesh::VertexHandle get_closes_vertex(Mesh::Point p);

	

private:
	std::list<DebugLine> debug_lines_;
	// Draw the lines and flush them
	void draw_debug_lines();	

	static void print_quaternion(Eigen::Quaternion<double>& Q);
private:
	// Utilities to calculate prism energy and decide if converge
	inline static bool converge_E(float Ek, float Ekm1) {
  		static const float sigma = 0.05f;
  		return (std::fabs(Ek - Ekm1) < sigma * Ek) ||
                 (Ek < 1e-8)
             ? true
             : false;
	}
	//defined in PrimoMeshViewer_Utilities.cpp
	float E(const std::vector<OpenMesh::FaceHandle> &face_handles)const;
	void squeeze_prisms(const std::vector<OpenMesh::FaceHandle> &face_handles, const OpenMesh::Vec3f &target);
	// used for moving camera while doing optimization 
	std::vector<std::thread> thread_pool_;

	bool bKey_space_is_move_;

};