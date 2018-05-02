#pragma once

#include "MeshViewer.hh"
#include "Transformation.hh"
#include "Crease.hh"
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

	enum class ESelectMode{OPTIMIZED, FOLDED};
	enum class EOptimizeMode{LOCAL = 0, GLOBAL = 1} optimizeMode_ ;
private:
	// Normalized direction
	OpenMesh::HPropHandleT<PrismProperty>  P_PrismProperty;
	OpenMesh::HPropHandleT<PrismProperty>  P_globalPrism_intermediate;
	// property used for handle self-collision
	OpenMesh::FPropHandleT<PrismFaceProperty> P_faceBN;
	OpenMesh::HPropHandleT<PrismCollisionProperty> P_collision;
	void setup_faceBN(const std::vector<OpenMesh::FaceHandle> &face_handles);
	void setup_collisionProperty();
	//OpenMesh::FPropHandleT<Transformation> P_FaceTransformationCache;
	// press p to visualize prisms
	bool drawPrisms_;
	// press x to visualize other stuff
	bool drawDebugInfo_;
	// colors of faces of prisms
	GLfloat optimizedFacesColor_[3];
	GLfloat notOptimizaedFacesColor_[3];
	// prism' height (homogeneous: all prisms' height are same now)
	float prismHeight_;
	float averageVertexDisance_;
	int global_optimize_iterations_;
	// 3 types of face handles
	// only optimize the optimizedFaces
	std::vector<OpenMesh::FaceHandle> optimizedFaceHandles_;
	std::vector<OpenMesh::FaceHandle> not_optimizedFaceHandles_;
	// maintain a set of optimized faces' idx only for global optimization
	
	std::unordered_map<int, int> optimizedFaceIdx_2_i_;
	//std::vector<unsigned int> optimizedVertexIndices_;
	
	// used for ray-casting, from prim_id to faceHandle
	std::vector<OpenMesh::FaceHandle> allFaceHandles_;
	//std::vector<unsigned int> allVertexIndices_;

	// face_handles is cleared and filled with all face handles in mesh_
	void get_allFace_handles(std::vector<OpenMesh::FaceHandle> &face_handles);
	
	
	void update_1typeface_indices(const std::vector<OpenMesh::FaceHandle> &face_handles, 
										std::vector<unsigned int> &indices);
	
	// delete a face_handle(fh) from face_handles, where fh.idx() == faceId
	// O(n), need optimize if this is raycast bottleneck
	// void delete_faceHandle(unsigned int faceId, std::vector<OpenMesh::FaceHandle> &face_handles,
	// 						std::unordered_map<int, int> *face_idx_2_i = nullptr);
	// each face could only have one type of STATIC/DYNAMI/NONE
	//std::unordered_map<unsigned int, ESelectMode> faceIdx_to_selType_;
	// draw prisms for all faces in array(vector)
	void draw_prisms(const std::vector<OpenMesh::FaceHandle> &face_handles) const;

public:
	//void test_read_crease_pattern();

	// Triangulate based on a connected list of half edges
	void triangulate_by_boundary(const std::vector<HalfedgeHandle>& boundary_hehs);

	// check if this list is a legal boundary
	bool is_legal_boundary(const std::vector<HalfedgeHandle>& boundary_hehs) const;
	// I might need t a beizer wrapper.
	
	//QYZ's version of the creases marked here.
	// std::vector<std::vector<Mesh::HalfedgeHandle>> creases;

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
private:
	float folding_angle_;// in degree, default: 0 degree

	// functions for paper folding
	bool read_dcc_file(const std::string &dcc_file_name);
	void update_vertices_based_on_prisms_self_collision();

	// data for paper folding
	std::vector<Crease> creases_;
};