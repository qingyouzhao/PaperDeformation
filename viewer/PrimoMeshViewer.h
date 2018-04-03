#pragma once

#include "MeshViewer.hh"


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

	enum class EViewMode{VIEW, MOVE} mode_;
private:
	// Normalized direction
	OpenMesh::HPropHandleT<PrismProperty>  P_PrismProperty;
};