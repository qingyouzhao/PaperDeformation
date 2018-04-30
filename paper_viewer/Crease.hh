#pragma once
#include <vector>
#include "PrismProperty.hh"
#include "gl.hh"
// used for store all creased
class Crease{
public:
    typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
    
    explicit Crease(const std::vector<OpenMesh::HalfedgeHandle> &other_edge_handles, Mesh &mesh);
    inline OpenMesh::HalfedgeHandle& operator[](const int& i){ 
        assert(i >= 0 && i < he_handles_.size());
        return he_handles_[i];
    }
    inline const OpenMesh::HalfedgeHandle& operator[](const int& i) const{ 
        assert(i >= 0 && i < he_handles_.size());
        return he_handles_[i];
    }
    void set_prism_height(const float height, OpenMesh::HPropHandleT<PrismProperty> &p_handle);
    enum class ECreaseType{NONE, MOUNTAIN, VALLEY} crease_type_;
    void draw() const;
    void draw_prisms(const OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty) const; 
private:
    // crease handles for each Crease
    std::vector<OpenMesh::HalfedgeHandle> he_handles_;
    Mesh &mesh_;
    // default: mountain(red)
    static const GLfloat mountainEdgeColor_[3];
	// default: valley(blue)
	static const GLfloat valleyEdgeColor_[3];
};