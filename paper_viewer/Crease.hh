#pragma once
#include <vector>
#include "PrismProperty.hh"
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
    inline void set_foldable(const bool isFoldable){
        foldable_ = isFoldable;
    }
    void set_prism_height(const float height, OpenMesh::HPropHandleT<PrismProperty> &p_handle);
private:
    // crease handles for each Crease
    std::vector<OpenMesh::HalfedgeHandle> he_handles_;
    Mesh &mesh_;
    // if foldable or not. Default: true
    bool foldable_;
    //
};