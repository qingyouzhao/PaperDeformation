#pragma once
#include <vector>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "PrismProperty.hh"
class OpUnit{
public:
    typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
    // given a single FaceHandle, build a unit only contain single triangle(prism)
    explicit OpUnit(const OpenMesh::FaceHandle &fh, Mesh &mesh);
    // given double pair of FaceHandles, with one public edge, build a unit contains double triangles(prisms)
    explicit OpUnit(const OpenMesh::FaceHandle &fh_i, const OpenMesh::FaceHandle &fh_j, 
            const OpenMesh::HalfedgeHandle &public_he, Mesh &mesh);
    // draw boundary prism of this OpUnit
    void draw_prism(const OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty) const;
public:
    std::vector<OpenMesh::FaceHandle> face_handles_;
    std::vector<OpenMesh::HalfedgeHandle> boundary_he_handles_;
    Mesh &mesh_;
};