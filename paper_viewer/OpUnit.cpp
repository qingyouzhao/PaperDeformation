#include "OpUnit.h"
#include "gl.hh"
// given a single FaceHandle, build a unit only contain single triangle(prism)
OpUnit::OpUnit(const OpenMesh::FaceHandle &fh, Mesh &mesh):mesh_(mesh){
    face_handles_.emplace_back(fh);
    // the boundary of a single triangle is just its inner halfedges
    for(Mesh::FaceHalfedgeCCWIter fh_ccwiter = mesh_.fh_ccwbegin(fh); fh_ccwiter.is_valid(); ++fh_ccwiter){
        boundary_he_handles_.emplace_back(*fh_ccwiter);
    }
    assert(boundary_he_handles_.size() == 3);
    assert(face_handles_.size() == 1);
}
// given double pair of FaceHandles, with one public edge, build a unit contains double triangles(prisms)
OpUnit::OpUnit(const OpenMesh::FaceHandle &fh_i, const OpenMesh::FaceHandle &fh_j, const OpenMesh::HalfedgeHandle &he_i, 
                    Mesh &mesh):mesh_(mesh){
    face_handles_.emplace_back(fh_i);
    face_handles_.emplace_back(fh_j);
    // the boundary of double triangles is just 4 halfedges except for the 2 on the public edge
    const OpenMesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
    for(Mesh::FaceHalfedgeCCWIter fh_ccwiter = mesh_.fh_ccwbegin(fh_i); fh_ccwiter.is_valid(); ++fh_ccwiter){
        if(fh_ccwiter->idx() == he_i.idx() || fh_ccwiter->idx() == he_j.idx()) continue;
        boundary_he_handles_.emplace_back(*fh_ccwiter);
    }
    for(Mesh::FaceHalfedgeCCWIter fh_ccwiter = mesh_.fh_ccwbegin(fh_j); fh_ccwiter.is_valid(); ++fh_ccwiter){
        if(fh_ccwiter->idx() == he_i.idx() || fh_ccwiter->idx() == he_j.idx()) continue;
        boundary_he_handles_.emplace_back(*fh_ccwiter);
    }
    assert(boundary_he_handles_.size() == 4);
    assert(face_handles_.size() == 2);
}
void OpUnit::draw_prism(const OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty) const{
    // draw boundary prism of this OpUnit
    for(const OpenMesh::HalfedgeHandle &he : boundary_he_handles_){
        const PrismProperty &prop(mesh_.property(P_PrismProperty,he));
        const OpenMesh::Vec3f &from_down = prop.FromVertPrismDown;
        const OpenMesh::Vec3f &from_up   = prop.FromVertPrismUp;
        const OpenMesh::Vec3f &to_down   = prop.ToVertPrismDown;
        const OpenMesh::Vec3f &to_up     = prop.ToVertPrismUp;
        // down line
        glVertex3f(from_down[0], from_down[1], from_down[2]);
        glVertex3f(to_down[0], to_down[1], to_down[2]);

        // up line
        glVertex3f(from_up[0], from_up[1], from_up[2]);
        glVertex3f(to_up[0], to_up[1], to_up[2]);

        // from line
        glVertex3f(from_down[0], from_down[1], from_down[2]);
        glVertex3f(from_up[0], from_up[1], from_up[2]);

        // to line
        glVertex3f(to_down[0], to_down[1], to_down[2]);
        glVertex3f(to_up[0], to_up[1], to_up[2]);
    }
}
