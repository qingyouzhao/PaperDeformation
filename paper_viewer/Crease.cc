#include "Crease.hh"
const GLfloat Crease::mountainEdgeColor_[3] = {1.0f, 0.0f, 0.0f};
	// default: valley(blue)
const GLfloat Crease::valleyEdgeColor_[3] = {0.0f, 0.0f, 1.0f};
Crease::Crease(const std::vector<OpenMesh::HalfedgeHandle> &other_he_handles, Mesh &mesh):
mesh_(mesh)
{
    // copy all edge handles into this crease
    this->he_handles_ = other_he_handles;
    this->crease_type_ = ECreaseType::NONE;
}
void Crease::set_prism_height(const float height, OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty){
    // only set prisms height of pair of halfedges
    for(const OpenMesh::HalfedgeHandle &he_i : he_handles_){
        Mesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
        PrismProperty &prop = mesh_.property(P_PrismProperty, he_i);
        const OpenMesh::Vec3f dFrom = (prop.FromVertPrismUp - prop.FromVertPrismDown).normalized();
        const OpenMesh::Vec3f dTo = (prop.ToVertPrismUp - prop.ToVertPrismDown).normalized();
        const OpenMesh::Vec3f midFrom = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
        const OpenMesh::Vec3f midTo = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
        prop.FromVertPrismUp = midFrom + dFrom * height;
        prop.FromVertPrismDown = midFrom + dFrom * height;
        prop.ToVertPrismUp = midTo + dTo * height;
        prop.ToVertPrismDown = midTo - dTo * height;
        if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
            PrismProperty &prop = mesh_.property(P_PrismProperty, he_j);
            const OpenMesh::Vec3f dFrom = (prop.FromVertPrismUp - prop.FromVertPrismDown).normalized();
            const OpenMesh::Vec3f dTo = (prop.ToVertPrismUp - prop.ToVertPrismDown).normalized();
            const OpenMesh::Vec3f midFrom = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
            const OpenMesh::Vec3f midTo = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
            prop.FromVertPrismUp = midFrom + dFrom * height;
            prop.FromVertPrismDown = midFrom + dFrom * height;
            prop.ToVertPrismUp = midTo + dTo * height;
            prop.ToVertPrismDown = midTo - dTo * height;
        }
    }
}
void Crease::draw() const{
    if(crease_type_ == ECreaseType::NONE){
        // do not draw NONE crease
        return;
    }
    const GLfloat *const edgeColor_ 
        = ( crease_type_ == ECreaseType::MOUNTAIN ? mountainEdgeColor_:valleyEdgeColor_);
    // change color
    glColor3fv(edgeColor_);
    for(const OpenMesh::HalfedgeHandle &he : he_handles_){
        const OpenMesh::Vec3f &from_pos = mesh_.point(mesh_.from_vertex_handle(he));
        const OpenMesh::Vec3f &to_pos   = mesh_.point(mesh_.to_vertex_handle(he));
        glVertex3f(from_pos[0], from_pos[1], from_pos[2]);
		glVertex3f(to_pos[0], to_pos[1], to_pos[2]);
    }
}
void Crease::draw_prisms(const OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty) const{
    if(crease_type_ == ECreaseType::NONE){
        // do not draw NONE crease
        return;
    }
    const GLfloat *const edgeColor_ 
        = ( crease_type_ == ECreaseType::MOUNTAIN ? mountainEdgeColor_:valleyEdgeColor_);
    // draw 8 lines of two faces
    for(const OpenMesh::HalfedgeHandle &he_i : he_handles_){
        Mesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
        const PrismProperty &prop = mesh_.property(P_PrismProperty, he_i);
        glVertex3f(prop.FromVertPrismDown[0], prop.FromVertPrismDown[1], prop.FromVertPrismDown[2]);
		glVertex3f(prop.FromVertPrismUp[0], prop.FromVertPrismUp[1], prop.FromVertPrismUp[2]);
        
        glVertex3f(prop.ToVertPrismDown[0], prop.ToVertPrismDown[1], prop.ToVertPrismDown[2]);
		glVertex3f(prop.ToVertPrismUp[0], prop.ToVertPrismUp[1], prop.ToVertPrismUp[2]);
        
        glVertex3f(prop.FromVertPrismDown[0], prop.FromVertPrismDown[1], prop.FromVertPrismDown[2]);
		glVertex3f(prop.ToVertPrismDown[0], prop.ToVertPrismDown[1], prop.ToVertPrismDown[2]);
        
        glVertex3f(prop.FromVertPrismUp[0], prop.FromVertPrismUp[1], prop.FromVertPrismUp[2]);
		glVertex3f(prop.ToVertPrismUp[0], prop.ToVertPrismUp[1], prop.ToVertPrismUp[2]);
        if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
            const PrismProperty &prop = mesh_.property(P_PrismProperty, he_j);
            glVertex3f(prop.FromVertPrismDown[0], prop.FromVertPrismDown[1], prop.FromVertPrismDown[2]);
            glVertex3f(prop.FromVertPrismUp[0], prop.FromVertPrismUp[1], prop.FromVertPrismUp[2]);
            
            glVertex3f(prop.ToVertPrismDown[0], prop.ToVertPrismDown[1], prop.ToVertPrismDown[2]);
            glVertex3f(prop.ToVertPrismUp[0], prop.ToVertPrismUp[1], prop.ToVertPrismUp[2]);
            
            glVertex3f(prop.FromVertPrismDown[0], prop.FromVertPrismDown[1], prop.FromVertPrismDown[2]);
            glVertex3f(prop.ToVertPrismDown[0], prop.ToVertPrismDown[1], prop.ToVertPrismDown[2]);
            
            glVertex3f(prop.FromVertPrismUp[0], prop.FromVertPrismUp[1], prop.FromVertPrismUp[2]);
            glVertex3f(prop.ToVertPrismUp[0], prop.ToVertPrismUp[1], prop.ToVertPrismUp[2]);
        }
    }

}

