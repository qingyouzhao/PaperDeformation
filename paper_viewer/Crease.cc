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
        PrismProperty &prop_i = mesh_.property(P_PrismProperty, he_i);
        const OpenMesh::Vec3f dVertical_i = (prop_i.FromVertPrismUp - prop_i.FromVertPrismDown).normalized();
        const OpenMesh::Vec3f midFrom_i = (prop_i.FromVertPrismUp + prop_i.FromVertPrismDown) * 0.5f;
        const OpenMesh::Vec3f midTo_i = (prop_i.ToVertPrismUp + prop_i.ToVertPrismDown) * 0.5f;
        prop_i.FromVertPrismUp = midFrom_i + dVertical_i * height;
        prop_i.FromVertPrismDown = midFrom_i - dVertical_i * height;
        prop_i.ToVertPrismUp = midTo_i + dVertical_i * height;
        prop_i.ToVertPrismDown = midTo_i - dVertical_i * height;
        if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
            PrismProperty &prop_j = mesh_.property(P_PrismProperty, he_j);
            const OpenMesh::Vec3f dVertical_j = (prop_j.FromVertPrismUp - prop_j.FromVertPrismDown).normalized();
            const OpenMesh::Vec3f midFrom_j = (prop_j.FromVertPrismUp + prop_j.FromVertPrismDown) * 0.5f;
            const OpenMesh::Vec3f midTo_j = (prop_j.ToVertPrismUp + prop_j.ToVertPrismDown) * 0.5f;
            prop_j.FromVertPrismUp = midFrom_j + dVertical_j * height;
            prop_j.FromVertPrismDown = midFrom_j - dVertical_j * height;
            prop_j.ToVertPrismUp = midTo_j + dVertical_j * height;
            prop_j.ToVertPrismDown = midTo_j - dVertical_j * height;
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
        const PrismProperty &prop_i = mesh_.property(P_PrismProperty, he_i);
        glVertex3f(prop_i.FromVertPrismDown[0], prop_i.FromVertPrismDown[1], prop_i.FromVertPrismDown[2]);
		glVertex3f(prop_i.FromVertPrismUp[0], prop_i.FromVertPrismUp[1], prop_i.FromVertPrismUp[2]);
        
        glVertex3f(prop_i.ToVertPrismDown[0], prop_i.ToVertPrismDown[1], prop_i.ToVertPrismDown[2]);
		glVertex3f(prop_i.ToVertPrismUp[0], prop_i.ToVertPrismUp[1], prop_i.ToVertPrismUp[2]);
        
        glVertex3f(prop_i.FromVertPrismDown[0], prop_i.FromVertPrismDown[1], prop_i.FromVertPrismDown[2]);
		glVertex3f(prop_i.ToVertPrismDown[0], prop_i.ToVertPrismDown[1], prop_i.ToVertPrismDown[2]);
        
        glVertex3f(prop_i.FromVertPrismUp[0], prop_i.FromVertPrismUp[1], prop_i.FromVertPrismUp[2]);
		glVertex3f(prop_i.ToVertPrismUp[0], prop_i.ToVertPrismUp[1], prop_i.ToVertPrismUp[2]);
        if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
            const PrismProperty &prop_j = mesh_.property(P_PrismProperty, he_j);
            glVertex3f(prop_j.FromVertPrismDown[0], prop_j.FromVertPrismDown[1], prop_j.FromVertPrismDown[2]);
            glVertex3f(prop_j.FromVertPrismUp[0], prop_j.FromVertPrismUp[1], prop_j.FromVertPrismUp[2]);
            
            glVertex3f(prop_j.ToVertPrismDown[0], prop_j.ToVertPrismDown[1], prop_j.ToVertPrismDown[2]);
            glVertex3f(prop_j.ToVertPrismUp[0], prop_j.ToVertPrismUp[1], prop_j.ToVertPrismUp[2]);
            
            glVertex3f(prop_j.FromVertPrismDown[0], prop_j.FromVertPrismDown[1], prop_j.FromVertPrismDown[2]);
            glVertex3f(prop_j.ToVertPrismDown[0], prop_j.ToVertPrismDown[1], prop_j.ToVertPrismDown[2]);
            
            glVertex3f(prop_j.FromVertPrismUp[0], prop_j.FromVertPrismUp[1], prop_j.FromVertPrismUp[2]);
            glVertex3f(prop_j.ToVertPrismUp[0], prop_j.ToVertPrismUp[1], prop_j.ToVertPrismUp[2]);
        }
    }

}
void Crease::fold(float dAngle, OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty){
    // #TODO[ZJW]: this implementation is using incremental way. Need review if it is not correct.

    // based on current pair of prisms along the crease, rotate dAngle(consistent with the crease type)
    if(crease_type_ == ECreaseType::NONE){
        // do not draw NONE crease
        return;
    }
    if(crease_type_ == ECreaseType::VALLEY){
        dAngle = -dAngle;
    }
    //
    const float dRad = dAngle * M_PI / 180.0f;
    for(const OpenMesh::HalfedgeHandle &he_i : he_handles_){
        Mesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
        PrismProperty &prop_i = mesh_.property(P_PrismProperty, he_i);
        const OpenMesh::Vec3f n_i = (prop_i.FromVertPrismUp - prop_i.FromVertPrismDown).normalized();
        const OpenMesh::Vec3f x_i = (prop_i.ToVertPrismUp - prop_i.FromVertPrismUp).normalized();
        const float height_i = (prop_i.FromVertPrismUp - prop_i.FromVertPrismDown).length() * 0.5f;
        const OpenMesh::Vec3f newN_i = std::cos(dRad) * n_i - std::sin(dRad) * (OpenMesh::cross(n_i, x_i));
        const Mesh::FaceHandle fh_i = mesh_.face_handle(he_i);
        for(Mesh::FaceHalfedgeIter fh_iter = mesh_.fh_begin(fh_i);fh_iter.is_valid();++fh_iter){
            //
            PrismProperty &prop = mesh_.property(P_PrismProperty, *fh_iter);
            const OpenMesh::Vec3f midFrom = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
            const OpenMesh::Vec3f midTo = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
            prop.FromVertPrismUp = midFrom + newN_i * height_i;
            prop.FromVertPrismDown = midFrom - newN_i * height_i;
            prop.ToVertPrismUp = midTo + newN_i * height_i;
            prop.ToVertPrismDown = midTo - newN_i * height_i;
        }
        if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
            PrismProperty &prop_j = mesh_.property(P_PrismProperty, he_j);
            const OpenMesh::Vec3f n_j = (prop_j.FromVertPrismUp - prop_j.FromVertPrismDown).normalized();
            const OpenMesh::Vec3f x_j = (prop_j.ToVertPrismUp - prop_j.FromVertPrismUp).normalized();
            const float height_j = (prop_j.FromVertPrismUp - prop_j.FromVertPrismDown).length() * 0.5f;
            const OpenMesh::Vec3f newN_j = std::cos(dRad) * n_j - std::sin(dRad) * (OpenMesh::cross(n_j, x_j));
            const Mesh::FaceHandle fh_j = mesh_.face_handle(he_j);
            for(Mesh::FaceHalfedgeIter fh_iter = mesh_.fh_begin(fh_j);fh_iter.is_valid();++fh_iter){
                //
                PrismProperty &prop = mesh_.property(P_PrismProperty, *fh_iter);
                const OpenMesh::Vec3f midFrom_j = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
                const OpenMesh::Vec3f midTo_j = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
                prop.FromVertPrismUp = midFrom_j + newN_j * height_j;
                prop.FromVertPrismDown = midFrom_j - newN_j * height_j;
                prop.ToVertPrismUp = midTo_j + newN_j * height_j;
                prop.ToVertPrismDown = midTo_j - newN_j * height_j;
            }
        }
    }
}
