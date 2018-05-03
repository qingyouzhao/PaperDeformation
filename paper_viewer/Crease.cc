#include "Crease.hh"
const GLfloat Crease::mountainEdgeColor_[3] = {1.0f, 0.0f, 0.0f};
	// default: valley(blue)
const GLfloat Crease::valleyEdgeColor_[3] = {0.0f, 0.0f, 1.0f};
Crease::Crease(const std::vector<OpenMesh::HalfedgeHandle> &other_he_handles, Mesh &mesh, const int type):
mesh_(mesh)
{
    // copy all edge handles into this crease
    this->he_handles_ = other_he_handles;
    // defaulty both side of faces are foldable
    // this will be setted when building OpUnits
    fromFace_foldable = toFace_foldable = std::vector<bool>(other_he_handles.size(), true);
    if(type == 2){
        this->crease_type_ = ECreaseType::MOUNTAIN;
    }else if(type == 3){
        this->crease_type_ = ECreaseType::VALLEY;
    }
    else{
        assert("Only Mountain and valley type is valid when initilizing a crease");
    }
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
    static constexpr int pv1i[9] = {0, 1, 0, 3, 4, 3, 0, 1, 2};
	static constexpr int pv2i[9] = {1, 2, 2, 4, 5, 5, 3, 4, 5};
    if(crease_type_ == ECreaseType::NONE){
        // do not draw NONE crease
        return;
    }
    const GLfloat *const edgeColor_ 
        = ( crease_type_ == ECreaseType::MOUNTAIN ? mountainEdgeColor_:valleyEdgeColor_);
    // change color
    glColor3fv(edgeColor_);
    // draw 8 lines of two faces
    for(const OpenMesh::HalfedgeHandle &he_i : he_handles_){
        Mesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
        const Mesh::FaceHandle fh_i = mesh_.face_handle(he_i);
        Mesh::ConstFaceHalfedgeCWIter fh_cwit_i = mesh_.cfh_cwbegin(fh_i);
		const OpenMesh::Vec3f* pv_i[6];// 6 vertices of prism
		for (int i = 0; fh_cwit_i.is_valid(); ++fh_cwit_i, ++i){
			assert(i < 3);
			const PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_cwit_i);
			// const Vec3f& from_v = mesh_.point(mesh_.from_vertex_handle(*fh_cwit));
			// pv[i]     = from_v + prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
			// pv[i + 3] = from_v - prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
			pv_i[i] = &(prop.FromVertPrismUp);
			pv_i[i + 3] = &(prop.FromVertPrismDown);
		}
		// have got all six vertices of prism, draw 9 edges
		// 01, 12, 02, 34, 45, 35, 03, 14, 25

		for(int i = 0; i < 9; ++i){
			glVertex3f((*pv_i[pv1i[i]])[0], (*pv_i[pv1i[i]])[1], (*pv_i[pv1i[i]])[2]);
			glVertex3f((*pv_i[pv2i[i]])[0], (*pv_i[pv2i[i]])[1], (*pv_i[pv2i[i]])[2]);
		}
        if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
            const Mesh::FaceHandle fh_j = mesh_.face_handle(he_j);
            Mesh::ConstFaceHalfedgeCWIter fh_cwit_j = mesh_.cfh_cwbegin(fh_j);
            const OpenMesh::Vec3f* pv_j[6];// 6 vertices of prism
            for (int i = 0; fh_cwit_j.is_valid(); ++fh_cwit_j, ++i){
                assert(i < 3);
                const PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_cwit_j);
                // const Vec3f& from_v = mesh_.point(mesh_.from_vertex_handle(*fh_cwit));
                // pv[i]     = from_v + prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
                // pv[i + 3] = from_v - prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
                pv_j[i] = &(prop.FromVertPrismUp);
                pv_j[i + 3] = &(prop.FromVertPrismDown);
            }
            // have got all six vertices of prism, draw 9 edges
            // 01, 12, 02, 34, 45, 35, 03, 14, 25

            for(int i = 0; i < 9; ++i){
                glVertex3f((*pv_j[pv1i[i]])[0], (*pv_j[pv1i[i]])[1], (*pv_j[pv1i[i]])[2]);
                glVertex3f((*pv_j[pv2i[i]])[0], (*pv_j[pv2i[i]])[1], (*pv_j[pv2i[i]])[2]);
            }
        }
    }





    // each triangle face has 6 prism vertices and 9 edges
	// for (const OpenMesh::FaceHandle& fh : face_handles){
	// 	Mesh::ConstFaceHalfedgeCWIter fh_cwit = mesh_.cfh_cwbegin(fh);
	// 	const Vec3f* pv[6];// 6 vertices of prism
	// 	for (int i = 0; fh_cwit.is_valid(); ++fh_cwit, ++i){
	// 		assert(i < 3);
	// 		const PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_cwit);
	// 		// const Vec3f& from_v = mesh_.point(mesh_.from_vertex_handle(*fh_cwit));
	// 		// pv[i]     = from_v + prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
	// 		// pv[i + 3] = from_v - prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
	// 		pv[i] = &(prop.FromVertPrismUp);
	// 		pv[i + 3] = &(prop.FromVertPrismDown);
	// 	}
	// 	// have got all six vertices of prism, draw 9 edges
	// 	// 01, 12, 02, 34, 45, 35, 03, 14, 25
	// 	static const int pv1i[9] = {0, 1, 0, 3, 4, 3, 0, 1, 2};
	// 	static const int pv2i[9] = {1, 2, 2, 4, 5, 5, 3, 4, 5};
	// 	for(int i = 0; i < 9; ++i){
	// 		glVertex3f((*pv[pv1i[i]])[0], (*pv[pv1i[i]])[1], (*pv[pv1i[i]])[2]);
	// 		glVertex3f((*pv[pv2i[i]])[0], (*pv[pv2i[i]])[1], (*pv[pv2i[i]])[2]);
	// 	}
	// }

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
    // need to rotate full prism of two faces of each half-handle
    // also need to change OpenMesh vertices of each faces
    for(const OpenMesh::HalfedgeHandle &he_i : he_handles_){
        Mesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
        PrismProperty &prop_i = mesh_.property(P_PrismProperty, he_i);
        const OpenMesh::Vec3f n_i = (prop_i.FromVertPrismUp - prop_i.FromVertPrismDown).normalized();
        const OpenMesh::Vec3f x_i = (prop_i.ToVertPrismUp - prop_i.FromVertPrismUp).normalized();
        const float height_i = (prop_i.FromVertPrismUp - prop_i.FromVertPrismDown).length() * 0.5f;
        const OpenMesh::Vec3f newN_i = (std::cos(dRad) * n_i - std::sin(dRad) * (OpenMesh::cross(n_i, x_i))).normalized() ;
        const Mesh::FaceHandle fh_i = mesh_.face_handle(he_i);
        Mesh::VertexHandle oppo_vi;
        for(Mesh::FaceVertexIter fv_it = mesh_.fv_begin(fh_i); fv_it.is_valid(); ++fv_it){
            if(mesh_.from_vertex_handle(he_i) != *fv_it && mesh_.to_vertex_handle(he_i) != *fv_it){
                oppo_vi = *fv_it;
                break;
            }
        }
        // opposite point is rotated
        Transformation tr(dRad, Vector3f(x_i[0], x_i[1],x_i[2]));
        mesh_.point(oppo_vi) = tr.transformPoint(mesh_.point(oppo_vi));
        for(Mesh::FaceHalfedgeIter fh_iter = mesh_.fh_begin(fh_i);fh_iter.is_valid();++fh_iter){
            //
            PrismProperty &prop = mesh_.property(P_PrismProperty, *fh_iter);
            const OpenMesh::Vec3f midFrom = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
            const OpenMesh::Vec3f midTo = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
            // update Prism data
            prop.FromVertPrismUp = midFrom + newN_i * height_i;
            prop.FromVertPrismDown = midFrom - newN_i * height_i;
            prop.ToVertPrismUp = midTo + newN_i * height_i;
            prop.ToVertPrismDown = midTo - newN_i * height_i;
            // update OpenMesh vertices
        }
        // for(Mesh::FaceHalfedgeIter fh_iter = mesh_.fh_begin(fh_i);fh_iter.is_valid();++fh_iter){
        //     PrismProperty &prop = mesh_.property(P_PrismProperty, *fh_iter);
        //     mesh_.point(mesh_.from_vertex_handle(*fh_iter)) = prop.TargetPosFrom();
        //     mesh_.point(mesh_.to_vertex_handle(*fh_iter)) = prop.TargetPosTo();
        // }
        // if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
        //     PrismProperty &prop_j = mesh_.property(P_PrismProperty, he_j);
        //     const OpenMesh::Vec3f n_j = (prop_j.FromVertPrismUp - prop_j.FromVertPrismDown).normalized();
        //     const OpenMesh::Vec3f x_j = (prop_j.ToVertPrismUp - prop_j.FromVertPrismUp).normalized();
        //     const float height_j = (prop_j.FromVertPrismUp - prop_j.FromVertPrismDown).length() * 0.5f;
        //     const OpenMesh::Vec3f newN_j = (std::cos(dRad) * n_j - std::sin(dRad) * (OpenMesh::cross(n_j, x_j))).normalized();
        //     const Mesh::FaceHandle fh_j = mesh_.face_handle(he_j);
        //     for(Mesh::FaceHalfedgeIter fh_iter = mesh_.fh_begin(fh_j);fh_iter.is_valid();++fh_iter){
        //         //
        //         PrismProperty &prop = mesh_.property(P_PrismProperty, *fh_iter);
        //         const OpenMesh::Vec3f midFrom = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
        //         const OpenMesh::Vec3f midTo = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
        //         // update Prism data
        //         prop.FromVertPrismUp = midFrom + newN_j * height_j;
        //         prop.FromVertPrismDown = midFrom - newN_j * height_j;
        //         prop.ToVertPrismUp = midTo + newN_j * height_j;
        //         prop.ToVertPrismDown = midTo - newN_j * height_j;
        //         // update OpenMesh vertices
        //         mesh_.point(mesh_.from_vertex_handle(*fh_iter)) = prop.TargetPosFrom();
        //         mesh_.point(mesh_.to_vertex_handle(*fh_iter)) = prop.TargetPosTo();
        //     }
        // }
    }
}
