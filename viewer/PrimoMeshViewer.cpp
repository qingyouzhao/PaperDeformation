#include "PrimoMeshViewer.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>


PrimoMeshViewer::PrimoMeshViewer(const char* _title, int _width, int _height)
	: MeshViewer(_title, _width, _height)
{
	mesh_.request_vertex_colors();

	mesh_.add_property(P_PrismProperty);

	// set color of 3 kind of faces(prisms) here 
	// default: dynamic(orange), 
	dynamicFacesColor_[0] = 1.0f;
	dynamicFacesColor_[1] = 0.64453125f;
	dynamicFacesColor_[2] = 0.0f;
	// static(dark green)
	staticFacesColor_[0] = 0.18039215686f;
	staticFacesColor_[1] = 0.54509803921f;
	staticFacesColor_[2] = 0.34117647058f;
	// optimized(blue)
	optimizedFacesColor_[0] = 0.5294117647f;
	optimizedFacesColor_[1] = 0.80784313725f;
	optimizedFacesColor_[2] = 0.98039215686f;
	
	// do not draw prisms at first
	drawPrisms_ = false;
	
	// Select mode is STATIC at first
	selectMode_ = ESelectMode::STATIC;
	printf("Select Mode: Static\n");

}

PrimoMeshViewer::~PrimoMeshViewer()
{

}

bool PrimoMeshViewer::open_mesh(const char* _filename)
{
	if (MeshViewer::open_mesh(_filename))
	{
		// do pre pass of stuff.

		glutPostRedisplay();

		// after successfully opening the mesh, we need firstly calculate average vertices distance
		// prismHeight_ = average vertices distance
		averageVertexDisance_ = get_average_vertex_distance(mesh_);
		prismHeight_ = averageVertexDisance_;

		// init face handles for ray-casting lookup from prim_id to faceHandle
		get_allFace_handles(allFaceHandles_);
		// build bvh for all faces using nanort at first
		build_allFace_BVH();
		// and then, prisms are set up 
		setup_prisms(EPrismExtrudeMode::VERT_NORMAL);

		// default: all faces are optimizable
		get_allFace_handles(optimizedFaceHandles_);
		update_1typeface_indices(optimizedFaceHandles_, optimizedVertexIndices_);
		for(const OpenMesh::FaceHandle& fh: optimizedFaceHandles_){
			faceIdx_to_selType[fh.idx()] = ESelectMode::NONE;
		}

		return true;
	}
	return false;
}

void PrimoMeshViewer::draw(const std::string& _draw_mode)
{
	if (indices_.empty())
	{
        GlutExaminer::draw(_draw_mode);
        return;
    }



    if (_draw_mode == "Wireframe")
    {
        glDisable(GL_LIGHTING);
        glColor3f(1.0, 1.0, 1.0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glEnableClientState(GL_VERTEX_ARRAY);
        GL::glVertexPointer(mesh_.points());

        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, &indices_[0]);

        glDisableClientState(GL_VERTEX_ARRAY);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	else if (_draw_mode == "Hidden Line")
	{

	    glDisable(GL_LIGHTING);
	    glShadeModel(GL_SMOOTH);
	    glColor3f(0.3, 0.3, 0.3);

	    glEnableClientState(GL_VERTEX_ARRAY);
	    GL::glVertexPointer(mesh_.points());

	    glDepthRange(0.01, 1.0);
	    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, &indices_[0]);
	    glDisableClientState(GL_VERTEX_ARRAY);
	    glColor3f(1.0, 1.0, 1.0);

	    glEnableClientState(GL_VERTEX_ARRAY);
	    GL::glVertexPointer(mesh_.points());

	    glDrawBuffer(GL_BACK);
	    glDepthRange(0.0, 1.0);
	    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	    glDepthFunc(GL_LEQUAL);
	    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, &indices_[0]);

	    glDisableClientState(GL_VERTEX_ARRAY);
	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	    glDepthFunc(GL_LESS);
	}
	else if (_draw_mode == "Solid Flat")
	{

		Mesh::ConstFaceVertexIter  fv_it;
		glEnable(GL_LIGHTING);
		glShadeModel(GL_FLAT);
		glEnable(GL_COLOR_MATERIAL);
		// draw 3 kind of faces
		glColor3fv(optimizedFacesColor_);
		glBegin(GL_TRIANGLES);
		for (const OpenMesh::FaceHandle& fh : optimizedFaceHandles_)
		{
			GL::glNormal(mesh_.normal(fh));
			fv_it = mesh_.cfv_iter(fh); 
			GL::glVertex(mesh_.point(*fv_it));
			++fv_it;
			GL::glVertex(mesh_.point(*fv_it));
			++fv_it;
			GL::glVertex(mesh_.point(*fv_it));
		}
		glEnd();
		//
		glColor3fv(dynamicFacesColor_);
		glBegin(GL_TRIANGLES);
		for (const OpenMesh::FaceHandle& fh : dynamicFaceHandles_)		
		{
			GL::glNormal(mesh_.normal(fh));
			fv_it = mesh_.cfv_iter(fh); 
			GL::glVertex(mesh_.point(*fv_it));
			++fv_it;
			GL::glVertex(mesh_.point(*fv_it));
			++fv_it;
			GL::glVertex(mesh_.point(*fv_it));
		}
		glEnd();
		//
		glColor3fv(staticFacesColor_);
		glBegin(GL_TRIANGLES);
		for (const OpenMesh::FaceHandle& fh : staticFaceHandles_)	
		{
			GL::glNormal(mesh_.normal(fh));
			fv_it = mesh_.cfv_iter(fh); 
			GL::glVertex(mesh_.point(*fv_it));
			++fv_it;
			GL::glVertex(mesh_.point(*fv_it));
			++fv_it;
			GL::glVertex(mesh_.point(*fv_it));
		}
		glEnd();
		glDisable(GL_COLOR_MATERIAL);
	}
	else if (_draw_mode == "Solid Smooth")
	{
		glEnable(GL_LIGHTING);
		glShadeModel(GL_SMOOTH);
		glEnable(GL_COLOR_MATERIAL);
		
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		GL::glVertexPointer(mesh_.points());
		GL::glNormalPointer(mesh_.vertex_normals());
		// draw 3 type of faces
		glColor3fv(optimizedFacesColor_);
		glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(optimizedVertexIndices_.size()), GL_UNSIGNED_INT, &optimizedVertexIndices_[0]);

		glColor3fv(dynamicFacesColor_);
		glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(dynamicVertexIndices_.size()), GL_UNSIGNED_INT, &dynamicVertexIndices_[0]);

		glColor3fv(staticFacesColor_);
		glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(staticVertexIndices_.size()), GL_UNSIGNED_INT, &staticVertexIndices_[0]);
		//
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisable(GL_COLOR_MATERIAL);
	}
	if(drawPrisms_){
		// visualize prisms with wireframes
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_LIGHTING);
		glColor3fv(dynamicFacesColor_);
		glBegin(GL_LINES);
		// draw 3 types of prisms with different color
		glColor3fv(dynamicFacesColor_);
		draw_prisms(dynamicFaceHandles_);
		glColor3fv(staticFacesColor_);
		draw_prisms(staticFaceHandles_);
		glColor3fv(optimizedFacesColor_);
		draw_prisms(optimizedFaceHandles_);
		//
		glEnd();
		glDisable(GL_COLOR_MATERIAL);
	}
}


void PrimoMeshViewer::keyboard(int key, int x, int y)
{
	// Let super handle us first
	switch (key)
	{
	case 'a':
	{
		// toggle if visualize prisms
		drawPrisms_ = !drawPrisms_;
		glutPostRedisplay();
	}
		break;
	case '+':
	{
		// add prisms' height
		prismHeight_ += averageVertexDisance_ * 0.1f;
		// immediately update all prisms
		setup_prisms(EPrismExtrudeMode::VERT_NORMAL);
		printf("prismHeight: %f\n", prismHeight_);

		// #TODO[ZJW][QYZ]: following the PriMo demo, after changing the prisms' height, we should at once optimize all surface
		// based on the new prism height.
		glutPostRedisplay();
	}
		break;
	case '-':
	{
		// minus prisms' height(keep prisms' height > 0)
		if(prismHeight_ - averageVertexDisance_ * 0.1f > FLT_EPSILON)
			prismHeight_ -= averageVertexDisance_ * 0.1f;
		// immediately update all prisms
		setup_prisms(EPrismExtrudeMode::VERT_NORMAL);
		printf("prismHeight: %f\n", prismHeight_);

		// #TODO[ZJW][QYZ]: following the PriMo demo, after changing the prisms' height, we should at once optimize all surface
		// based on the new prism height.
		glutPostRedisplay();
	}
		break;
	case '1':
	{
		// select mode changed to STATIC
		// which means that pressing shift + left mouse will select STATIC faces
		printf("Select Mode: Static\n");

		// rebuild nanort bvh for all faces before selecting STATIC/DYNAMIC faces
		if(selectMode_ == ESelectMode::NONE){
			build_allFace_BVH();
		}
		selectMode_ = ESelectMode::STATIC;
	}
		break;
	case '2':
	{
		// select mode changed to DYNAMIC
		// which means that pressing shift + left mouse will select DYNAMIC faces
		printf("Select Mode: Dynamic\n");

		// rebuild nanort bvh for all faces before selecting STATIC/DYNAMIC faces
		if(selectMode_ == ESelectMode::NONE){
			build_allFace_BVH();
		}
		selectMode_ = ESelectMode::DYNAMIC;

	}
		break;
	case '3':
	{
		// select mode changed to NONE
		// which means that pressing shift + left mouse and moving mouse will rotate all DYNAMIC faces
		// Also, pressing shift + middle mouse and moving mouse will translate all DYNAMIC faces
		selectMode_ = ESelectMode::NONE;
		printf("Select Mode: None\n"); 
	}
		break;
	default:
		GlutExaminer::keyboard(key, x, y);
		break;
	}
}

void PrimoMeshViewer::motion(int x, int y)
{
	switch (viewMode_)
    {
        default:
        case EViewMode::VIEW:
        {
            GlutExaminer::motion(x, y);
            break;
        }
        case EViewMode::MOVE:
        {
            // rotate or translate dynamicFaces when pressing shift + left/middle mouse button 
			// and in ESelect::None mode

			if(selectMode_ == ESelectMode::NONE){
				// #TODO[ZJW]: rotate or translate dynamicFaces when pressing shift + left/middle mouse button 
				// and in ESelect::None mode
			}

            // // rotation
            // if (button_down_[0])
            // {
            //     if (last_point_ok_)
            //     {
            //         Vec2i  new_point_2D;
            //         Vec3f  new_point_3D;
            //         bool   new_point_ok;

            //         new_point_2D = Vec2i(x, y);
            //         new_point_ok = map_to_sphere(new_point_2D, new_point_3D);

            //         if (new_point_ok)
            //         {
            //             Vec3f axis      = (last_point_3D_ % new_point_3D);
            //             float cos_angle = (last_point_3D_ | new_point_3D);

            //             if (fabs(cos_angle) < 1.0)
            //             {
            //                 float angle = 2.0*acos(cos_angle);
            //                 //rotate(axis, angle);
            //                 Transformation mv_tr = Transformation::retrieve_gl();
            //                 mv_tr.translation_.fill(0);
            //                 Transformation tr(angle, Vector3f(axis[0],axis[1],axis[2]));

            //                 transformations_[currIndex_] = mv_tr.inverse() * tr * mv_tr * transformations_[currIndex_];
            //             }
            //         }
            //     }
            // }
            // // translation
            // else if (button_down_[1])
            // {
            //     float dx = x - last_point_2D_[0];
            //     float dy = y - last_point_2D_[1];

            //     float z = - ((modelview_matrix_[ 2]*center_[0] +
            //                   modelview_matrix_[ 6]*center_[1] +
            //                   modelview_matrix_[10]*center_[2] +
            //                   modelview_matrix_[14]) /
            //                  (modelview_matrix_[ 3]*center_[0] +
            //                   modelview_matrix_[ 7]*center_[1] +
            //                   modelview_matrix_[11]*center_[2] +
            //                   modelview_matrix_[15]));

            //     float aspect = (float)width_ / (float)height_;
            //     float up     = tan(fovy_/2.0f*M_PI/180.f) * near_;
            //     float right  = aspect*up;

            //     Transformation mv_tr = Transformation::retrieve_gl();
            //     Transformation tr(2.0*dx/width_*right/near_*z, -2.0*dy/height_*up/near_*z, 0.0f);
            //     transformations_[currIndex_] = mv_tr.inverse() * tr * mv_tr * transformations_[currIndex_];
            // }


            // // remeber points
            // last_point_2D_ = Vec2i(x, y);
            // last_point_ok_ = map_to_sphere(last_point_2D_, last_point_3D_);

            // glutPostRedisplay();
            break;
        }
    }
}

void PrimoMeshViewer::mouse(int button, int state, int x, int y)
{
    if( glutGetModifiers() & GLUT_ACTIVE_SHIFT ){
		viewMode_ = EViewMode::MOVE;
		// printf("View Mode: Move\n");
		if(state == GLUT_UP){
			// release button, each mode does different thing
			switch(selectMode_){
				case ESelectMode::DYNAMIC:{
					// 1. ray cast allfaces
					// 2. add to STATIC/DYNAMIC faces based on selectMode_
					// 3. update face STATIC/DYNAMIC indices for drawing
				}
				case ESelectMode::STATIC:{
					// 1. ray cast allfaces
					// 2. add to STATIC/DYNAMIC faces based on selectMode_
					// 3. update face STATIC/DYNAMIC indices for drawing
					raycast_faces(x, y);
					break;
				}
				case ESelectMode::NONE:{
					// the dynamic faces have been transformed by motion(), minimize all optimizedFaces
					// #TODO[ZJW][QYZ]: minimize all optimizedFaces
					break;
				}
				default:
					assert(false);
					break;
			}
		}
	}
    else{
		viewMode_ = EViewMode::VIEW;
		// printf("View Mode: View\n");
	}

	GlutExaminer::mouse(button, state, x, y);
}

void PrimoMeshViewer::setup_prisms(EPrismExtrudeMode PrismExtrudeMode /*= EPrismExtrudeMode::FACE_NORMAL*/)
{
	for (Mesh::FaceIter f_iter = mesh_.faces_begin(); f_iter!= mesh_.faces_end(); f_iter++)
	{
		Mesh::FaceHalfedgeCWIter fh_cwit = mesh_.fh_cwbegin(*f_iter);
		for (; fh_cwit.is_valid(); fh_cwit++)
		{
			switch (PrismExtrudeMode)
			{
			case EPrismExtrudeMode::VERT_NORMAL:
			{
				PrismProperty prop;
				Mesh::VertexHandle v0 = mesh_.from_vertex_handle(*fh_cwit);
				Mesh::VertexHandle v1 = mesh_.to_vertex_handle(*fh_cwit);

				Mesh::Normal n0 = mesh_.normal(v0);
				Mesh::Normal n1 = mesh_.normal(v1);

				prop.FromVertPrismDir = n0;
				prop.ToVertPrimsDir = n1;
				prop.FromVertPrismSize = prop.ToVertPrismSize = prismHeight_;
				mesh_.property(P_PrismProperty, *fh_cwit) = prop;
			}
				break;
			case EPrismExtrudeMode::FACE_NORMAL:
			{
				PrismProperty prop;
				prop.FromVertPrismDir = mesh_.normal(*f_iter);
				prop.ToVertPrimsDir = mesh_.normal(*f_iter);
				prop.FromVertPrismSize = prismHeight_;
				prop.ToVertPrismSize = prismHeight_;
				mesh_.property(P_PrismProperty, *fh_cwit) = prop;
			}
				break;
			case EPrismExtrudeMode::CUSTOM:
			{
				assert(false);
			}
				break;
			default:
				break;
			}
		}
	}
}

void PrimoMeshViewer::manipulate(Mesh::VertexHandle vh_, Mesh::Point target_location)
{

}

void PrimoMeshViewer::local_optimize(int iterations)
{
	//TODO: Randomly sample one face
	Mesh::FaceHandle fh;
	//TODO: optimize for that face
	local_optimize_face(fh);
}

void PrimoMeshViewer::local_optimize_face(Mesh::FaceHandle _fh)
{
	//TODO: optimize for this face
	// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
	Eigen::Matrix3f A;
	Eigen::Vector3f b;

	
	struct EdgePrismData {
		float weight_ij;
		Vec3f centroid_ij;
		Vec3f centroid_ji;
		Vec3f f_ij[4]; // let's store this number as 00,01,10,11
		Vec3f f_ji[4]; // let's store this number as 00,01,10,11
	};

	std::vector<EdgePrismData> f_ij_data;
	Eigen::Matrix4f N; // TODO: Make sure this is zeroed
	Eigen::Matrix3f S; // TODO: Make sure this is zeroed
	Vec3f centroid_i(0,0,0);
	Vec3f centroid_star(0,0,0);
	float weight_sum = 0;
	for (Mesh::FaceHalfedgeCCWIter fhe_ccwiter = mesh_.fh_ccwbegin(_fh); fhe_ccwiter.is_valid(); fhe_ccwiter++)
	{
		// Grab the data to construct the face 
		Mesh::HalfedgeHandle he_ji = mesh_.opposite_halfedge_handle(*fhe_ccwiter);
		Mesh::HalfedgeHandle he_ij = *fhe_ccwiter;
		PrismProperty P_ij = mesh_.property(P_PrismProperty, he_ij);
		PrismProperty P_ji = mesh_.property(P_PrismProperty, he_ji);
		// A simple illustration of how the prism is stored
		/*
		10---------11 ^       10------------11   ^
		| f_ij     |  |       |  f_ji       |    |
		f-he_ij-->to  |       to<--he_ji---from  |
		00---------01 normal  00------------01   normal
		*/
		EdgePrismData Data;
		// Calc face weight
		float edge_len = mesh_.calc_edge_length(*fhe_ccwiter);
		float area_face_i = calc_face_area(_fh); // TODO: calc this
		Mesh::FaceHandle fh_j = mesh_.opposite_face_handle(*fhe_ccwiter);
		float area_face_j = calc_face_area(fh_j); // TODO: calc this
		Data.weight_ij = edge_len / (area_face_i + area_face_j);
		// Calc f_ij
		Data.f_ij[0] = P_ij.calc_f_uv(0, 0, true);
		Data.f_ij[1] = P_ij.calc_f_uv(0, 1, true);
		Data.f_ij[2] = P_ij.calc_f_uv(1, 0, true);
		Data.f_ij[3] = P_ij.calc_f_uv(1, 1, true);
		Data.centroid_ij = (Data.f_ij[0] + Data.f_ij[1] + Data.f_ij[2] + Data.f_ij[3]) * 0.25f;
		// Calc f_ji
		Data.f_ji[0] = P_ji.calc_f_uv(0, 0, false);
		Data.f_ji[1] = P_ji.calc_f_uv(0, 1, false);
		Data.f_ji[2] = P_ji.calc_f_uv(1, 0, false);
		Data.f_ji[3] = P_ji.calc_f_uv(1, 1, false);
		Data.centroid_ij = (Data.f_ji[0] + Data.f_ij[1] + Data.f_ij[2] + Data.f_ji[3]) * 0.25f;
		// I am pretty sure the above can be done with a for loop but not now.
		
		// Calculate centroids
		centroid_i += Data.centroid_ij * Data.weight_ij;
		centroid_star += Data.centroid_ij * Data.weight_ij;
		weight_sum += Data.weight_ij;

		float root_2s[4] = {1, 0.5, 0.5, 0.25}; // TODO : his guy should be global later.
		// Calculate S value for edge
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				// Sum over ijkl 
				float S_ijkl = 0;
				for (int ij = 0; ij < 4; ij++)
				{
					for (int kl = 0; kl < 4; kl++)
					{
						int ijkl_dist = ((~ij) & kl);
						S_ijkl += (Data.f_ij[ij] - Data.centroid_ij)[i] * (Data.f_ji[kl] - Data.centroid_ji)[j] * root_2s[ijkl_dist];
					}
				}
				S(i, j) += S_ijkl / 9.f * Data.weight_ij;
			}
		}
	}
	centroid_i /= weight_sum;
	centroid_star /= weight_sum;

	// Now we have all the S values, time to construct N
	/*
	N = 
	| Sxx + Syy + Szz		Syz - Szy			Szx - Sxz			Sxy - Syx		 |
	| Syz - Szy				Sxx - Syy - Szz		Sxy + Syx			Szx + Sxz		 |
	| Szx - Sxz				Sxy + Syx			-Sxx + Syy - Szz	Syz + Szy		 |
	| Sxy - Syx				Szx + Sxz			Syz + Szy			-Sxx - Syy + Szz |
	*/
	const int x = 0;
	const int y = 1;
	const int z = 2;
	N(0, 0) = S(x, x) + S(y, y) + S(z, z);
	N(0, 1) = S(y, z) - S(z, y);
	N(0, 2) = S(z, x) - S(x, z);
	N(0, 3) = S(x, y) - S(y, x);

	N(1, 0) = S(y, z) - S(z, y);
	N(1, 1) = S(x, x) - S(y, y) - S(z, z);
	N(1, 2) = S(x, y) + S(y, x);
	N(1, 3) = S(z, x) + S(x, z);

	N(2, 0) = S(z, x) - S(x, z);
	N(2, 1) = S(x, y) + S(y, x);
	N(2, 2) = -S(x, x) + S(y, y) - S(z, z);
	N(2, 3) = S(y, z) + S(z, y);

	N(3, 0) = S(x, y) - S(y, x);
	N(3, 1) = S(z, x) + S(x, z);
	N(3, 2) = S(y, z) + S(z, y);
	N(3, 3) = -S(x, x) - S(y, y) - S(z, z);
	// I must say, I hate writing these stuff up there. Hop for no bug

	// Now time tofind the largest eigen vector of N
	//https://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
	Eigen::EigenSolver<Eigen::Matrix4f> es;
	es.compute(N, true);
	auto eigenvectors = es.eigenvectors();
	auto eigenvalues = es.eigenvalues();
	std::cout << "We have " << es.eigenvectors().size() << " eigen vectors" << std::endl;
	
	// Now if we can find that max eigen value -> eigen vector, we get the rotation
	for (int col_i = 0; col_i < eigenvectors.cols(); col_i++)
	{
		auto eigenvector = eigenvectors.col(col_i);
	}

	// Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

	// Now we have th rotation Ri, just need to compute ti with the two centroids.
	// Wish I have a rotation class, I don't want to do that ,Ask Zejian Tomorrow, need to sleep
}

void PrimoMeshViewer::global_optimize_all_faces(int iterations)
{

}

float PrimoMeshViewer::calc_face_area(Mesh::FaceHandle _fh) const
{
// TODO: Implement 
	return 1.0f;
}

float PrimoMeshViewer::get_average_vertex_distance(const Mesh& _mesh) const
{
	float accDist = 0;
    int accCount = 0;

    Mesh::ConstHalfedgeIter he_it = _mesh.halfedges_begin();
    for (; he_it != _mesh.halfedges_end(); ++he_it) {
        OpenMesh::Vec3f p = _mesh.point(_mesh.from_vertex_handle(*he_it));
        OpenMesh::Vec3f q = _mesh.point(_mesh.to_vertex_handle(*he_it));
        float edgeLength = sqrt((p - q) | (p - q));
        accDist += edgeLength;
        accCount++;
    }

    if (accCount > 0)
        return accDist / float(accCount);
    else
        return 0;
}
void PrimoMeshViewer::get_allFace_handles(std::vector<OpenMesh::FaceHandle> &face_handles)
{
	Mesh::ConstFaceIter        f_it(mesh_.faces_sbegin()), 
                               f_end(mesh_.faces_end());
    Mesh::ConstFaceVertexIter  fv_it;

    face_handles.clear();
    face_handles.reserve(mesh_.n_faces());

    for (; f_it!=f_end; ++f_it)
    {
      face_handles.push_back(*f_it);
    }

}
void PrimoMeshViewer::build_allFace_BVH()
{
	allFaces_BVH_ = nanort::BVHAccel<float>();
	nanort::TriangleMesh<float> triangle_mesh(
      (const float *)mesh_.points(), indices_.data(), sizeof(float) * 3);
    nanort::TriangleSAHPred<float> triangle_pred(
      (const float *)mesh_.points(), indices_.data(), sizeof(float) * 3);
    nanort::BVHBuildOptions<float> build_options;  // Use default option
    if (!allFaces_BVH_.Build(mesh_.n_faces(), triangle_mesh, triangle_pred, build_options)) {
        printf("\t[Build Target BVH]: build BVH failed\n");
        assert(false);
    }
    // print bvh info
    nanort::BVHBuildStatistics stats = allFaces_BVH_.GetStatistics();
    printf(
      "[BVH statistics]: %d leaf nodes, %d branch nodes, max tree depth %d\n",
      stats.num_leaf_nodes, stats.num_branch_nodes, stats.max_tree_depth);
}
void PrimoMeshViewer::update_1typeface_indices(const std::vector<OpenMesh::FaceHandle>& face_handles, 
										std::vector<unsigned int>& indices_array){

    Mesh::ConstFaceVertexIter  fv_it;
    indices_array.clear();
    indices_array.reserve(face_handles.size()*3);
    for (size_t i = 0; i < face_handles.size(); ++i)
    {
        Mesh::FaceHandle fh = face_handles[i];
        indices_array.push_back((*(fv_it=mesh_.cfv_iter(fh))).idx());
        indices_array.push_back((*(++fv_it)).idx());
        indices_array.push_back((*(++fv_it)).idx());
    }
}
void PrimoMeshViewer::raycast_faces(int mouse_x, int mouse_y){
	// 1. ray cast allfaces
	// 1.1 find origin
	//  http://antongerdelan.net/opengl/raycasting.html
	/*  Step 1: 3d Normalised Device Coordinates, range [-1:1, -1:1, -1:1]
		The next step is to transform it into 3d normalised device coordinates. 
		This should be in the ranges of x [-1:1] y [-1:1] and z [-1:1]. 
		We have an x and y already, so we scale their range, and reverse the direction of y.
	*/
	float x = (2.0f * mouse_x) / width_ - 1.0f;
	float y = 1.0f - (2.0f * mouse_y) / height_;
	float z = 1.0f;
	Eigen::Vector3f ray_nds(x, y, z);
	/*  Step 2: 4d Homogeneous Clip Coordinates, range [-1:1, -1:1, -1:1, -1:1]
	    We want our ray's z to point forwards - this is usually the negative z direction in OpenGL style. 
		We can add a w, just so that we have a 4d vector.
		we do not need to reverse perspective division here because this is a ray with no intrinsic depth. 
		We would do that only in the special case of points, for certain special effects.
	*/
	Eigen::Vector4f ray_clip(ray_nds.x(), ray_nds.y(), -1.0, 1.0);
	/*  Step 3: 4d Eye (Camera) Coordinates, range [-x:x, -y:y, -z:z, -w:w]
	    Normally, to get into clip space from eye space we multiply the vector by a projection matrix. 
		We can go backwards by multiplying by the inverse of this matrix.
	*/
	Eigen::Matrix4f proMatrix;
	proMatrix << projection_matrix_[0], projection_matrix_[4], projection_matrix_[8],  projection_matrix_[12],
	             projection_matrix_[1], projection_matrix_[5], projection_matrix_[9],  projection_matrix_[13],
			     projection_matrix_[2], projection_matrix_[6], projection_matrix_[10], projection_matrix_[14],
			     projection_matrix_[3], projection_matrix_[7], projection_matrix_[11], projection_matrix_[15];
	Eigen::Vector4f ray_eye = proMatrix.inverse() * ray_clip;
	/*  Now, we only needed to un-project the x,y part, so let's manually set the z,w part to mean "forwards, and not a point".
	*/
	ray_eye = Eigen::Vector4f(ray_eye.x(), ray_eye.y(), -1.0, 0.0);
	/*  Step 4: 4d World Coordinates, range [-x:x, -y:y, -z:z, -w:w]
	    Same again, to go back another step in the transformation pipeline. 
		Remember that we manually specified a -1 for the z component, which means that our ray isn't normalised. 
		We should do this before we use it.
		This should balance the up-and-down, left-and-right, and forwards components for us. 
		So, assuming our camera is looking directly along the -Z world axis, 
		we should get [0,0,-1] when the mouse is in the centre of the screen, 
		and less significant z values when the mouse moves around the screen. 
		This will depend on the aspect ratio, and field-of-view defined in the view and projection matrices. 
		We now have a ray that we can compare with surfaces in world space.
	*/
	Eigen::Matrix4f viewMatrix;
	viewMatrix << modelview_matrix_[0], modelview_matrix_[4], modelview_matrix_[8],  modelview_matrix_[12],
	              modelview_matrix_[1], modelview_matrix_[5], modelview_matrix_[9],  modelview_matrix_[13],
			      modelview_matrix_[2], modelview_matrix_[6], modelview_matrix_[10], modelview_matrix_[14],
			      modelview_matrix_[3], modelview_matrix_[7], modelview_matrix_[11], modelview_matrix_[15];
	Eigen::Matrix4f viewToWorld(viewMatrix.inverse());
	Eigen::Vector4f ray_dir4 = (viewToWorld * ray_eye);
	Eigen::Vector3f ray_ori3(viewToWorld(0, 3), viewToWorld(1, 3), viewToWorld(2, 3));
	Eigen::Vector3f ray_dir3(ray_dir4.x(),ray_dir4.y(),ray_dir4.z());
	// don't forget to normalise the vector at some point
	ray_dir3.normalize();
	// 1.2 set nanort intersection and traverse bvh
	nanort::TriangleIntersector<float, nanort::TriangleIntersection<float>>
      triangle_intersecter((const float *)mesh_.points(),
	  indices_.data(), sizeof(float) * 3);
	static const float tFar = 1e30;
	nanort::Ray<float> ray;
    ray.min_t = 0.0f;
    ray.max_t = tFar;
    ray.org[0] = ray_ori3[0];
    ray.org[1] = ray_ori3[1];
    ray.org[2] = ray_ori3[2];

    ray.dir[0] = ray_dir3[0];
    ray.dir[1] = ray_dir3[1];
    ray.dir[2] = ray_dir3[2];
    nanort::TriangleIntersection<float> isect;
    bool hit = allFaces_BVH_.Traverse(ray, triangle_intersecter, &isect);
	if(!hit) return;
	// 2. add to STATIC/DYNAMIC faces based on selectMode_
	auto hit_faceIter = faceIdx_to_selType.find(isect.prim_id);
	assert(hit_faceIter != faceIdx_to_selType.end());
	bool needUpdateStatic = false;
	bool needUpdateDynamic = false;
	unsigned int hit_faceId = hit_faceIter->first;
	ESelectMode hit_faceType = hit_faceIter->second;
	if(selectMode_ == ESelectMode::STATIC){
		if(hit_faceType == ESelectMode::DYNAMIC){
			delete_faceHandle(hit_faceId, dynamicFaceHandles_);
			needUpdateDynamic = true;
			needUpdateStatic = true;
			staticFaceHandles_.push_back(mesh_.face_handle(hit_faceId));
			faceIdx_to_selType[hit_faceId] = selectMode_;
		}else if(hit_faceType == ESelectMode::NONE){
			delete_faceHandle(hit_faceId, optimizedFaceHandles_);
			needUpdateStatic = true;
			staticFaceHandles_.push_back(mesh_.face_handle(hit_faceId));
			faceIdx_to_selType[hit_faceId] = selectMode_;
		}else{
			delete_faceHandle(hit_faceId, staticFaceHandles_);
			needUpdateStatic = true;
			optimizedFaceHandles_.push_back(mesh_.face_handle(hit_faceId));
			faceIdx_to_selType[hit_faceId] = ESelectMode::NONE;
		}
	}else if(selectMode_ == ESelectMode::DYNAMIC){
		if(hit_faceType == ESelectMode::STATIC){
			delete_faceHandle(hit_faceId, staticFaceHandles_);
			needUpdateStatic = true;
			needUpdateDynamic = true;
			dynamicFaceHandles_.push_back(mesh_.face_handle(hit_faceId));
			faceIdx_to_selType[hit_faceId] = selectMode_;
		}else if(hit_faceType == ESelectMode::NONE){
			delete_faceHandle(hit_faceId, optimizedFaceHandles_);
			needUpdateDynamic = true;
			dynamicFaceHandles_.push_back(mesh_.face_handle(hit_faceId));
			faceIdx_to_selType[hit_faceId] = selectMode_;
		}
		else{
			delete_faceHandle(hit_faceId, dynamicFaceHandles_);
			needUpdateDynamic = true;
			optimizedFaceHandles_.push_back(mesh_.face_handle(hit_faceId));
			faceIdx_to_selType[hit_faceId] = ESelectMode::NONE;
		}
	}else{
		// assertion: raycast_faces could not be called in ESelectMode::NONE mode
		assert(selectMode_ != ESelectMode::NONE); 
	}
	
	// 3. update face STATIC/DYNAMIC optimized indices for drawing(That's why they should be update immediatly if necessary)
	if(needUpdateStatic){
		update_1typeface_indices(staticFaceHandles_, staticVertexIndices_);
	}
	if(needUpdateDynamic){
		update_1typeface_indices(dynamicFaceHandles_, dynamicVertexIndices_);
	}
	if((needUpdateStatic && !needUpdateDynamic) || (!needUpdateStatic && needUpdateDynamic)){
		// need to update optimized when only one type of STATIC/DYNAMIC is changed.
		update_1typeface_indices(optimizedFaceHandles_, optimizedVertexIndices_);
	}
	// assertion for debug face number and indices number
	assert(dynamicFaceHandles_.size() * 3 == dynamicVertexIndices_.size());
	assert(staticFaceHandles_.size() * 3 == staticVertexIndices_.size());
	assert(optimizedFaceHandles_.size() * 3 == optimizedVertexIndices_.size());
	assert(dynamicFaceHandles_.size() + staticFaceHandles_.size() + optimizedFaceHandles_.size()
			== mesh_.n_faces());

}
void PrimoMeshViewer::delete_faceHandle(unsigned int faceId, std::vector<OpenMesh::FaceHandle>& face_handles){
	for(auto it = face_handles.begin(); it != face_handles.end(); ++it){
		if(it->idx() == faceId){
			// remove this fh
			face_handles.erase(it);
			return;
		}
	}
	// this funtion is only used by raycast, and fh must be in face_handles where fh.idx()==faceId
	// this assert is just for debug
	assert(false);
}
void PrimoMeshViewer::draw_prisms(const std::vector<OpenMesh::FaceHandle> face_handles) const{
	// each triangle face has 6 prism vertices and 9 edges
	for (const OpenMesh::FaceHandle& fh : face_handles){
		Mesh::ConstFaceHalfedgeCWIter fh_cwit = mesh_.cfh_cwbegin(fh);
		Vec3f pv[6];// 6 vertices of prism
		for (int i = 0; fh_cwit.is_valid(); ++fh_cwit, ++i){
			assert(i < 3);
			const PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_cwit);
			const Vec3f& from_v = mesh_.point(mesh_.from_vertex_handle(*fh_cwit));
			pv[i]     = from_v + prop.FromVertPrismDir * prop.FromVertPrismSize;
			pv[i + 3] = from_v - prop.FromVertPrismDir * prop.FromVertPrismSize;
		}
		// have got all six vertices of prism, draw 9 edges
		// 01, 12, 02, 34, 45, 35, 03, 14, 25
		static const int pv1i[9] = {0, 1, 0, 3, 4, 3, 0, 1, 2};
		static const int pv2i[9] = {1, 2, 2, 3, 5, 5, 3, 4, 5};
		for(int i = 0; i < 9; ++i){
			glVertex3f(pv[pv1i[i]][0], pv[pv1i[i]][1], pv[pv1i[i]][2]);
			glVertex3f(pv[pv2i[i]][0], pv[pv2i[i]][1], pv[pv2i[i]][2]);
		}
	}
}
