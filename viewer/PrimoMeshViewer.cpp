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
	// static(white)
	staticFacesColor_[0] = 1.0f;
	staticFacesColor_[1] = 1.0f;
	staticFacesColor_[2] = 1.0f;
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
		update_allFace_handles();
		// build bvh for all faces using nanort at first
		build_allFace_BVH();
		// and then, prisms are set up 
		setup_prisms(EPrismExtrudeMode::VERT_NORMAL);

		return true;
	}
	return false;
}

void PrimoMeshViewer::draw(const std::string& _draw_mode)
{
	MeshViewer::draw(_draw_mode);
	if(drawPrisms_){
		// visualize prisms with wireframes
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_LIGHTING);
		glColor3fv(dynamicFacesColor_);
		glBegin(GL_LINES);
		// each triangle face has 6 prism vertices and 9 edges
		for (Mesh::FaceIter f_iter = mesh_.faces_begin(); f_iter!= mesh_.faces_end(); ++f_iter){
			Mesh::FaceHalfedgeCWIter fh_cwit = mesh_.fh_cwbegin(*f_iter);
			Vec3f pv[6];// 6 vertices of prism
			for (int i = 0; fh_cwit.is_valid(); ++fh_cwit,++i){
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
		glutPostRedisplay();
	}
		break;
	case '-':
	{
		// minus prisms' height
		prismHeight_ -= averageVertexDisance_ * 0.1f;
		// immediately update all prisms
		setup_prisms(EPrismExtrudeMode::VERT_NORMAL);
		printf("prismHeight: %f\n", prismHeight_);
		glutPostRedisplay();
	}
		break;
	case '1':
	{
		// select mode changed to STATIC
		// which means that pressing shift + left mouse will select STATIC faces
		selectMode_ = ESelectMode::STATIC;
		printf("Select Mode: Static\n");

		// rebuild nanort bvh for all faces before selecting STATIC/DYNAMIC faces
		build_allFace_BVH();
	}
		break;
	case '2':
	{
		// select mode changed to DYNAMIC
		// which means that pressing shift + left mouse will select DYNAMIC faces
		selectMode_ = ESelectMode::DYNAMIC;
		printf("Select Mode: Dynamic\n");

		// rebuild nanort bvh for all faces before selecting STATIC/DYNAMIC faces
		build_allFace_BVH();
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
				// TODO[ZJW]: rotate or translate dynamicFaces when pressing shift + left/middle mouse button 
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
					// TODO[ZJW]: ray cast and give back a face handle, add to dynamic faces
					break;
				}
				case ESelectMode::STATIC:{
					// TODO[ZJW]: ray cast and give back a face handle, add to static faces
					break;
				}
				case ESelectMode::NONE:{
					// the dynamic faces have been transformed by motion(), minimize all optimizedFaces
					// TODO[ZJW][QYZ]: minimize all optimizedFaces
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
void PrimoMeshViewer::update_allFace_handles()
{
	Mesh::ConstFaceIter        f_it(mesh_.faces_sbegin()), 
                               f_end(mesh_.faces_end());
    Mesh::ConstFaceVertexIter  fv_it;

    allFaceHandles_.clear();
    allFaceHandles_.reserve(mesh_.n_faces());

    for (; f_it!=f_end; ++f_it)
    {
      allFaceHandles_.push_back(*f_it);
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
