#include "PrimoMeshViewer.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <glm/gtc/quaternion.hpp>
#include <unordered_set>
#include <string>


const std::string test_crease_file("../data/curve1.cpx");

PrimoMeshViewer::PrimoMeshViewer(const char* _title, int _width, int _height)
	: MeshViewer(_title, _width, _height)
{
	mesh_.request_vertex_colors();

	mesh_.add_property(P_PrismProperty);
	mesh_.add_property(P_globalPrism_intermediate);
	mesh_.add_property(P_collision);
	mesh_.add_property(P_faceBN);
	//mesh_.add_property(P_FaceTransformationCache);

	// default: optimized(white)
	optimizedFacesColor_[0] = 0.8f;
	optimizedFacesColor_[1] = 0.8f;
	optimizedFacesColor_[2] = 0.8f;
	
	// do not draw prisms at first
	drawPrisms_ = false;
	global_optimize_iterations_ = 25;
	printf("Gloabl Max Iteration: %d\n", global_optimize_iterations_);

	
	// select mode is STATIC at first
	// selectMode_ = ESelectMode::STATIC;
	//viewMode_ = EViewMode::VIEW;
	//printf("Select Mode: Static\n");
	
	// optimize mode is LOCAL at first
	optimizeMode_ = EOptimizeMode::GLOBAL;
	printf("Optimize Mode: Global\n");

	// do not draw debug info at first
	drawDebugInfo_ = false;
	printf("Draw Debug Info: false\n");

	bKey_space_is_move_ = true;
	folding_angle_ = 0.0f;
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

		// default: all faces are optimizable
		get_allFace_handles(optimizedFaceHandles_);
		//update_1typeface_indices(optimizedFaceHandles_, optimizedVertexIndices_);
		// for(const OpenMesh::FaceHandle& fh: optimizedFaceHandles_){
		// 	faceIdx_to_selType_[fh.idx()] = ESelectMode::OPTIMIZED;
		// }
		// and then, prisms are set up 
		setup_prisms(allFaceHandles_, EPrismExtrudeMode::VERT_NORMAL);
		setup_faceBN(allFaceHandles_);
		setup_collisionProperty();
		//update_1typeface_indices(allFaceHandles_,allVertexIndices_);
		// init the set of idx of opmizedFaces only for global optimization
		for(int i = 0; i <  optimizedFaceHandles_.size(); ++i){
			optimizedFaceIdx_2_i_[optimizedFaceHandles_[i].idx()] = i;
		}
		float initE = E(optimizedFaceHandles_);
		assert(fabs(initE) < FLT_EPSILON);

		// read crease pattern curve
		test_read_crease_pattern();
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

	// 
	Transformation tm;
	add_debug_coordinate(tm, 30.0f);

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
		for (const OpenMesh::FaceHandle& fh : allFaceHandles_)
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
		glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, &indices_[0]);

		// if(dynamicVertexIndices_.size()>0)
		// {
		// 	glColor3fv(dynamicFacesColor_);
		// 	glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(dynamicVertexIndices_.size()), GL_UNSIGNED_INT, &dynamicVertexIndices_[0]);
		// }
		// if(staticVertexIndices_.size() > 0)
		// {
		// 	glColor3fv(staticFacesColor_);
		// 	glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(staticVertexIndices_.size()), GL_UNSIGNED_INT, &staticVertexIndices_[0]);
		// }
		//
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisable(GL_COLOR_MATERIAL);
	}
	// draw mountain and valley edges
	float prev_line_width;
	glGetFloatv(GL_LINE_WIDTH, &prev_line_width);
	glLineWidth(3 * prev_line_width);
	for(const Crease &crease : creases_){
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
		crease.draw();
		glEnd();
		glDisable(GL_COLOR_MATERIAL);
	}
	glLineWidth(prev_line_width);
	if(drawPrisms_){
		// visualize prisms with wireframes
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
		// draw 3 types of prisms with different color
		//glColor3fv(dynamicFacesColor_);
		//draw_prisms(dynamicFaceHandles_);
		//glColor3fv(staticFacesColor_);
		//draw_prisms(staticFaceHandles_);
		glColor3fv(optimizedFacesColor_);
		draw_prisms(optimizedFaceHandles_);

		// overide the color 
		for(const Crease &crease : creases_){
			crease.draw_prisms(P_PrismProperty);
		}
		//
		glEnd();
		glDisable(GL_COLOR_MATERIAL);
	}
	if (drawDebugInfo_)
	{
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_LIGHTING);
		draw_debug_lines();
		glDisable(GL_COLOR_MATERIAL);
	}
	// draw dynamic faces rotation axis

}


void PrimoMeshViewer::keyboard(int key, int x, int y)
{
	// Let super handle us first
	switch (key)
	{
	case 'a':
	case 'A': 
	{
		// toggle if visualize prisms
		drawPrisms_ = !drawPrisms_;
		glutPostRedisplay();
	}
		break;
	case 'd':
	case 'D':
	{
		// toggle if visualize prisms
		drawDebugInfo_ = !drawDebugInfo_;
		glutPostRedisplay();
		printf("Draw Debug Info: %s\n",drawDebugInfo_ ? "true" : "false");
	}
	break;
	case '+':
	{
		// add prisms' height
		prismHeight_ += averageVertexDisance_ * 0.1f;
		// immediately update all prisms, should not use setup_prisms.
		update_prisms_height_uniform(allFaceHandles_, averageVertexDisance_ * 0.1f);
		printf("prismHeight: %f\n", prismHeight_);

		// following the PriMo demo, after changing the prisms' height, we should at once optimize all surface

		thread_pool_.emplace_back([&]() { optimize_faces(optimizedFaceHandles_, optimizedFaceIdx_2_i_, global_optimize_iterations_);});

		glutPostRedisplay();
	}
		break;
	case '-':
	{
		// minus prisms' height(keep prisms' height > 0)
		if(prismHeight_ - averageVertexDisance_ * 0.1f > FLT_EPSILON){
			prismHeight_ -= averageVertexDisance_ * 0.1f;
			// immediately update all prisms, should not use setup_prisms.
			update_prisms_height_uniform(allFaceHandles_, averageVertexDisance_ * -0.1f);
			// following the PriMo demo, after changing the prisms' height, we should at once optimize all surface
			thread_pool_.emplace_back([&]() { optimize_faces(optimizedFaceHandles_, optimizedFaceIdx_2_i_, global_optimize_iterations_);});
		}
		printf("prismHeight: %f\n", prismHeight_);

		
		// based on the new prism height.
		glutPostRedisplay();
	}
		break;
	case 'o':
	case 'O':
	{
		// switch optimization method (default: global)
		bool opIsLocal = (optimizeMode_ == EOptimizeMode::LOCAL);
		optimizeMode_ = (opIsLocal ? EOptimizeMode::GLOBAL : EOptimizeMode::LOCAL);
		printf("Optimize Mode: %s\n", opIsLocal ? "Global" : "Local");
	}
		break;
	case 'c':
	case 'C':{
		// read Dynamic Crease Configuration(DCC) file
		// including: 
		// crease index, 0/1 folding or not, prism height percentegate(0.2 measn 0.2 * default height)
		
		// Assertion: only load DCC file after triangulation
		assert(creases_.size() > 0);
		std::string dcc_filename;
		std::cout<<"Please type in the Dynamic Crease Configuration(DCC) file\n";
		std::cin>> dcc_filename;
		if(!read_dcc_file(dcc_filename)){
			std::cout<<"Read DCC file failed!\n";
			break;
		}
		//
	}
		break;
	case 'f':
	case 'F':{
		// forward folding
		folding_angle_ += 0.5;
		for(Crease &crease : creases_){
			crease.fold(0.5f, P_PrismProperty);
		}
		// thread_pool_.emplace_back([&]() { optimize_faces(optimizedFaceHandles_, optimizedFaceIdx_2_i_, global_optimize_iterations_);});

	}
		break;
	case 'b':
	case 'B':{
		// backward folding
		folding_angle_ -= 0.5;
		for(Crease &crease : creases_){
			crease.fold(-0.5f, P_PrismProperty);
		}
		// thread_pool_.emplace_back([&]() { optimize_faces(optimizedFaceHandles_, optimizedFaceIdx_2_i_, global_optimize_iterations_);});
	}
		break;
	default:
		GlutExaminer::keyboard(key, x, y);
		break;
	}
}

void PrimoMeshViewer::draw_prisms(const std::vector<OpenMesh::FaceHandle> &face_handles) const{
	// each triangle face has 6 prism vertices and 9 edges
	for (const OpenMesh::FaceHandle& fh : face_handles){
		Mesh::ConstFaceHalfedgeCWIter fh_cwit = mesh_.cfh_cwbegin(fh);
		const Vec3f* pv[6];// 6 vertices of prism
		for (int i = 0; fh_cwit.is_valid(); ++fh_cwit, ++i){
			assert(i < 3);
			const PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_cwit);
			// const Vec3f& from_v = mesh_.point(mesh_.from_vertex_handle(*fh_cwit));
			// pv[i]     = from_v + prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
			// pv[i + 3] = from_v - prop.FromVertPrismDir_DEPRECATED * prop.FromVertPrismSize_DEPRECATED;
			pv[i] = &(prop.FromVertPrismUp);
			pv[i + 3] = &(prop.FromVertPrismDown);
		}
		// have got all six vertices of prism, draw 9 edges
		// 01, 12, 02, 34, 45, 35, 03, 14, 25
		static const int pv1i[9] = {0, 1, 0, 3, 4, 3, 0, 1, 2};
		static const int pv2i[9] = {1, 2, 2, 4, 5, 5, 3, 4, 5};
		for(int i = 0; i < 9; ++i){
			glVertex3f((*pv[pv1i[i]])[0], (*pv[pv1i[i]])[1], (*pv[pv1i[i]])[2]);
			glVertex3f((*pv[pv2i[i]])[0], (*pv[pv2i[i]])[1], (*pv[pv2i[i]])[2]);
		}
	}
}

void PrimoMeshViewer::optimize_faces(const std::vector<OpenMesh::FaceHandle> &face_handles, 
										const std::unordered_map<int,int> &face_idx_2_i, const int max_iterations){
	// here max_iterations is for global optimization, for local we simply *100.
	if(optimizeMode_ == EOptimizeMode::LOCAL){
		local_optimize(face_handles,max_iterations * 100);
	}
	else{
		global_optimize_faces(face_handles, face_idx_2_i, max_iterations);
	}
}

