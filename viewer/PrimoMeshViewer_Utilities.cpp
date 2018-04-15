#include "PrimoMeshViewer.h"
#include "parallel.hh"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <glm/gtc/quaternion.hpp>
#include <unordered_set>

const LinearColor LinearColor::RED(1.f,0.f,0.f);
const LinearColor LinearColor::BLUE(0.f,0.f,1.0f);
const LinearColor LinearColor::GREEN(0.f,1.f,0.f);
const LinearColor LinearColor::BLACK(0.f,0.f,0.f);
const LinearColor LinearColor::GREY(0.5f,0.5f,0.5f);
const LinearColor LinearColor::WHITE(1.f,1.f,1.f);
const LinearColor LinearColor::YELLOW(1.f,1.f,0.f);
const LinearColor LinearColor::CYAN(0.f,1.f,1.f);
const LinearColor LinearColor::MAGENTA(1.f,0.f,1.f);
const LinearColor LinearColor::ORANGE(1.f,0.5f,0.f);
const LinearColor LinearColor::PURPLE(0.5f, 0.f,1.f);
const LinearColor LinearColor::TURQUOISE(0.f, 0.5f, 0.5f);
const LinearColor LinearColor::SILVER(0.8f,0.8f,0.8f);
const LinearColor LinearColor::EMERALD(0.2f,0.8f,0.4f);

float PrimoMeshViewer::calc_face_area(Mesh::FaceHandle _fh) const
{
	// TODO: Implement 
	if (!_fh.is_valid())
	{
		std::cout << "Calc face area received invalid face handle, returning 1" << std::endl;
		return 1.0f;
	}
	Mesh::FaceHalfedgeCCWIter fh_ccwit = mesh_.cfh_ccwbegin(_fh);
	Mesh::VertexHandle v0 = mesh_.from_vertex_handle(*fh_ccwit);
	Mesh::VertexHandle v1 = mesh_.to_vertex_handle(*fh_ccwit);
	fh_ccwit++;
	Mesh::VertexHandle v2 = mesh_.to_vertex_handle(*fh_ccwit);

	Vec3f v0v1 = mesh_.point(v1) - mesh_.point(v0);
	Vec3f v0v2 = mesh_.point(v2) - mesh_.point(v0);

	float area = (v0v1 % v0v2).norm() * 0.5f;
	return area;
}

void PrimoMeshViewer::draw_debug_lines()
{
	for (Transformation& t : g_debug_transformations_to_draw_local_optimization)
	{
		add_debug_coordinate(t, 100);
	}

	for (Arrow& a : g_debug_arrows_to_draw_local_optimizations)
	{
		add_debug_arrow(a.from, a.to, a.color, a.arrow_size);
	}

	for (DebugLine& line : debug_lines_)
	{
		glLineWidth(line.width_);
		glColor4fv(line.color_.rgba_);
		glBegin(GL_LINES);
		glVertex3f(line.from_[0], line.from_[1], line.from_[2]);
		glVertex3f(line.to_[0], line.to_[1], line.to_[2]);
		glEnd();
	}
	// flush the debug line buffer
	debug_lines_.clear();
}


void PrimoMeshViewer::add_debug_arrow(const Vector3d& from,const Vector3d& to, LinearColor color, double arrow_size)
{
	DebugLine line(from, to, 1.0f, color);
	debug_lines_.emplace_back(line);
	Vector3d dir = (to - from).normalize();
	Vector3d up(0, 1, 0);
	Vector3d left = dir % up;
	if (length(left) != 1.0f)
	{
		dir.find_best_axis_vectors(up, left);
	}

	Vector3d origin(0, 0, 0);
	// Make a transform
	Transformation tm(left,up,dir,origin);
	
	float arrow_sqrt = sqrt(arrow_size);

	DebugLine arrow1(to, to + tm.transformVector(Vector3d(-arrow_sqrt , 0, -arrow_sqrt)), 1.0f, color);
	debug_lines_.emplace_back(arrow1);
	DebugLine arrow2(to, to + tm.transformVector(Vector3d(arrow_sqrt, 0, -arrow_sqrt)), 1.0f, color );
	debug_lines_.emplace_back(arrow2);


}

void PrimoMeshViewer::add_debug_coordinate(Transformation& world_transform, double size, Transformation base_transform)
{
	add_debug_arrow(world_transform.translation_, world_transform.translation_ + world_transform.x_axis() * size, LinearColor::RED, 0.1f * size);
	add_debug_arrow(world_transform.translation_, world_transform.translation_ + world_transform.y_axis() * size, LinearColor::GREEN, 0.1f * size);
	add_debug_arrow(world_transform.translation_, world_transform.translation_ + world_transform.z_axis() * size, LinearColor::BLUE, 0.1f * size);
}


void PrimoMeshViewer::add_debug_line(Vector3d& from, Vector3d& to, LinearColor color, double width /*= 1.0f*/)
{
	DebugLine line(from, to, width, color);
	debug_lines_.emplace_back(line);
}


void PrimoMeshViewer::print_quaternion(Eigen::Quaternion<double>& Q)
{
	float half_theta = acos(Q.w());
	float sin_half_theta = sin(half_theta);
	float axis_x = Q.x() / sin_half_theta;
	float axis_y = Q.y() / sin_half_theta;
	float axis_z = Q.z() / sin_half_theta;

	std::cout << "== Begin Quaternion Log == " << std::endl;
	std::cout << "w = " << Q.w() << std::endl;
	std::cout << "x = " << Q.x() << std::endl;
	std::cout << "y = " << Q.y() << std::endl;
	std::cout << "z = " << Q.z() << std::endl;
	std::cout << "theta = " << half_theta * 2 << std::endl;
	std::cout << "around axis = " << std::endl;
	std::cout << axis_x << std::endl;
	std::cout << axis_y << std::endl;
	std::cout << axis_z << std::endl;
	std::cout << "== End Quaternion Log == " << std::endl;


}
void PrimoMeshViewer::update_vertices_based_on_prisms()
{
	std::unordered_set<int> visited_vertices_idx;
	for(const OpenMesh::FaceHandle &fh: optimizedFaceHandles_){
		for (Mesh::FaceVertexIter fv_iter = mesh_.fv_begin(fh); fv_iter.is_valid(); ++fv_iter)
		{
			// check if this vertice is visited
			if(visited_vertices_idx.find(fv_iter->idx()) != visited_vertices_idx.end()){
				continue;
			}
			visited_vertices_idx.emplace(fv_iter->idx());
			
			Mesh::Point original_point = mesh_.point(*fv_iter);
			Vector3d new_point(0, 0, 0);
			float weight = 0;

			for (Mesh::VertexOHalfedgeCCWIter voh_ccwiter = mesh_.voh_ccwbegin(*fv_iter); voh_ccwiter.is_valid(); voh_ccwiter++)
			{
				if (mesh_.face_handle(*voh_ccwiter).is_valid()) // Make sure this half edge has a face
				{
					PrismProperty& voh_prop = mesh_.property(P_PrismProperty, *voh_ccwiter);
					new_point += voh_prop.TargetPosFrom();
					weight += 1.0f;

					#ifndef NDEBUG
					Arrow arrow(
						Vector3d(original_point),
						Vector3d(voh_prop.TargetPosFrom()),
						LinearColor::RED,
						3.0f
					);
					g_debug_arrows_to_draw_local_optimizations.emplace_back(arrow);
					#endif
				}
			}
			for (Mesh::VertexIHalfedgeCCWIter vih_ccwiter = mesh_.vih_ccwbegin(*fv_iter); vih_ccwiter.is_valid(); vih_ccwiter++)
			{
				if (mesh_.face_handle(*vih_ccwiter).is_valid())
				{
					PrismProperty& voh_prop = mesh_.property(P_PrismProperty, *vih_ccwiter);
					new_point += voh_prop.TargetPosTo();
					weight += 1.0f;

					#ifndef NDEBUG
					Arrow arrow(
						Vector3d(original_point),
						Vector3d(voh_prop.TargetPosTo()),
						LinearColor::GREEN,
						3.0f
					);
					g_debug_arrows_to_draw_local_optimizations.emplace_back(arrow);
					#endif
				}
			}
			new_point /= weight;

			mesh_.point(*fv_iter) = Vec3f(new_point[0], new_point[1], new_point[2]);

			#ifndef NDEBUG
			Arrow arrow(
				Vector3d(original_point),
				Vector3d(new_point),
				LinearColor::BLUE,
				3.0f
			);
			g_debug_arrows_to_draw_local_optimizations.emplace_back(arrow);
			#endif
		}
	}
}
float PrimoMeshViewer::E(const std::vector<OpenMesh::FaceHandle> &face_handles) const{
	//#TODO[ZJW]: could parallel
	//AtomicFloat E(0.0f);
	float E = 0.0f;
    // std::unordered_set<int> he_id_set;
    //////////////////////////////////////////////////////////////////////////////
    //std::cout<< "B:\n" <<  B <<std::endl;
    //std::cout<< "-A^T:\n" << negA_T << std::endl;
    // double plus_equal_duration = 0.0;
    //////////////////////////////////////////////////////////////////////////////
    //ParallelFor(0, (int)face_handles.size(), [&](int i){
	for(int i = 0; i < face_handles.size(); ++i){
        // iterate all faces
        const OpenMesh::FaceHandle &fh_i = face_handles[i];
        const int f_i_id = fh_i.idx();
        for(Mesh::ConstFaceHalfedgeIter fhe_it = mesh_.cfh_iter(fh_i); fhe_it.is_valid(); ++fhe_it){
            // Grab the opposite half edge and face
            const Mesh::HalfedgeHandle he_i = *fhe_it;
            Mesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
		    Mesh::FaceHandle fh_j = mesh_.opposite_face_handle(*fhe_it);
            // if he_i is a boundary halfedge, there is no opposite prism face,
            // which means that it has no contribution to the total energy E = Sum(w_ij * E_ij)
            // we could skip this half edge he_i now
            if (!he_j.is_valid() || mesh_.is_boundary(he_i) || !fh_j.is_valid()){
                #ifndef NDEBUG
			    std::cout << "halfedge's opposite doesnot exist, idx = "<< he_i.idx() << std::endl;
                #endif
			    continue;
		    }

            // for same pair half edge Eij = Eji, we only want to calculate it once
            // (he_i, he_j) == (he_j, he_i)
            const int he_i_id = he_i.idx();
            const int he_j_id = he_j.idx();
            
            // get the f^ij_[0/1][0/1] in the PriMo equation
            const int f_j_id = fh_j.idx();
            const PrismProperty * const P_i = &(mesh_.property(P_PrismProperty, he_i));
            const PrismProperty * const P_j = &(mesh_.property(P_PrismProperty, he_j));
            const float w_ij = P_i->weight_ij;
            // assert for debug, opposite half edges should have same edge weight

			// get (fij - fji)[2][2]
            assert(fabs(w_ij - P_j->weight_ij) < FLT_EPSILON);
			const OpenMesh::Vec3f *f_ij[2][2] = {
				{&(P_i->FromVertPrismDown), &(P_i->FromVertPrismUp)},
				{&(P_i->ToVertPrismDown), &(P_i->ToVertPrismUp)}
			};
    		const OpenMesh::Vec3f *f_ji[2][2] = {      
				{&(P_j->ToVertPrismDown), &(P_j->ToVertPrismUp)},
                {&(P_j->FromVertPrismDown), &(P_j->FromVertPrismUp)}
    		};
			const OpenMesh::Vec3f fij_m_fji[2][2] = {
				{*f_ij[0][0] - *f_ji[0][0], *f_ij[0][1] - *f_ji[0][1]},
				{*f_ij[1][0] - *f_ji[1][0], *f_ij[1][1] - *f_ji[1][1]}
			};
			// repeat again, but YOLO.
			static const int uv_integrate_id[10][4] = {
            	{0, 0, 0, 0},
            	{0, 0, 0, 1},
            	{0, 0, 1, 0},
            	{0, 0, 1, 1},
            	{0, 1, 0, 1},
            	{0, 1, 1, 0},
            	{0, 1, 1, 1},
            	{1, 0, 1, 0},
            	{1, 0, 1, 1},
            	{1, 1, 1, 1}
        	};
			static const float one_ninth = 1.0f / 9.0f;
        	static const float weight[10] = {
                one_ninth,
                one_ninth,
                one_ninth,
                0.5f * one_ninth,
                one_ninth,
                0.5f * one_ninth,
                one_ninth,
                one_ninth,
                one_ninth,
                one_ninth
        	};
			float Eij = 0.0f;
			for(int cid = 0; cid < 10; ++cid){
            	const OpenMesh::Vec3f &a = fij_m_fji[ uv_integrate_id[cid][0] ][ uv_integrate_id[cid][1] ];
            	const OpenMesh::Vec3f &b = fij_m_fji[ uv_integrate_id[cid][2] ][ uv_integrate_id[cid][3] ];
            	Eij += OpenMesh::dot(a, b) * weight[cid];
        	}
			//E.Add(Eij * w_ij);
			E += Eij * w_ij;

		}
	}
	return E * 0.5f;
}
