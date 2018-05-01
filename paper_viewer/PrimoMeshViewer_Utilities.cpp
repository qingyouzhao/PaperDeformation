#include "PrimoMeshViewer.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <glm/gtc/quaternion.hpp>
#include <unordered_set>
#include <istream>
#include <fstream>
#include "BezierCurve.h"
// Triangulation stuff
#include "triangle.h"
#include "libigl/include/igl/delaunay_triangulation.h"
#include "libigl/include/igl/triangle/triangulate.h"

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
	float E = 0.0f;
    std::unordered_set<int> he_id_set;
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
            const bool he_i_in_set = (he_id_set.find(he_i_id) != he_id_set.end());	
            if(he_i_in_set){	
                #ifndef NDEBUG	
                const bool he_j_in_set = (he_id_set.find(he_j_id) != he_id_set.end());	
                // assert for debug, he pair should be both in set or neither in set 	
                assert(he_i_in_set && he_j_in_set);	
                #endif	
                continue;	
            }	
            // this pair is visited now, put them into set	
            he_id_set.insert(he_i_id);	
            he_id_set.insert(he_j_id);
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
	return E;
}
void PrimoMeshViewer::update_prisms_height_uniform(const std::vector<OpenMesh::FaceHandle> &face_handles, const float dh){
	for (const Mesh::FaceHandle &fh : face_handles){
		Mesh::FaceHalfedgeIter fh_it = mesh_.fh_begin(fh);
		for (; fh_it.is_valid(); ++fh_it){
			PrismProperty &prop = mesh_.property(P_PrismProperty, *fh_it);
			const OpenMesh::Vec3f dFrom = (prop.FromVertPrismUp - prop.FromVertPrismDown).normalized() * dh;
			const OpenMesh::Vec3f dTo = (prop.ToVertPrismUp - prop.ToVertPrismDown) * dh;
			prop.FromVertPrismUp += dFrom;
			prop.FromVertPrismDown -= dFrom;
			prop.ToVertPrismUp += dTo;
			prop.ToVertPrismDown -= dTo;
		}
	}
}
void PrimoMeshViewer::setup_prisms(const std::vector<OpenMesh::FaceHandle> &face_handles, EPrismExtrudeMode PrismExtrudeMode /*= EPrismExtrudeMode::FACE_NORMAL*/)
{
	for (const Mesh::FaceHandle &fh : face_handles)
	{
		Mesh::FaceHalfedgeCWIter fh_cwit = mesh_.fh_cwbegin(fh);
		// Initialize a default face transformation
		//mesh_.property(P_FaceTransformationCache, fh) = Transformation();
		float area_face_i = calc_face_area(fh);
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
				Mesh::Point  p0 = mesh_.point(v0);
				Mesh::Point p1 = mesh_.point(v1);
				prop.FromVertPrismUp = p0 + n0 * prismHeight_;
				prop.FromVertPrismDown = p0 - n0 * prismHeight_;
				prop.ToVertPrismUp = p1 + n1 * prismHeight_;
				prop.ToVertPrismDown = p1 - n1 * prismHeight_;

				// calculate weight_ij
				// Grab the data to construct the face 
				Mesh::HalfedgeHandle he_ji = mesh_.opposite_halfedge_handle(*fh_cwit);
				Mesh::FaceHandle fh_j = mesh_.opposite_face_handle(*fh_cwit);
				float area_face_j = 0.0f;
				float edge_len_sqr = (p0 - p1).sqrnorm();
				if (he_ji.is_valid() && !mesh_.is_boundary(*fh_cwit) && fh_j.is_valid())
				{
					//opposite halfedge and face exist.
					area_face_j = calc_face_area(fh_j);
				}
				prop.weight_ij = edge_len_sqr / (area_face_i + area_face_j);
				mesh_.property(P_PrismProperty, *fh_cwit) = prop;
			}
				break;
			case EPrismExtrudeMode::FACE_NORMAL:
			{
				PrismProperty prop;
				Mesh::Normal n0 = mesh_.normal(fh);
				Mesh::Normal n1 = mesh_.normal(fh);
				Mesh::VertexHandle v0 = mesh_.from_vertex_handle(*fh_cwit);
				Mesh::VertexHandle v1 = mesh_.to_vertex_handle(*fh_cwit);
				Mesh::Point  p0 = mesh_.point(v0);
				Mesh::Point p1 = mesh_.point(v1);

				prop.FromVertPrismUp = p0 + n0 * prismHeight_;
				prop.FromVertPrismDown = p0 - n0 * prismHeight_;
				prop.ToVertPrismUp = p1 + n1 * prismHeight_;
				prop.ToVertPrismDown = p1 - n1 * prismHeight_;

				// calculate weight_ij
				// Grab the data to construct the face 
				Mesh::HalfedgeHandle he_ji = mesh_.opposite_halfedge_handle(*fh_cwit);
				Mesh::FaceHandle fh_j = mesh_.opposite_face_handle(*fh_cwit);
				float area_face_j = 0.0f;
				float edge_len_sqr = (p0 - p1).sqrnorm();
				if (he_ji.is_valid() && !mesh_.is_boundary(*fh_cwit) && fh_j.is_valid())
				{
					//opposite halfedge and face exist.
					area_face_j = calc_face_area(fh_j);
				}
				prop.weight_ij = edge_len_sqr / (area_face_i + area_face_j);

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
float PrimoMeshViewer::get_average_vertex_distance(const Mesh &_mesh) const
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
										std::vector<unsigned int> &indices_array){

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
void PrimoMeshViewer::delete_faceHandle(unsigned int faceId, std::vector<OpenMesh::FaceHandle> &face_handles,
											std::unordered_map<int, int> *face_idx_2_i){
	for(auto it = face_handles.begin(); it != face_handles.end(); ++it){
		if(it->idx() == faceId){
			if(face_idx_2_i){
				// before remove, minus 1 all faces after it
				for(auto jt = it + 1; jt != face_handles.end(); ++jt){
					auto map_it = face_idx_2_i->find(jt->idx());
					assert(map_it != face_idx_2_i->end());
					--map_it->second;
					//(*face_idx_2_i)[jt->idx()] -= 1;
				}
				face_idx_2_i->erase(faceId);
			}

			// remove this fh
			face_handles.erase(it);
			return;
		}
	}
	// this funtion is only used by raycast, and fh must be in face_handles where fh.idx()==faceId
	// this assert is just for debug
	assert(false);
}
void PrimoMeshViewer::translate_faces_and_prisms_along_axis(const OpenMesh::Vec3f &axis, float dist, 
											std::vector<OpenMesh::FaceHandle> &face_handles){
	// translate all the vertices and prisms of face_handles, along axis, dist 
	std::unordered_set<int> vertex_idxs;
	OpenMesh::Vec3f trans(dist * axis);
	//Transformation tr(angle, Vector3f(rotation_axis[0], rotation_axis[1],rotation_axis[2]));
	for(OpenMesh::FaceHandle &fh : face_handles){
		for(Mesh::FaceVertexIter fv_it = mesh_.fv_begin(fh); fv_it.is_valid(); ++fv_it){
			// if vertex is visited, do nothing
 			if(vertex_idxs.find(fv_it->idx()) != vertex_idxs.end()) continue;
			vertex_idxs.insert(fv_it->idx());
			// rotate this vertex
			mesh_.point(*fv_it) += trans;
 		}
		// transform all vertices of this face
		// only 4 vertices of each prism face are transformed, FromVertNormal/ToVertNormal
		for(Mesh::FaceHalfedgeIter fh_it = mesh_.fh_begin(fh); fh_it.is_valid(); ++fh_it){
			PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_it);

			prop.FromVertPrismUp += trans;
			prop.FromVertPrismDown += trans;
			prop.ToVertPrismUp += trans;
			prop.ToVertPrismDown += trans;
		}
	}
}
void PrimoMeshViewer::squeeze_prisms(const std::vector<OpenMesh::FaceHandle> &face_handles, const OpenMesh::Vec3f &target){
	// transform all faces of face_handles to the target position
	// while KEEP REGIDITY OF PRISMS!
	for (const Mesh::FaceHandle &fh : face_handles){
		Mesh::FaceHalfedgeIter fh_it = mesh_.fh_begin(fh);
		Mesh::FaceVertexIter fv_it = mesh_.fv_begin(fh);
		// calculate centoird of this face
		Mesh::Point centoird(0,0,0);
		for(; fv_it.is_valid(); ++fv_it){
			centoird += mesh_.point(*fv_it);
		}
		static constexpr float one_third = 1 / 3.0f; 
		centoird *= one_third;
		Mesh::Point dir(target - centoird);
		float angle = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/M_PI));
		dir += centoird;
		Transformation tr(angle, mesh_.normal(fh));
		for(; fv_it.is_valid(); ++fv_it){
			mesh_.point(*fv_it) = tr.transformPoint(mesh_.point(*fv_it)-centoird) + dir;
		}
		for(; fh_it.is_valid(); ++fh_it){
			PrismProperty &prop = mesh_.property(P_PrismProperty, *fh_it);
			prop.FromVertPrismUp = tr.transformPoint(prop.FromVertPrismUp-centoird) + dir;
			prop.FromVertPrismDown = tr.transformPoint(prop.FromVertPrismDown-centoird) + dir;
			prop.ToVertPrismUp = tr.transformPoint(prop.ToVertPrismUp-centoird) + dir;
			prop.ToVertPrismDown = tr.transformPoint(prop.ToVertPrismDown-centoird) + dir;
		}
	}
	update_vertices_based_on_prisms();
	mesh_.update_normals();
}
void PrimoMeshViewer::rotate_faces_and_prisms_around_centroid(const OpenMesh::Vec3f &rotation_centroid, const OpenMesh::Vec3f &rotation_axis
										, float angle, std::vector<OpenMesh::FaceHandle> &face_handles){
	// rotate all the vertices and prisms of face_handles, around rotation_centroid & axis, angle rad
	std::unordered_set<int> vertex_idxs;
	Transformation tr(angle, Vector3f(rotation_axis[0], rotation_axis[1],rotation_axis[2]));
	for(OpenMesh::FaceHandle &fh : face_handles){
		for(Mesh::FaceVertexIter fv_it = mesh_.fv_begin(fh); fv_it.is_valid(); ++fv_it){
			// if vertex is visited, do nothing
 			if(vertex_idxs.find(fv_it->idx()) != vertex_idxs.end()) continue;
			vertex_idxs.insert(fv_it->idx());
			// rotate this vertex
			mesh_.point(*fv_it) = tr.transformPoint(mesh_.point(*fv_it) - rotation_centroid) + rotation_centroid;
 		}
		// transform all vertices of this face
		// only 4 vertices of each prism face are transformed, FromVertNormal/ToVertNormal
		// are not transformed. 
		for(Mesh::FaceHalfedgeIter fh_it = mesh_.fh_begin(fh); fh_it.is_valid(); ++fh_it){
			PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_it);
			prop.FromVertPrismUp -= rotation_centroid;
			prop.FromVertPrismDown -= rotation_centroid;
			prop.ToVertPrismUp -= rotation_centroid;
			prop.ToVertPrismDown -= rotation_centroid;
			
			prop.FromVertPrismUp = tr.transformPoint(prop.FromVertPrismUp);
			prop.FromVertPrismDown = tr.transformPoint(prop.FromVertPrismDown);
			prop.ToVertPrismUp = tr.transformPoint(prop.ToVertPrismUp);
			prop.ToVertPrismDown = tr.transformPoint(prop.ToVertPrismDown);

			prop.FromVertPrismUp += rotation_centroid;
			prop.FromVertPrismDown += rotation_centroid;
			prop.ToVertPrismUp += rotation_centroid;
			prop.ToVertPrismDown += rotation_centroid;
		}
	}
}


void PrimoMeshViewer::read_mesh_and_cp(const std::string& mesh_filename, const std::string& crease_pattern_filename)
{
	// Read the obj file to make sure we have the mesh_ property

	// read the crease pattern file to get the important points
	// populate/update teh crease pattern edge list based on the file
	read_crease_pattern(crease_pattern_filename);


	// if triangulation is needed, compute triangulation on the mesh, preserving the crease patter
	bool b_triangulation_needed = false;

	// refresh the crease edge list on the newly triangulated points after triangulation
}

void PrimoMeshViewer::test_read_crease_pattern()
{
	// Read crease pattern
	read_crease_pattern(test_crease_file);

	// Then triangulate

}

void PrimoMeshViewer::read_crease_pattern(const std::string& filename)
{
	creases.clear();
	// check if file type is properly .cpx
	std::string extension = ".cpx";
	// open file
	std::ifstream ifs(filename);
	if (ifs)
	{
		// read line by line

		// for each line parse the line
		// store approriate info in creases
		std::string line;
		// assume line is a single line of crease pattern
		int crease_count = 0;
		while (std::getline(ifs, line))
		{
			std::stringstream ss(line);
			int type;
			ss >> type;
			switch (type)
			{
			case 1:
			{
				// countour, should should do sanity check here if time allows
				std::cout << "read a countour" << std::endl;
			}
			break;
			case 2:
			case 3:
			{
				// 2 means mountain
				// 3 means valley

				crease_count++;
				std::vector<Vector3f> crease_points;
				int segments;
				get_points_from_line(line, crease_points, segments);

				// with crease points, search for corresponding half edges
				std::vector<Mesh::HalfedgeHandle> crease_hehs;
				std::vector<Mesh::Point> crease_points_on_mesh;
				// init parameters
				const Vector3f start = crease_points[0];
				Mesh::Point start_point(start[0], start[1], start[2]);
				const Vector3f end = crease_points[3];
				Mesh::Point end_point(end[0], end[1], end[2]);
				Mesh::VertexHandle start_vh = get_closes_vertex(start_point);
				Mesh::VertexHandle end_vh = get_closes_vertex(end_point);
				switch (crease_points.size())
				{
				case 4:
				{
					// shortest
					std::cout << "start searching crease halfedges." << std::endl;

					Mesh::VertexHandle current_vh = start_vh;
					Mesh::VertexHandle last_vh;
					BezierCurve<float> bezier_curve(crease_points);
					// I am just going to search based on the crease points and segment
					for (int i = 0; i < segments; i++)
					{
						// try to find the half edge based on the segment point count
						Vector3f segment_pos = bezier_curve.Eval((double)(i + 0.5) / segments);
						for (Mesh::VertexOHalfedgeCCWIter voh_iter = mesh_.voh_ccwbegin(current_vh); voh_iter.is_valid(); voh_iter++)
						{
							Mesh::VertexHandle candidate_vh = mesh_.to_vertex_handle(*voh_iter);
							Mesh::Point candidate_point = mesh_.point(candidate_vh);
							Vector3f potential_pt(candidate_point[0], candidate_point[1], candidate_point[2]);
							if (length(segment_pos - potential_pt) < 1e-5)
							{
								std::cout << "found a potential half edge for point " << potential_pt.to_string() << std::endl;
								last_vh = current_vh;
								current_vh = candidate_vh;
								crease_hehs.push_back(*voh_iter);
								crease_points_on_mesh.push_back(candidate_point);
								break;
							}
						}
					}
					// Now add the last one
					for (Mesh::VertexOHalfedgeCCWIter voh_iter = mesh_.voh_ccwbegin(current_vh); voh_iter.is_valid(); voh_iter++)
					{
						Mesh::VertexHandle candidate_vh = mesh_.to_vertex_handle(*voh_iter);
						Mesh::Point candidate_point = mesh_.point(candidate_vh);
						if (candidate_vh == end_vh)
						{
							std::cout << "found the end of our crease " << std::endl;
							current_vh = candidate_vh;
							crease_hehs.push_back(*voh_iter);
							crease_points_on_mesh.push_back(candidate_point);
						}
					}
					/*
					while (current_vh != end_vh)
					{
					if (crease_hehs.size() % 1 == 0)
					{
					std::cout << "current crease has" << crease_hehs.size()  << "halfedges. "<< std::endl;
					}
					for (Mesh::VertexOHalfedgeCCWIter voh_iter = mesh_.voh_ccwbegin(current_vh); voh_iter.is_valid(); voh_iter++)
					{
					Mesh::VertexHandle candidate_vh = mesh_.to_vertex_handle(*voh_iter);
					// skip the last half edge
					if (candidate_vh == last_vh) { continue; }

					Mesh::Point candidate_point = mesh_.point(candidate_vh);
					Vector3f potential_pt(candidate_point[0], candidate_point[1], candidate_point[2]);
					if (bezier_curve.IsPointOnSplineInaccurate(potential_pt,segments))
					{
					std::cout << "found a potential half edge for point " << potential_pt.to_string()<< std::endl;
					last_vh = current_vh;
					current_vh = candidate_vh;
					crease_hehs.push_back(*voh_iter);
					crease_points_on_mesh.push_back(candidate_point);
					break;
					}
					}
					}*/
				}
				break;
				default:
				{
					std::cout << "crease point number not supported" << std::endl;
				}
				break;
				}
				creases.emplace_back(crease_hehs);
			}
			break;
			default:
				break;
			}
		}
	}
	else
	{
		std::cout << "file is not found " << filename << std::endl;
	}
	std::cout << "finished parsing crease pattern" << filename << std::endl;
	std::cout << "Total crease found " << creases.size() << std::endl;

}


void PrimoMeshViewer::triangulate_by_boundary(const std::vector<HalfedgeHandle>& boundary_hehs)
{
	if (!is_legal_boundary(boundary_hehs))
	{
		std::cout << "the boundary passed in is not legal, sorry mate" << std::endl;
	}

	// Actual triangulation
	void triangulate(char *, struct triangulateio *, struct triangulateio *,
		struct triangulateio *);

}

bool PrimoMeshViewer::is_legal_boundary(const std::vector<HalfedgeHandle>& boundary_hehs) const
{
	return false;
}

void PrimoMeshViewer::get_points_from_line(std::string& line, std::vector<Vector3f>& out_points, int& segments)
{
	out_points.clear();
	std::cout << "reading crease line" << std::endl;
	std::stringstream ss(line);
	int type;
	ss >> type;
	// crease index should start from here
	float start_x, start_y;
	float end_x, end_y;
	float p1_x, p1_y;
	float p2_x, p2_y;
	ss >> start_x;
	ss >> start_y;
	Vector3f p0(start_x, 0, start_y);
	out_points.push_back(p0);
	ss >> end_x;
	ss >> end_y;
	Vector3f p1(end_x, 0, end_y);
	out_points.push_back(p1);
	if (ss)
	{
		// 1st control point
		ss >> p1_x;
		ss >> p1_y;
		Vector3f p(p1_x, 0, p1_y);
		out_points.insert(out_points.end() - 1, p);
	}
	if (ss)
	{
		// 2nd control point
		ss >> p2_x;
		ss >> p2_y;
		Vector3f p(p2_x, 0, p2_y);
		out_points.insert(out_points.end() - 1, p);
	}
	if (ss)
	{
		// get num segments
		ss >> segments;
	}
	std::cout << "finished reading crease line " << line << std::endl;

}

MeshViewer::Mesh::VertexHandle PrimoMeshViewer::get_closes_vertex(Mesh::Point p)
{
	float smallest_dist = INT_MAX;
	Mesh::VertexHandle vh;
	for (Mesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); v_it++)
	{
		Mesh::Point cur_p = mesh_.point(*v_it);
		if ((cur_p - p).norm() < smallest_dist)
		{
			vh = *v_it;
			smallest_dist = (cur_p - p).norm();
		}
	}
	if (!vh.is_valid())
	{
		std::cout << "Cannot find closest point" << std::endl;
	}
	return vh;
}
