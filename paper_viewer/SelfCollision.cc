// #include "PrimoMeshViewer.h"
// #include <iostream>
// #include <Eigen/Dense>
// #include <Eigen/Sparse>
// #include <Eigen/Eigenvalues>
// #include <glm/gtc/quaternion.hpp>
// #include <unordered_set>
// #include <fstream>
// void PrimoMeshViewer::setup_faceBN(const std::vector<OpenMesh::FaceHandle> &face_handles){
// 	// save normal(N), barycenter(B) into faceproperty P_faceBN for all face_handle in face_handles
// 	static constexpr float one_third = 1.0f/ 3.0f;
// 	for(const auto &face_handle : face_handles){
// 		PrismFaceProperty &prop = mesh_.property(P_faceBN, face_handle);
// 		prop.n = mesh_.normal(face_handle);
// 		Mesh::Point b(0.0f, 0.0f, 0.0f);
// 		for(Mesh::FaceVertexIter fv_iter = mesh_.fv_begin(face_handle); fv_iter.is_valid(); ++fv_iter){
// 			b += mesh_.point(*fv_iter);
// 		}
// 		prop.b = one_third * b;
// 	}
// }
// void PrimoMeshViewer::setup_collisionProperty(){
// 	// initialize all haledges' collision property
// 	// must be used after setup_faceBN
// 	for (Mesh::FaceIter f_it1 = mesh_.faces_begin(); f_it1 != mesh_.faces_end(); ++f_it1){
// 		Mesh::FaceHalfedgeIter fh_it1 = mesh_.fh_begin(*f_it1);
// 		int face_id1 = f_it1->idx();
// 		// Initialize a default face transformation
// 		//mesh_.property(P_FaceTransformationCache, fh) = Transformation();
// 		for (; fh_it1.is_valid(); ++fh_it1){
// 			PrismCollisionProperty &collision_prop = mesh_.property(P_collision,*fh_it1);
// 			// iterate all other faces
// 			for (Mesh::FaceIter f_it2 = mesh_.faces_begin(); f_it2 != mesh_.faces_end(); ++f_it2){
// 				if(f_it2->idx() == face_id1){
// 					continue;
// 				}
// 				const PrismFaceProperty &bn_prop = mesh_.property(P_faceBN, *f_it2);
// 				const float from_dot = OpenMesh::dot(bn_prop.n, (mesh_.point(mesh_.from_vertex_handle(*fh_it1)) - bn_prop.b));
// 				const float to_dot   = OpenMesh::dot(bn_prop.n, (mesh_.point(mesh_.to_vertex_handle(*fh_it1)) - bn_prop.b));
// 				collision_prop.from_faceID_val[f_it2->idx()] = from_dot;
// 				collision_prop.to_faceID_val[f_it2->idx()] = to_dot;
// 			}
// 			// default all is ok(actually does not matter)
// 			collision_prop.from_OK = collision_prop.to_OK = true;
// 		}
// 	}
// }
// void PrimoMeshViewer::update_vertices_based_on_prisms_self_collision()
// {
// 	std::unordered_set<int> visited_vertices_idx;
// 	for(const OpenMesh::FaceHandle &fh: optimizedFaceHandles_){
// 		for (Mesh::FaceVertexIter fv_iter = mesh_.fv_begin(fh); fv_iter.is_valid(); ++fv_iter)
// 		{
// 			// check if this vertice is visited
// 			if(visited_vertices_idx.find(fv_iter->idx()) != visited_vertices_idx.end()){
// 				continue;
// 			}
// 			visited_vertices_idx.emplace(fv_iter->idx());
			
// 			Mesh::Point original_point = mesh_.point(*fv_iter);
// 			Vector3d new_point(0, 0, 0);
// 			float weight = 0;

// 			for (Mesh::VertexOHalfedgeCCWIter voh_ccwiter = mesh_.voh_ccwbegin(*fv_iter); voh_ccwiter.is_valid(); voh_ccwiter++)
// 			{
// 				if (mesh_.face_handle(*voh_ccwiter).is_valid()) // Make sure this half edge has a face
// 				{
// 					PrismProperty& voh_prop = mesh_.property(P_PrismProperty, *voh_ccwiter);
// 					new_point += voh_prop.TargetPosFrom();
// 					weight += 1.0f;

// 					#ifndef NDEBUG
// 					Arrow arrow(
// 						Vector3d(original_point),
// 						Vector3d(voh_prop.TargetPosFrom()),
// 						LinearColor::RED,
// 						3.0f
// 					);
// 					g_debug_arrows_to_draw_local_optimizations.emplace_back(arrow);
// 					#endif
// 				}
// 			}
// 			for (Mesh::VertexIHalfedgeCCWIter vih_ccwiter = mesh_.vih_ccwbegin(*fv_iter); vih_ccwiter.is_valid(); vih_ccwiter++)
// 			{
// 				if (mesh_.face_handle(*vih_ccwiter).is_valid())
// 				{
// 					PrismProperty& voh_prop = mesh_.property(P_PrismProperty, *vih_ccwiter);
// 					new_point += voh_prop.TargetPosTo();
// 					weight += 1.0f;

// 					#ifndef NDEBUG
// 					Arrow arrow(
// 						Vector3d(original_point),
// 						Vector3d(voh_prop.TargetPosTo()),
// 						LinearColor::GREEN,
// 						3.0f
// 					);
// 					g_debug_arrows_to_draw_local_optimizations.emplace_back(arrow);
// 					#endif
// 				}
// 			}
// 			new_point /= weight;

// 			mesh_.point(*fv_iter) = Vec3f(new_point[0], new_point[1], new_point[2]);

// 			#ifndef NDEBUG
// 			Arrow arrow(
// 				Vector3d(original_point),
// 				Vector3d(new_point),
// 				LinearColor::BLUE,
// 				3.0f
// 			);
// 			g_debug_arrows_to_draw_local_optimizations.emplace_back(arrow);
// 			#endif
// 		}
// 	}
// }