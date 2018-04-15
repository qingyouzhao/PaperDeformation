
/*
This source contains the local optimize operations
*/

#include "PrimoMeshViewer.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <glm/gtc/quaternion.hpp>
#include <unordered_set>
void PrimoMeshViewer::local_optimize(const std::vector<OpenMesh::FaceHandle> &face_handles, const int max_iterations)
{
	// do nothing if no optimizable faces or invalid iteration
	if(optimizedFaceHandles_.size() <= 0 || max_iterations <= 0) return;
#ifndef NDEBUG
	g_debug_arrows_to_draw_local_optimizations.clear();
	g_debug_transformations_to_draw_local_optimization.clear();
#endif

	const float E_origin = E(face_handles);
	const float E_threashold = E_origin * 0.1;
	float E_k;
	for (int i = 0; i < max_iterations; i++)
	{

		Mesh::FaceHandle fh;
		// #TODOZQY: take from optimizedFaceHandles_ to get the right face to optimize
		
		int num_faces = optimizedFaceHandles_.size();;
		int idx = rand() % num_faces;

		fh = optimizedFaceHandles_[idx];
		//std::cout << "optimizing face handle idx " << fh.idx() << "optimized face handles idx = " << idx << std::endl;
		local_optimize_face(fh, P_PrismProperty, false);
		E_k = E(face_handles);
		if(E_k <= E_threashold){
		 	printf("[Local Optimization]: converge, finish local optimization\n");
		 	break;
		}
		std::cout << "[Local Optimization]: iteration "<<i<<", E = "<<E_k<<std::endl;
		
	}
	// Now we want to update the face vertex based on rigid prisms instead of the values
	update_vertices_based_on_prisms();
	// update normals of OpenMesh
	mesh_.update_normals();
}




void PrimoMeshViewer::local_optimize_face(Mesh::FaceHandle _fh, const OpenMesh::HPropHandleT<PrismProperty> &Pji_property, bool is_ij)
{
	// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
	// struct EdgePrismData {
	// 	//float weight_ij; // the weigth of the edge prism based on the face area
	// 	Vec3f centroid_ij; // centroid of prism face on i facing j
	// 	Vec3f centroid_ji; // centroid of prism face on j facing i
	// 	Vec3f f_ij[4]; // let's store this number as 00,01,10,11
	// 	Vec3f f_ji[4]; // let's store this number as 00,01,10,11
	// };

	// Clear our debug info before every optimization

	//std::vector<EdgePrismData> f_ij_data;
	//Eigen::Matrix3f S; 
	//S << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	// We need to experiment this
	//Eigen::Matrix3f S_1;
	//S_1 << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	Eigen::Matrix3f S_16 = Eigen::Matrix3f::Zero();
	//S_16 << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	//Eigen::Matrix3f S_4;
	//S_4 << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	// In order to generalize local shape matching of Hor87
	// http://people.csail.mit.edu/bkph/papers/Absolute_Orientation.pdf
	// we first we first compute the weighted centroids c_i and c_∗ of the two face sets to be aligned
	// c^i in the paper
	Vec3f centroid_i(0, 0, 0);
	// c* in the paper
	Vec3f centroid_star(0, 0, 0);
	// sum of neighbour triangle i's
	float weight_sum = 0;
	for (Mesh::FaceHalfedgeCCWIter fhe_ccwiter = mesh_.fh_ccwbegin(_fh); fhe_ccwiter.is_valid(); fhe_ccwiter++)
	{
		Mesh::HalfedgeHandle he_ij = *fhe_ccwiter;
		const PrismProperty &P_ij = mesh_.property(P_PrismProperty, he_ij);
		// circulate Neighbours to calculate centroid
		Mesh::HalfedgeHandle he_ji = (is_ij ? he_ij: mesh_.opposite_halfedge_handle(*fhe_ccwiter));
		// Mesh::FaceHandle fh_j = mesh_.opposite_face_handle(*fhe_ccwiter);
		if (!he_ji.is_valid())
		{
			//std::cout << "halfedge's opposite doesnot exist, idx = " << fhe_ccwiter->idx() << std::endl;
			continue;
		}

		const PrismProperty &P_ji = mesh_.property(Pji_property, he_ji);

		const float weight_ij = P_ij.weight_ij; // the weigth of the edge prism based on the face area
		assert(P_ij.weight_ij == P_ji.weight_ij);

		Vec3f centroid_ij; // centroid of prism face on i facing j
		Vec3f centroid_ji; // centroid of prism face on j facing i
		Vec3f f_ij[4]; // let's store this number as 00,01,10,11
		Vec3f f_ji[4]; // let's store this number as 00,01,10,11

		// Calc face weight
		//float edge_len_sqr = mesh_.calc_edge_sqr_length(*fhe_ccwiter);
		//float area_face_i = calc_face_area(_fh);
		//float area_face_j = calc_face_area(fh_j);
		//weight_ij = edge_len_sqr / (area_face_i + area_face_j);

		// Evaluate f_ij
		f_ij[0] = P_ij.f_uv(0, 0, true);
		f_ij[1] = P_ij.f_uv(0, 1, true);
		f_ij[2] = P_ij.f_uv(1, 0, true);
		f_ij[3] = P_ij.f_uv(1, 1, true);
		centroid_ij = (f_ij[0] + f_ij[1] + f_ij[2] + f_ij[3]) * 0.25f;
		
		// Evaluate f_ji
		f_ji[0] = P_ji.f_uv(0, 0, is_ij);
		f_ji[1] = P_ji.f_uv(0, 1, is_ij);
		f_ji[2] = P_ji.f_uv(1, 0, is_ij);
		f_ji[3] = P_ji.f_uv(1, 1, is_ij);
		centroid_ji = (f_ji[0] + f_ji[1] + f_ji[2] + f_ji[3]) * 0.25f;

		// w_ij / 4 * \sum{f_ij_kl}
		centroid_i += centroid_ij * weight_ij;
		// w_ij / 4 * \sum(f_ji_kl}
		centroid_star += centroid_ji * weight_ij;
		weight_sum += weight_ij;
	}
	// Is my centroid not correct? we need to 
	centroid_i /= weight_sum;
	centroid_star /= weight_sum;
	//std::cout << "Weighted c_i = " << centroid_i << std::endl;
	//std::cout << "Weighted c_* = " << centroid_star << std::endl;

	// Now Calculate neighbor S where S(x,y) = \sum_{N_i} <(f_ij - c_i)_i, (f_ji - c_i)_j>_2
	// On Component-Wise inner product https://en.wikipedia.org/wiki/Frobenius_inner_product
	// 
	for (Mesh::FaceHalfedgeCCWIter fhe_ccwiter = mesh_.fh_ccwbegin(_fh); fhe_ccwiter.is_valid(); fhe_ccwiter++)
	{
		Mesh::HalfedgeHandle he_ij = *fhe_ccwiter;
		const PrismProperty &P_ij = mesh_.property(P_PrismProperty, he_ij);
		// circulate Neighbours to calculate centroid
		Mesh::HalfedgeHandle he_ji = (is_ij ? he_ij: mesh_.opposite_halfedge_handle(*fhe_ccwiter));
		// Mesh::FaceHandle fh_j = mesh_.opposite_face_handle(*fhe_ccwiter);
		if (!he_ji.is_valid())
		{
			//std::cout << "halfedge's opposite doesnot exist, idx = " << fhe_ccwiter->idx() << std::endl;
			continue;
		}

		const PrismProperty &P_ji = mesh_.property(Pji_property, he_ji);

		const float weight_ij = P_ij.weight_ij; // the weigth of the edge prism based on the face area
		assert(P_ij.weight_ij == P_ji.weight_ij);
		Vec3f centroid_ij; // centroid of prism face on i facing j
		Vec3f centroid_ji; // centroid of prism face on j facing i
		Vec3f f_ij[4]; // let's store this number as 00,01,10,11
		Vec3f f_ji[4]; // let's store this number as 00,01,10,11

		// Calc face weight, again
		//float edge_len_sqr = mesh_.calc_edge_sqr_length(*fhe_ccwiter);
		//float area_face_i = calc_face_area(_fh);
		//float area_face_j = calc_face_area(fh_j);
		//weight_ij = edge_len_sqr / (area_face_i + area_face_j);

		// Evaluate f_ij
		f_ij[0] = P_ij.f_uv(0, 0, true);
		f_ij[1] = P_ij.f_uv(0, 1, true);
		f_ij[2] = P_ij.f_uv(1, 0, true);
		f_ij[3] = P_ij.f_uv(1, 1, true);
		centroid_ij = (f_ij[0] + f_ij[1] + f_ij[2] + f_ij[3]) * 0.25f;

		// Evaluate f_ji
		f_ji[0] = P_ji.f_uv(0, 0, is_ij);
		f_ji[1] = P_ji.f_uv(0, 1, is_ij);
		f_ji[2] = P_ji.f_uv(1, 0, is_ij);
		f_ji[3] = P_ji.f_uv(1, 1, is_ij);
		centroid_ji = (f_ji[0] + f_ji[1] + f_ji[2] + f_ji[3]) * 0.25f;

		// Update S_components based on contribution from this face
		for (int s_i = 0; s_i < 3; s_i++)
		{
			for (int s_j = 0; s_j < 3; s_j++)
			{
				// If we assume each face has 4 points contributing
				// float S_4_ij = 0;
				// for(int kl = 0; kl < 4; kl++)
				// {
				// 	S_4_ij += weight_ij * ((f_ij[kl] - centroid_i)[s_i] * (f_ji[kl] - centroid_star)[s_j]);
				// }
				// S_4(s_i, s_j) += S_4_ij;

				// If we assume each face has 1 point contributing, only from the centroid
				// float S_1_ij = weight_ij * (centroid_ij - centroid_i)[s_i] * (centroid_ji - centroid_star)[s_j];
				// S_1(s_i, s_j) += S_1_ij;
				// if we assume each face has the 16 point pair contribution
				float S_16_ij = 0;
				static const float root_2s[4] = { 1, 0.5, 0.5, 0.25 }; // TODO : his guy should be global later but for experimentation purpose, heck
				for (int ij = 0; ij < 4; ij++)
				{
					for (int kl = 0; kl < 4; kl++)
					{
						// int ijkl_dist = ((~ij) & kl);
						int ijkl_dist = (ij) ^ (kl);
						S_16_ij += (f_ij[ij] - centroid_i)[s_i] * (f_ji[kl] - centroid_star)[s_j] * root_2s[ijkl_dist];
					}
				}

				S_16(s_i, s_j) += S_16_ij;
			}	
		}
	}
	Eigen::Quaternion<double> target_quat;
	// std::cout << "Computing optimal with 1 centroid: " << std::endl;
	// Transformation TargetTransformation_1 = compute_optimal_face_transform(S_1, centroid_i, centroid_star);
	// std::cout << "Computing optimal with 4 points: " << std::endl;
	// Transformation TargetTransformation_4 = compute_optimal_face_transform(S_4, centroid_i, centroid_star);
	// std::cout << "Computing optimal with 4x4 point matching: " << std::endl;
	Transformation TargetTransformation = compute_optimal_face_transform(S_16, centroid_i, centroid_star);
	//Transformation TargetTransformation = TargetTransformation_16;

	#ifndef NDEBUG
	g_debug_transformations_to_draw_local_optimization.emplace_back(TargetTransformation);
	#endif
	
	// Update the prism on each half edge with the new transformation
	for (Mesh::FaceHalfedgeCCWIter fh_ccwit = mesh_.fh_ccwbegin(_fh); fh_ccwit.is_valid(); fh_ccwit++)
	{
		PrismProperty& prop = mesh_.property(P_PrismProperty, *fh_ccwit);
		// Wrap transform prism with our points
		#ifndef NDEBUG
		Arrow from_up(prop.FromVertPrismUp, prop.FromVertPrismUp, LinearColor::YELLOW);
		Arrow from_down(prop.FromVertPrismDown, prop.FromVertPrismDown, LinearColor::YELLOW * 0.5);
		Arrow to_up(prop.ToVertPrismUp, prop.ToVertPrismUp, LinearColor::PURPLE);
		Arrow to_down(prop.ToVertPrismDown, prop.ToVertPrismDown, LinearColor::PURPLE*0.5);
		#endif
		
		prop.TransformPrism(TargetTransformation);
		
		#ifndef NDEBUG
		from_up.to = prop.FromVertPrismUp;
		from_down.to = prop.FromVertPrismDown;
		to_up.to = prop.ToVertPrismUp;
		to_down.to = prop.ToVertPrismDown;
		g_debug_arrows_to_draw_local_optimizations.emplace_back(from_down);
		g_debug_arrows_to_draw_local_optimizations.emplace_back(from_up);
		g_debug_arrows_to_draw_local_optimizations.emplace_back(to_up);
		g_debug_arrows_to_draw_local_optimizations.emplace_back(to_down);
		#endif
	}

	// bool update_half_edge_data = false;
	// if(update_half_edge_data)
	// {
	// 	// Update verts for this face, the verts should be the average value of the neighbouring prism verts.
	// 	for (Mesh::FaceVertexCCWIter fv_ccwit = mesh_.fv_ccwbegin(_fh); fv_ccwit.is_valid(); fv_ccwit++)
	// 	{

	// 		Vec3f& pt = mesh_.point(*fv_ccwit);

	// 		// Calculate the new vert position based on the weighted average of all the prisms
	// 		// #TODOZQY: might need a Laplacian weight but for now just use area
	// 		Vec3f newPos(0, 0, 0);
	// 		Vec3f newNormal(0, 0, 0);
	// 		float weight = 0.f;
	// 		// Iterate thru all the half edges that are out going, we should be able to find all the vertex 


	// 		for (Mesh::VertexOHalfedgeCCWIter voh_ccwit = mesh_.voh_ccwbegin(*fv_ccwit); voh_ccwit.is_valid(); voh_ccwit++)
	// 		{
	// 			PrismProperty& prop_out = mesh_.property(P_PrismProperty, *voh_ccwit);
	// 			Vec3f vert_pt_out = prop_out.TargetPosFrom();
	// 			FaceHandle fo = mesh_.face_handle(*voh_ccwit);
	// 			if (!fo.is_valid())
	// 			{
	// 				//std::cout << "out halfedge handle does not have a face, skipping halfedge" << std::endl;
	// 				continue;
	// 			}
	// 			float ohe_weight = calc_face_area(fo);
	// 			ohe_weight = 1.f; // #TODOZQY: get the proper way to calculate weight center of multiple points
	// 			newPos += vert_pt_out * ohe_weight;
	// 			weight += ohe_weight;
	// 			//std::cout << "Adding weighted point = " << vert_pt_out << "with weight = " << ohe_weight << std::endl;
	// 		}
	// 		for (Mesh::VertexIHalfedgeCCWIter vih_ccwit = mesh_.vih_ccwbegin(*fv_ccwit); vih_ccwit.is_valid(); vih_ccwit++)
	// 		{
	// 			PrismProperty& prop_in = mesh_.property(P_PrismProperty, *vih_ccwit);
	// 			Vec3f vert_pt_in = prop_in.TargetPosTo();
	// 			FaceHandle fi = mesh_.face_handle(*vih_ccwit);
	// 			if (!fi.is_valid())
	// 			{
	// 				//std::cout << "in halfedge handle does not have a face, skipping halfedge" << std::endl;
	// 				continue;
	// 			}
	// 			float ihe_weight = calc_face_area(fi);
	// 			ihe_weight = 1.f; // #TODOZQY: get the proper way to calculate weight center of multiple points
	// 			newPos += vert_pt_in * ihe_weight;
	// 			weight += ihe_weight;
	// 			//std::cout << "Adding weighted point = " << vert_pt_in << "with weight = " << ihe_weight << std::endl;

	// 		}
	// 		// calculate now position and update to the vertex
	// 		newPos /= weight;
	// 		//std::cout << "UPDATE: moving vert " << mesh_.point(*fv_ccwit) << " to " << newPos << std::endl;
	// 		//std::cout << std::endl;
	// 		mesh_.point(*fv_ccwit) = newPos;
	// 	}
	// 	mesh_.update_face_normals();
	// }
	// else
	// {
	// 	// In this case, following the new solution, we only compose the transformation, don't update faces yet.
	// 	Transformation& cached_transformation = mesh_.property(P_FaceTransformationCache, _fh);
	// 	std::cout << "Updating face transformation from " << cached_transformation.to_string()  << " composing with " << TargetTransformation.to_string() << std::endl;
	// 	cached_transformation = TargetTransformation * cached_transformation;
	// 	std::cout << "Updated transformation on face " << _fh.idx() << " to " << cached_transformation.to_string() << std::endl;
	// }
	

}


Transformation PrimoMeshViewer::compute_optimal_face_transform(const Eigen::Matrix3f &S, const Vector3d &c_i, const Vector3d &c_star) const
{

	// Construct matrix N 
	/*
	N =
	| Sxx + Syy + Szz		Syz - Szy			Szx - Sxz			Sxy - Syx		 |
	| Syz - Szy				Sxx - Syy - Szz		Sxy + Syx			Szx + Sxz		 |
	| Szx - Sxz				Sxy + Syx			-Sxx + Syy - Szz	Syz + Szy		 |
	| Sxy - Syx				Szx + Sxz			Syz + Szy			-Sxx - Syy + Szz |
	*/

	Eigen::Matrix4d N;
	N << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	const int x = 0;
	const int y = 1;
	const int z = 2;
	N(0, 0) = S(x, x) + S(y, y) + S(z, z);
	N(1, 0) = S(y, z) - S(z, y);
	N(2, 0) = S(z, x) - S(x, z);
	N(3, 0) = S(x, y) - S(y, x);

	N(0, 1) = S(y, z) - S(z, y);
	N(1, 1) = S(x, x) - S(y, y) - S(z, z);
	N(2, 1) = S(x, y) + S(y, x);
	N(3, 1) = S(z, x) + S(x, z);

	N(0, 2) = S(z, x) - S(x, z);
	N(1, 2) = S(x, y) + S(y, x);
	N(2, 2) = -S(x, x) + S(y, y) - S(z, z);
	N(3, 2) = S(y, z) + S(z, y);

	N(0, 3) = S(x, y) - S(y, x);
	N(1, 3) = S(z, x) + S(x, z);
	N(2, 3) = S(y, z) + S(z, y);
	N(3, 3) = -S(x, x) - S(y, y) + S(z, z);

	// Find the largest eigen vector of N and interpret that eigen vector as a rotation quaternion
	//https://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
	Eigen::EigenSolver<Eigen::Matrix4d> es;
	es.compute(N, true);
	Eigen::EigenSolver<Eigen::Matrix4d>::EigenvectorsType eigenvectors = es.eigenvectors();
	Eigen::Matrix<std::complex<double>, 4, 1> eigenvalues = es.eigenvalues();
	//std::cout << "We have " << es.eigenvectors().size() << " eigen vectors" << std::endl;

	// Now if we can find that max eigen value -> eigen vector, we get the rotation
	float max_eigen_val = INT_MIN;
	float max_col = -1;
	for (int i = 0; i < eigenvalues.rows(); i++)
	{
		// Values 
		if (eigenvalues(i, 0).real() > max_eigen_val)
		{
			max_eigen_val = eigenvalues(max_col = i, 0).real();
		}
	}
	assert(max_col >= 0 && max_col < 4);
	Eigen::Quaternion<double> target_quat;
	target_quat.w() = eigenvectors.col(max_col)(0).real();
	target_quat.x() = eigenvectors.col(max_col)(1).real();
	target_quat.y() = eigenvectors.col(max_col)(2).real();
	target_quat.z() = eigenvectors.col(max_col)(3).real();
	// for (int col_i = 0; col_i < eigenvectors.cols(); col_i++)
	// {
	// 	Eigen::Quaternion<double> rot_quat;
	// 	// convert the i the eigen vector to a quaternion
	// 	Eigen::Matrix<std::complex<double> ,4, 1> eigenvector = eigenvectors.col(col_i);
	// 	// Eigen::EigenSolver<Eigen::Matrix4d>::ComplexScalar eigenvector = eigenvectors.col(col_i);
	// 	rot_quat.x() = eigenvector(0).real() * max_eigen_val;
	// 	rot_quat.y() = eigenvector(1).real() * max_eigen_val;
	// 	rot_quat.z() = eigenvector(2).real() * max_eigen_val;
	// 	rot_quat.w() = eigenvector(3).real() * max_eigen_val;

	// 	// Multiply matrix N and the eigen vector to get a vector
	// 	Eigen::Matrix<std::complex<double>, 4, 1> Nv = (N * eigenvector);

	// 	// Check if we have a small enough difference
	// 	Eigen::Vector4d diff;
	// 	diff(0) = Nv(0).real() - rot_quat.x();
	// 	diff(1) = Nv(1).real() - rot_quat.y();
	// 	diff(2) = Nv(2).real() - rot_quat.z();
	// 	diff(3) = Nv(3).real() - rot_quat.w();

	// 	if (diff.dot(diff) < 1E-4 && diff.dot(diff) > -1E-4)
	// 	{
	// 		// WE got that  new rotation
	// 		std::cout << "The closes quaternion is \n" << eigenvector << std::endl;
	// 		target_quat.w() = eigenvector(0).real();
	// 		target_quat.x() = eigenvector(1).real();
	// 		target_quat.y() = eigenvector(2).real();
	// 		target_quat.z() = eigenvector(3).real();
	// 		break;
	// 	}
	// }

	//target_quat.normalize();
	// Calculate R, t and c
	Eigen::Vector3d T_i = Eigen::Vector3d(c_star[0], c_star[1], c_star[2]) - target_quat._transformVector(Eigen::Vector3d(c_i[0], c_i[1], c_i[2]));
	Vector3d T_i_v3d = Vector3d(T_i(0), T_i(1), T_i(2));
	//std::cout << "Target translation = \n" << T_i << std::endl;


	Transformation tm(target_quat, T_i_v3d);
	//std::cout << "Source S matrix = " << std::endl;
	//std::cout << S << std::endl;
	//std::cout << "Optimal rotation Q" << std::endl;
	//print_quaternion(target_quat);
	//std::cout << "Transformation = " << std::endl;
	//std::cout << tm.to_string() << std::endl;

	return tm;
}