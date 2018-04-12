#include "PrimoMeshViewer.h"
#include <Eigen/Sparse>
#include <unordered_set>
typedef Eigen::SparseMatrix<float> SpMat;
typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
static void fill_in_D_T(const OpenMesh::Vec3f &p_ij, 
                        const int f_i_id, bool is_filling_i, SpMat &D_T){
    //helper function to fill in D_ab and D_cd, refer to ZJW's note
    const int i_startId = f_i_id * 6;
    const OpenMesh::Vec3f p = is_filling_i ? p_ij : -p_ij;
    const int one = is_filling_i ? 1 : -1;
    // const int j_startId = f_j_id * 6;
    // fill in first column
    D_T.insert(i_startId + 1, 0) =  p[2];
    D_T.insert(i_startId + 2, 0) = -p[1];
    D_T.insert(i_startId + 3, 0) =  one;
    D_T.insert(i_startId + 4, 0) =  one;
    D_T.insert(i_startId + 5, 0) =  one;

    // D_T.insert(j_startId + 1, 0) = -p_ji[2];
    // D_T.insert(j_startId + 2, 0) =  p_ji[1];
    // D_T.insert(j_startId + 3, 0) = -1;
    // D_T.insert(j_startId + 4, 0) = -1;
    // D_T.insert(j_startId + 5, 0) = -1;

    // fill in second column
    D_T.insert(i_startId    , 1) = -p[2];
    D_T.insert(i_startId + 2, 1) =  p[0];
    D_T.insert(i_startId + 3, 1) =  one;
    D_T.insert(i_startId + 4, 1) =  one;
    D_T.insert(i_startId + 5, 1) =  one;

    // D_T.insert(j_startId    , 1) =  p_ji[2];
    // D_T.insert(j_startId + 2, 1) = -p_ji[0];
    // D_T.insert(j_startId + 3, 1) = -1;
    // D_T.insert(j_startId + 4, 1) = -1;
    // D_T.insert(j_startId + 5, 1) = -1;

    // fill in third column
    D_T.insert(i_startId    , 2) =  p[1];
    D_T.insert(i_startId + 1, 2) = -p[0];
    D_T.insert(i_startId + 3, 2) =  one;
    D_T.insert(i_startId + 4, 2) =  one;
    D_T.insert(i_startId + 5, 2) =  one;

    // D_T.insert(j_startId    , 2) = -p_ji[1];
    // D_T.insert(j_startId + 1, 2) =  p_ji[0];
    // D_T.insert(j_startId + 3, 2) = -1;
    // D_T.insert(j_startId + 4, 2) = -1;
    // D_T.insert(j_startId + 5, 2) = -1;
}
// helper funtion to build linear system
static void build_problem_Eigen(const int n6, const Mesh &mesh, const OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty, 
                            const std::vector<OpenMesh::FaceHandle> &face_handles, const std::unordered_set<int> &face_id_set, 
                            SpMat &B_add_BT, 
                            Eigen::VectorXf &negA_T){
    std::unordered_set<int> he_id_set;
    //////////////////////////////////////////////////////////////////////////////
    std::cout<< "B_and_BT:\n" <<  B_add_BT<<std::endl;
    std::cout<< "-A^T:\n" << negA_T << std::endl;
    //////////////////////////////////////////////////////////////////////////////
    assert(face_handles.size() == face_id_set.size());
    for(int i = 0; i < face_handles.size(); ++i){
        // iterate all faces
        const OpenMesh::FaceHandle &fh_i = face_handles[i];
        const int f_i_id = fh_i.idx();
        assert(face_id_set.find(f_i_id) != face_id_set.end());
        for(Mesh::ConstFaceHalfedgeIter fhe_it = mesh.cfh_iter(fh_i); fhe_it.is_valid(); ++fhe_it){
            // Grab the opposite half edge and face
            const Mesh::HalfedgeHandle he_i = *fhe_it;
            Mesh::HalfedgeHandle he_j = mesh.opposite_halfedge_handle(he_i);
		    Mesh::FaceHandle fh_j = mesh.opposite_face_handle(*fhe_it);
            // if he_i is a boundary halfedge, there is no opposite prism face,
            // which means that it has no contribution to the total energy E = Sum(w_ij * E_ij)
            // we could skip this half edge he_i now
            if (!he_j.is_valid() || mesh.is_boundary(he_i) || !fh_j.is_valid()){
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
            const PrismProperty * const P_i = &(mesh.property(P_PrismProperty, he_i));
            const PrismProperty * const P_j = &(mesh.property(P_PrismProperty, he_j));
            const OpenMesh::Vec3f f_ij[2][2] = { 
                {P_i->FromVertPrismDown, P_i->FromVertPrismUp},
                {P_i->ToVertPrismDown, P_i->ToVertPrismUp}
            };
            const OpenMesh::Vec3f f_ji[2][2] = { 
                {P_j->ToVertPrismDown, P_j->ToVertPrismUp},
                {P_j->FromVertPrismDown, P_j->FromVertPrismUp}
            };
            const float w_ij = P_i->weight_ij;
            // assert for debug, opposite half edges should have same edge weight
            assert(fabs(w_ij - P_j->weight_ij) < FLT_EPSILON);

            // 10 non-repetitive combination for the diecretion of integration over [0, 1]^[0, 1] 
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
            // final weight
            // #TODO[ZJW]: Here I suppose that the 1/9 constant does not influence the result,
            // need double check.
            const float lambdas[10] = {
                w_ij,
                w_ij,
                w_ij,
                0.5f * w_ij,
                w_ij,
                0.5f * w_ij,
                w_ij,
                w_ij,
                w_ij,
                w_ij
            };
            // iterate each combination
            for(int cid = 0; cid < 10; ++cid){
                // follow the convention of ZJW's note
                // #TODO[ZJW]: Double Check and add note link
                const OpenMesh::Vec3f &a = f_ij[ uv_integrate_id[cid][0] ][ uv_integrate_id[cid][1] ];
                const OpenMesh::Vec3f &b = f_ji[ uv_integrate_id[cid][0] ][ uv_integrate_id[cid][1] ];
                const OpenMesh::Vec3f &c = f_ij[ uv_integrate_id[cid][2] ][ uv_integrate_id[cid][3] ];
                const OpenMesh::Vec3f &d = f_ji[ uv_integrate_id[cid][2] ][ uv_integrate_id[cid][3] ];
                const float lambda = lambdas[cid];

                // Compute SparseMatrix D_ab, D_cd in ZJW's note
                
                SpMat D_ab_T(n6, 3);
                SpMat D_cd_T(n6, 3);
                // Boundary condition: one of the face(prisms)'s velocity(w, v) should be zero 
                D_ab_T.reserve(Eigen::VectorXi::Constant(3, std::min(10, n6)));
                D_cd_T.reserve(Eigen::VectorXi::Constant(3, std::min(10, n6)));

                // fill_in_D_T(a, b, f_i_id, f_j_id, D_ab_T);
                // fill_in_D_T(c, d, f_i_id, f_j_id, D_cd_T);
                fill_in_D_T(a, f_i_id, true, D_ab_T);
                fill_in_D_T(c, f_i_id, true, D_cd_T);
                if(face_id_set.find(f_j_id) != face_id_set.end()){
                    // opposite is optimizable face, fill it in
                    fill_in_D_T(b, f_j_id, false, D_ab_T);
                    fill_in_D_T(d, f_j_id, false, D_cd_T);
                }
                //////////////////////////////////////////////////////////////////////////////
                std::cout<< "D_ab_T:\n" << D_ab_T << std::endl;
                std::cout<< "D_cd_T:\n" << D_cd_T << std::endl;
                //////////////////////////////////////////////////////////////////////////////
                SpMat D_ab = D_ab_T.transpose();
                SpMat D_cd = D_cd_T.transpose();
                // add contribution to -A^T ("b" in "Ax = b")
                OpenMesh::Vec3f lambda_b_minus_a_OpenMesh = lambda * (b - a);
                OpenMesh::Vec3f lambda_d_minus_c_OpenMesh = lambda * (d - c);
                Eigen::Vector3f lambda_b_minus_a_Eigen;
                Eigen::Vector3f lambda_d_minus_c_Eigen;
                lambda_b_minus_a_Eigen(0) = lambda_b_minus_a_OpenMesh[0];
                lambda_b_minus_a_Eigen(1) = lambda_b_minus_a_OpenMesh[1];
                lambda_b_minus_a_Eigen(2) = lambda_b_minus_a_OpenMesh[2];
                
                lambda_d_minus_c_Eigen(0) = lambda_d_minus_c_OpenMesh[0];
                lambda_d_minus_c_Eigen(1) = lambda_d_minus_c_OpenMesh[1];
                lambda_d_minus_c_Eigen(2) = lambda_d_minus_c_OpenMesh[2];
                negA_T += D_cd_T * lambda_b_minus_a_Eigen + D_ab_T * lambda_d_minus_c_Eigen;
                // add contribution to B + B^T ("A" in "Ax = b")
                B_add_BT += D_ab_T * D_cd + D_cd_T * D_ab;
            }
        }
    }
    
}
void PrimoMeshViewer::global_optimize_faces(const std::vector<OpenMesh::FaceHandle> &face_handles, 
										const std::unordered_set<int> &face_idx_set)
{
    /* #TODO[ZJW][QYZ]: Here we are using Eigen to solve the SPD(symmetric positive definite) linear system.
       if it is the speed bottleneck, try SuiteSparse(even cuda - GPU implementation) with more pain :)
       Also, here we are using float instead of double, need to make sure it is sufficient. 
    */
    /* build the linear system. Here we follow the convention in 
       [PLH02]:http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=F1D81AF3257335BC6E902048DF161317?doi=10.1.1.6.5569&rep=rep1&type=pdf
       (B + B^T) C = -A^T. #TODO[ZJW]: add link of supplemental notes for explanation
    */
    int n6 = (int)face_handles.size() * 6;
    // -A^T ("b" in "Ax = b"), it is init to zero.
    Eigen::VectorXf negA_T = Eigen::VectorXf::Zero(n6);
    // B + B^T ("A" in "Ax = b")
    SpMat B_add_BT(n6, n6);
    
    //  
    build_problem_Eigen(n6, mesh_, P_PrismProperty, face_handles, face_idx_set, B_add_BT, negA_T);
    //////////////////////////////////////////////////////////////////////////////
    std::cout<< "B_and_BT:\n" <<  B_add_BT<<std::endl;
    std::cout<< "-A^T:\n" << negA_T << std::endl;
    //////////////////////////////////////////////////////////////////////////////
    // solve the linear system 
    // #TODO[ZJW]: need look at the other Cholesky factorization in Eigen 
    Eigen::SparseLU<SpMat> solver;  // performs a Cholesky factorization of A
    solver.compute(B_add_BT);
    // decompose should be success
    assert(solver.info() == Eigen::Success);

    Eigen::VectorXf x = solver.solve(negA_T);    // use the factorization to solve for the given right hand side
    std::cout<< x << std::endl;
    // #TODO[ZJW]: update vertices position based on faces(prisms) around each vertex
   
    // update OpenMesh's normals
    mesh_.update_normals(); 
}
