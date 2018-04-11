#include "PrimoMeshViewer.h"
#include <Eigen/Sparse>
#include <unordered_set>
typedef Eigen::SparseMatrix<float> SpMat;
typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;

// helper funtion to build linear system
static void build_problem_Eigen(const Mesh &mesh, const OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty, 
                            const std::vector<OpenMesh::FaceHandle> &face_handles, SpMat &B_add_BT, 
                            Eigen::VectorXf &negA_T){
    std::unordered_set<int> he_id_set;
    // static const float one_ninth = 1.0f / 9.0f; 
    
    for(int i = 0; i < face_handles.size(); ++i){
        // iterate all faces
        const OpenMesh::FaceHandle &fh_i = face_handles[i];
        const int f_i_id = fh_i.idx();
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

                // #TODO[ZJW]: add contribution to -A^T ("b" in "Ax = b")
                // #TODO[ZJW]: add contribution to B + B^T ("A" in "Ax = b")
            }
        }
    }
    
}
void PrimoMeshViewer::global_optimize_faces(std::vector<OpenMesh::FaceHandle> &face_handles)
{
    /* #TODO[ZJW][QYZ]: Here we are using Eigen to solve the SPD(symmetric positive definite) linear system.
       if it is the speed bottleneck, try SuiteSparse(even cuda - GPU implementation) with more pain :)
       Also, here we are using float instead of double, need to make sure it is sufficient. 
    */
    /* build the linear system. Here we follow the convention in 
       [PLH02]:http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=F1D81AF3257335BC6E902048DF161317?doi=10.1.1.6.5569&rep=rep1&type=pdf
       (B + B^T) C = -A^T. #TODO[ZJW]: add link of supplemental notes for explanation
    */
    int n = (int)face_handles.size() * 6;
    // -A^T ("b" in "Ax = b")
    Eigen::VectorXf negA_T(n);
    // B + B^T ("A" in "Ax = b")
    SpMat B_add_BT(n, n);
    
    //  
    build_problem_Eigen(mesh_, P_PrismProperty, face_handles, B_add_BT, negA_T);
    // solve the linear system 
    // #TODO[ZJW]: need look at the other Cholesky factorization in Eigen 
    Eigen::SimplicialCholesky<SpMat> chol(B_add_BT);  // performs a Cholesky factorization of A
    Eigen::VectorXf x = chol.solve(negA_T);    // use the factorization to solve for the given right hand side
   
    // #TODO[ZJW]: update vertices position based on faces(prisms) around each vertex
   
    // update OpenMesh's normals
    mesh_.update_normals(); 
}
