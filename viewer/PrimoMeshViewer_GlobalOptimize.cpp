#include "PrimoMeshViewer.h"
#include <Eigen/Sparse>
typedef Eigen::SparseMatrix<float> SpMat;
// helper funtion to build linear system
static void build_problem_Eigen(const std::vector<OpenMesh::FaceHandle> &face_handles, SpMat &B, 
                                Eigen::VectorXf &negA_T){
    //
    //                
}
void PrimoMeshViewer::global_optimize_all_faces(std::vector<OpenMesh::FaceHandle> &face_handles)
{
    /* #TODO[ZJW][QYZ]: Here we are using Eigen to solve the SPD(symmetric positive definite) linear system.
       if it is the speed bottleneck, try SuiteSparse(even cuda - GPU implementation) with more pain :)
       Also, here we are using float instead of double, need to make sure it is sufficient. 
    */
    /* build the linear system. Here we follow the convention in 
       [PLH02]:http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=F1D81AF3257335BC6E902048DF161317?doi=10.1.1.6.5569&rep=rep1&type=pdf
       BC = -A^T. #TODO[ZJW]: supplemental note for explanation
    */
    int n = (int)face_handles.size() * 6;
    // -A^T, "b" in "Ax=b"
    Eigen::VectorXf negA_T(n);
    // B, "A" in "Ax=b"
    SpMat B(n, n);
    
    //  
    build_problem_Eigen(face_handles, B, negA_T);
    // solve the linear system 
    // #TODO[ZJW]: need look at the other Cholesky factorization in Eigen 
    Eigen::SimplicialCholesky<SpMat> chol(B);  // performs a Cholesky factorization of A
    Eigen::VectorXf x = chol.solve(negA_T);    // use the factorization to solve for the given right hand side
   
    // #TODO[ZJW]: update vertices position based on faces(prisms) around each vertex
   
    // update OpenMesh's normals
    mesh_.update_normals(); 
}
