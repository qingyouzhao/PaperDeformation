#include "Crease.hh"
Crease::Crease(const std::vector<OpenMesh::HalfedgeHandle> &other_he_handles, Mesh &mesh):
mesh_(mesh)
{
    // copy all edge handles into this crease
    this->he_handles_ = other_he_handles;
    foldable_ = true;
}
void Crease::set_prism_height(const float height, OpenMesh::HPropHandleT<PrismProperty> &P_PrismProperty){
    // only set prisms height of pair of halfedges
    for(const OpenMesh::HalfedgeHandle &he_i : he_handles_){
        Mesh::HalfedgeHandle he_j = mesh_.opposite_halfedge_handle(he_i);
        PrismProperty &prop = mesh_.property(P_PrismProperty, he_i);
        const OpenMesh::Vec3f dFrom = (prop.FromVertPrismUp - prop.FromVertPrismDown).normalized();
        const OpenMesh::Vec3f dTo = (prop.ToVertPrismUp - prop.ToVertPrismDown).normalized();
        const OpenMesh::Vec3f midFrom = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
        const OpenMesh::Vec3f midTo = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
        prop.FromVertPrismUp = midFrom + dFrom * height;
        prop.FromVertPrismDown = midFrom + dFrom * height;
        prop.ToVertPrismUp = midTo + dTo * height;
        prop.ToVertPrismDown = midTo - dTo * height;
        if (he_j.is_valid() && !mesh_.is_boundary(he_i)){
            PrismProperty &prop = mesh_.property(P_PrismProperty, he_j);
            const OpenMesh::Vec3f dFrom = (prop.FromVertPrismUp - prop.FromVertPrismDown).normalized();
            const OpenMesh::Vec3f dTo = (prop.ToVertPrismUp - prop.ToVertPrismDown).normalized();
            const OpenMesh::Vec3f midFrom = (prop.FromVertPrismUp + prop.FromVertPrismDown) * 0.5f;
            const OpenMesh::Vec3f midTo = (prop.ToVertPrismUp + prop.ToVertPrismDown) * 0.5f;
            prop.FromVertPrismUp = midFrom + dFrom * height;
            prop.FromVertPrismDown = midFrom + dFrom * height;
            prop.ToVertPrismUp = midTo + dTo * height;
            prop.ToVertPrismDown = midTo - dTo * height;
        }
    }
}


