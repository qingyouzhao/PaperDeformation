#pragma once
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "Transformation.hh"
/// This struct store the prism data structure and provides basic functionalities for retrieval and claculation
struct PrismProperty {
	// Vec3f FromVertPrismDir_DEPRECATED;
	// float FromVertPrismSize_DEPRECATED;
	// Vec3f ToVertPrimsDir_DEPRECATED;
	// float ToVertPrismSize_DEPRECATED;

	OpenMesh::Vec3f FromVertPrismUp;
	OpenMesh::Vec3f FromVertPrismDown;
	OpenMesh::Vec3f ToVertPrismUp;
	OpenMesh::Vec3f ToVertPrismDown;


	// This weight is set up in setup_prisms() in the beginning,
	// and it is NEVER changed when deformation happens
	float weight_ij;

	// A simple illustration of how the prism is stored
	/*
	10---------11 ^       10------------11   ^
	| f_ij     |  |       |  f_ji       |    |
	f-he_ij-->to  |       to<--he_ji---from  |
	00---------01 normal  00------------01   normal
	*/

	// A simple illustration of how the prism is stored
	/*
	From up		
	01---------11 ^       01------------11   ^
	| f_ij     |  |       |  f_ji       |    |
	f-he_ij-->to  |       to<--he_ji---from  |
	00---------10 normal  00------------10   normal
	*/

	inline const OpenMesh::Vec3f& f_uv(int u, int v, bool is_i_j) const
	{
		// We only care uv edges
		assert(u == 0 || u == 1);
		assert(v == 0 || v == 1);
		if (is_i_j)
		{
			return (v == 1) ? ( u == 0 ? FromVertPrismUp : ToVertPrismUp)
							  : ( u == 0 ? FromVertPrismDown : ToVertPrismDown);
		}
		else
		{
			return (v == 1) ? (u == 0 ? ToVertPrismUp: FromVertPrismUp)
							  : (u == 0 ? ToVertPrismDown: FromVertPrismDown);
		}
	}

	OpenMesh::Vec3f TargetPosFrom()
	{
		return (FromVertPrismUp + FromVertPrismDown) * 0.5f;
	}
	OpenMesh::Vec3f TargetPosTo()
	{	
		return (ToVertPrismDown + ToVertPrismUp) * 0.5f;

	}

	void TransformPrism(const Transformation& Transform)
	{
		FromVertPrismUp = Transform * (FromVertPrismUp);
		FromVertPrismDown = Transform * (FromVertPrismDown);
		ToVertPrismUp = Transform * (ToVertPrismUp);
		ToVertPrismDown = Transform * (ToVertPrismDown);
	}

};
