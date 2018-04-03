#include "PrimoMeshViewer.h"



PrimoMeshViewer::PrimoMeshViewer(const char* _title, int _width, int _height)
	: MeshViewer(_title, _width, _height)
{
	mesh_.request_vertex_colors();

	mesh_.add_property(P_PrismProperty);

	add_draw_mode("Visualize Prisms");

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
		return true;
	}
	return false;
}

void PrimoMeshViewer::draw(const std::string& _draw_mode)
{
	if (indices_.empty())
	{
		MeshViewer::draw(_draw_mode);
		return;
	}

	if (_draw_mode == "Vertex Valences")
	{

		glDisable(GL_LIGHTING);
		glShadeModel(GL_SMOOTH);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		GL::glVertexPointer(mesh_.points());
		GL::glNormalPointer(mesh_.vertex_normals());
		GL::glColorPointer(mesh_.vertex_colors());
		glDepthRange(0.01, 1.0);
		glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);
		glColor3f(0.1, 0.1, 0.1);
		glEnableClientState(GL_VERTEX_ARRAY);
		GL::glVertexPointer(mesh_.points());
		glDrawBuffer(GL_BACK);
		glDepthRange(0.0, 1.0);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDepthFunc(GL_LEQUAL);
		glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
		glDisableClientState(GL_VERTEX_ARRAY);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDepthFunc(GL_LESS);
	}

	else MeshViewer::draw(_draw_mode);
}


void PrimoMeshViewer::keyboard(int key, int x, int y)
{
	// Let super handle us first
	GlutViewer::keyboard(key, x, y);
	switch (key)
	{

	default:
		break;
	}
}

void PrimoMeshViewer::motion(int x, int y)
{
	GlutViewer::motion(x, y);
}

void PrimoMeshViewer::mouse(int button, int state, int x, int y)
{
	GlutViewer::mouse(button, state, x, y);
	// Then we handle this 
}

void PrimoMeshViewer::setup_prisms(EPrismExtrudeMode PrismExtrudeMode /*= EPrismExtrudeMode::FACE_NORMAL*/)
{
	for (Mesh::FaceIter f_iter = mesh_.faces_begin(); f_iter!= mesh_.faces_end(); f_iter++)
	{
		Mesh::FaceHalfedgeCWIter fh_cwit = mesh_.fh_cwbegin(*f_iter);
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

				prop.FromVertPrismDir = n0;
				prop.ToVertPrimsDir = n1;
				prop.FromVertPrismSize = prop.ToVertPrismSize = 1.0f;
				mesh_.property(P_PrismProperty, *fh_cwit) = prop;
			}
				break;
			case EPrismExtrudeMode::FACE_NORMAL:
			{
				PrismProperty prop;
				prop.FromVertPrismDir = mesh_.normal(*fh_cwit);
				prop.ToVertPrimsDir = mesh_.normal(*fh_cwit);
				prop.FromVertPrismSize = 1.0f;
				prop.ToVertPrismSize = 1.0f;
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

void PrimoMeshViewer::manipulate(Mesh::VertexHandle vh_, Mesh::Point target_location)
{

}

void PrimoMeshViewer::local_optimize(int iterations)
{
	//TODO: Randomly sample one face
	Mesh::FaceHandle fh;
	//TODO: optimize for that face
	local_optimize_face(fh);
}

void PrimoMeshViewer::local_optimize_face(Mesh::FaceHandle _fh)
{
	//TODO: optimize for this face
}

void PrimoMeshViewer::global_optimize_all_faces(int iterations)
{

}

