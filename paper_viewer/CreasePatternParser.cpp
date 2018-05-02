#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>

#include "CreasePatternParser.h"
#include "libigl/include/igl/delaunay_triangulation.h"
#include "libigl/include/igl/triangle/triangulate.h"
#include "MeshViewer.hh"
#include "BezierCurve.h"


const int CONTOUR		= 1;
const int MOUNTAIN		= 2;
const int VALLEY	    = 3;
void CreasePatternParser::igl_example() const
{

	// Input polygon
	Eigen::MatrixXd V;
	Eigen::MatrixXi E;
	Eigen::MatrixXd H;

	// Triangulated interior
	Eigen::MatrixXd V2;
	Eigen::MatrixXi F2;
	// Create the boundary of a square
	V.resize(8, 2);
	E.resize(8, 2);
	H.resize(1, 2);

	V << -1, -1, 1, -1, 1, 1, -1, 1,
		-2, -2, 2, -2, 2, 2, -2, 2;

	E << 0, 1, 1, 2, 2, 3, 3, 0,
		4, 5, 5, 6, 6, 7, 7, 4;

	H << 0, 0;

	// Triangulate the interior
	igl::triangle::triangulate(V, E, H, "a0.005q", V2, F2);
}

bool CreasePatternParser::is_point_on_segment(const Vector3f& point, const Vector3f& start, const Vector3f& end) const
{
	float seg_len = length(start - end);
	if ((point - start).is_nearly_zero() || (point - end).is_nearly_zero()) { 
		return true; }
	Vector3f dir = (end - start).get_normalized();
	float cos_start = (point - start).get_normalized() | dir;
	float cos_end = (end - point).get_normalized() | dir;
	return cos_start * cos_end > 0.99f;
}

void CreasePatternParser::read_crease_pattern(const std::string& filename)
{
	// check if file type is properly .cpx
	std::string extension = ".cpx";
	// open file
	std::ifstream ifs(filename);
	if (ifs)
	{
		// read line by line
		crease_edge_data.clear();
		std::string line;
		// assume line is a single line of crease pattern
		int crease_count = 0;
		while (std::getline(ifs, line))
		{
			CreaseEdge e = read_line(line);
			crease_edge_data.push_back(e);
		}
	}
	else
	{
		std::cout << "file is not found " << filename << std::endl;
	}
	std::cout << "finished parsing crease pattern" << filename << std::endl;
	std::cout << "Total crease found " << crease_edge_data.size() << std::endl;
}


void CreasePatternParser::crease_pattern_to_open_mesh(Mesh& out_mesh, std::vector<std::vector<Mesh::HalfedgeHandle>>& out_hehs, std::vector<int>& out_crease_types)
{
	if (crease_edge_data.size() == 0)
	{
		std::cout << "there is not enough data, read crease pattern first" << std::endl;
		return;
	}

	// based on current data, triangulate
	triangulate(out_mesh);

	out_hehs.clear();
	// with the newly genrated mesh, we should populate the creases;
	for (CreaseEdge ce : crease_edge_data)
	{
		if (ce.type == MOUNTAIN || ce.type == VALLEY)
		{
			// let's try to find our out creases
			std::vector<Vector3f> crease_control_points;

			std::vector<Mesh::HalfedgeHandle> crease_hehs; // zqy: this is the list for this crease
			std::vector<Mesh::Point> crease_points_on_mesh;
			// init parameters
			const Vector3f start = ce.start;
			Mesh::Point start_point(start[0], start[1], start[2]);
			const Vector3f end = ce.end;
			Mesh::Point end_point(end[0], end[1], end[2]);

			Mesh::VertexHandle crease_start_vh = MeshViewer::get_closes_vertex_handle(out_mesh, start_point);
			Mesh::VertexHandle crease_end_vh = MeshViewer::get_closes_vertex_handle(out_mesh, end_point);
			crease_control_points.push_back(start);
			crease_control_points.insert(crease_control_points.begin()+1, ce.control_points.begin(),ce.control_points.end());
			crease_control_points.push_back(end);
			switch (crease_control_points.size())
			{
			case 4:
			{
				// shortest
				std::cout << "start searching crease halfedges." << std::endl;

				Mesh::VertexHandle seg_end_vh = crease_start_vh;
				Mesh::VertexHandle seg_start_vh = crease_start_vh;
				BezierCurve<float> bezier_curve(crease_control_points);
				
				const Mesh::Point& ssp = out_mesh.point(crease_start_vh);
				Vector3f segment_start(ssp[0],ssp[1],ssp[2]);
				Vector3f segment_end(ssp[0], ssp[1], ssp[2]);
				// I am just going to search based on the crease points and segment
				for (int i = 0; i <= ce.segments; i++)
				{
					// try to find the half edge based on the segment point count
					segment_start = segment_end;
					seg_start_vh = seg_end_vh;
					if(i < ce.segments)
					{
						Vector3f segment_pos = bezier_curve.Eval((double)(i + 0.5) / ce.segments);
						OpenMesh::Vec3f seg_pos_om(segment_pos[0], segment_pos[1], segment_pos[2]);
						seg_end_vh = MeshViewer::get_closes_vertex_handle(out_mesh, seg_pos_om);
						segment_end = segment_pos;
					}
					else
					{
						// progress to the end
						seg_end_vh = crease_end_vh;
						segment_end = ce.end;
					}
					Vector3f seg_dir = (segment_end - segment_start).get_normalized();
					// define our segments
					// prepare a vector to store half edges
					std::vector<HalfedgeHandle> hehs_start_to_end;
					std::vector<Mesh::Point> points_start_to_end;
					bool reach_seg_end = false;
					Mesh::VertexHandle current_search_vh = seg_start_vh;
					Mesh::HalfedgeHandle last_he;
					while (!reach_seg_end)
					{
						bool can_continue = true;
						float max_cos = 0;
						for (Mesh::VertexOHalfedgeCCWIter cvohe_iter = out_mesh.voh_ccwbegin(current_search_vh); cvohe_iter.is_valid(); cvohe_iter++)
						{
							// check if the half edge is on he segment, if yes, cache it and advance
							// avoid cycle
							Mesh::HalfedgeHandle this_he = *cvohe_iter;
							Mesh::HalfedgeHandle this_he_opp = out_mesh.opposite_halfedge_handle(*cvohe_iter);
							if ( this_he_opp== last_he) 
								continue;
							Mesh::Point from = out_mesh.point(out_mesh.from_vertex_handle(*cvohe_iter));
							Mesh::Point to = out_mesh.point(out_mesh.to_vertex_handle(*cvohe_iter));

							Vector3f he_dir(to[0] - from[0], to[1]-from[1], to[2] - from[2]);
							he_dir.normalize();
							float cos_he_dir = he_dir | seg_dir;
							if (max_cos < cos_he_dir)
							{
								max_cos = cos_he_dir;
							}
							else
							{
								continue;
							}

							if (/*is_point_on_segment(from, segment_start, segment_end) && is_point_on_segment(to, segment_start, segment_end) && */
								cos_he_dir > 0.99999f)
							{
								hehs_start_to_end.push_back(*cvohe_iter); // temp storage 
								crease_hehs.push_back(*cvohe_iter); // all crease storage
								current_search_vh = out_mesh.to_vertex_handle(*cvohe_iter); 
								last_he = *cvohe_iter;
								points_start_to_end.push_back(to);
								can_continue = true;
								break;
							}
							can_continue = false;
						}
						assert(can_continue);
						reach_seg_end = current_search_vh == seg_end_vh;
					}
					std::cout << "Found " << hehs_start_to_end.size() << " half edges on segment"  << i << std::endl;
				}
				std::cout << "Found " << crease_hehs.size() << " half edges on this crease" << std::endl;
				std::cout << std::endl;

			}
			break;
			default:
			{
				std::cout << "number of crease control points not supported" << crease_control_points.size() << std::endl;
			}
				break;
			}

			out_hehs.push_back(crease_hehs);
			out_crease_types.push_back(ce.type);
		}
	}
}

void CreasePatternParser::triangulate(Mesh& out_mesh)
{
	// Input polygon
	Eigen::MatrixXd V;
	Eigen::MatrixXi E;
	Eigen::MatrixXd H;

	// populate V,E,H by the current points;
	std::vector<Vector3f>					points;
	std::vector<std::pair<int,int>>			edges;
	

	{
		// construct points to triangulate
		for (CreaseEdge ce : crease_edge_data)
		{
			int start_nv = points.size();
			int start_ne = edges.size();
			// add start
			// bool found = false;
			int from_idx = -1;
			int to_idx = -1;
			std::vector<Vector3f>::iterator it;
			it = std::find(points.begin(), points.end(), ce.start);
			if (it != points.end())
			{
				from_idx = to_idx = it - points.begin();
			}
			else
			{
				from_idx = to_idx = points.size();
				points.push_back(ce.start);
			}

			// add intermediate
			for (int i = 0; i < ce.segments; i++)
			{
				Vector3f p;
				std::vector<Vector3f> beizer_points;
				beizer_points.push_back(ce.start);
				if(ce.type == CONTOUR)
				{
					beizer_points.push_back(ce.start);
					beizer_points.push_back(ce.end);
				}
				else
				{
					beizer_points.push_back(ce.control_points[0]);
					beizer_points.push_back(ce.control_points[1]);
				}
				beizer_points.push_back(ce.end);
				BezierCurve<float> b(beizer_points);
				if (ce.type == CONTOUR)
				{
					// straight line
					double t = (double)(i + 0.5) / ce.segments;
					p = lerp(ce.start, ce.end, (float)t);
				}
				else
				{
					// curved lines
					double t = (double)(i + 0.5) / ce.segments;
					p = b.Eval(t);
				}

				std::vector<Vector3f>::iterator it;
				it = std::find(points.begin(), points.end(), p);
				if (it != points.end())
				{
					from_idx = to_idx;
					to_idx = it - points.begin();
				}
				else
				{
					from_idx = to_idx;
					to_idx = points.size();
					points.push_back(p);
				}
				edges.push_back(std::pair<int, int>(from_idx, to_idx));
			}

			// add end
			from_idx = to_idx;
			it = std::find(points.begin(), points.end(), ce.end);
			if (it != points.end())
			{
				to_idx = it - points.begin();
			}
			else
			{
				to_idx = points.size();
				points.push_back(ce.end);
			}
			edges.push_back(std::pair<int, int>(from_idx, to_idx));
			int end_nv = points.size();
			int end_ne = edges.size();
			std::cout << "finished crease parse, added " << end_nv - start_nv << " points and " << end_ne - start_ne << " edges " << std::endl;
			int hahaha = 0;
		}
	}


	// assume there is no holes

	int nv = points.size();
	int ne = edges.size();

	V.resize(nv, 2);
	E.resize(ne, 2);
	int v_c = 0;
	for (int i = 0; i < nv; i++)
	{
		std::cout << "Pushed v to eigen V" << i << points[i].to_string() << std::endl;
		// store xz of v
		V(i, 0) = points[i][0];
		V(i, 1) = points[i][2];
	}

	for (int i = 0; i < ne; i++)
	{
		std::cout << "Pushed e to Eigen E" << edges[i].first << "," << edges[i].second << std::endl;
		E(i, 0) = edges[i].first;
		E(i, 1) = edges[i].second;
	}

	// Triangulated interior
	Eigen::MatrixXd V2;
	Eigen::MatrixXi F2;
	igl::triangle::triangulate(V, E, H, "a0.005q", V2, F2);


	out_mesh.clear();
	// Just attemp to create a mesh here then
	std::vector<Mesh::VertexHandle> vhandle_cache;
	// add all vertexes
	int nv2 = V2.rows();

	for (int i = 0; i < nv2; i++)
	{
		Mesh::VertexHandle vh = out_mesh.add_vertex(Mesh::Point(V2(i, 0), 0, V2(i, 1)));
		vhandle_cache.push_back(vh);
	}
	int nf2 = F2.rows();
	for (int i = 0; i < nf2; i++)
	{
		std::vector<Mesh::VertexHandle> face_vhandles;
		if(i%100 == 0)
		{std::cout << "pushed " << i<< " faces" << std::endl;}
		face_vhandles.push_back(vhandle_cache[F2(i, 0)]);
		face_vhandles.push_back(vhandle_cache[F2(i, 1)]);
		face_vhandles.push_back(vhandle_cache[F2(i, 2)]);
		out_mesh.add_face(face_vhandles);
	}
}

CreaseEdge CreasePatternParser::read_line(const std::string& line)
{
	CreaseEdge result;
	result.control_points.clear();
	std::cout << "reading crease line" << std::endl;
	std::stringstream ss(line);

	ss >> result.type;
	// parse start and end types
	float start_x, start_y;
	ss >> start_x;
	ss >> start_y;
	float end_x, end_y;
	ss >> end_x;
	ss >> end_y;
	result.start = Vector3f(start_x, 0, start_y);
	result.end = Vector3f(end_x, 0, end_y);
	switch (result.type)
	{
	case 1:
	{
		result.segments = 0;// [ZJW] defaultly, contour only need 0 segment(no middle point)
	}
	break;
	case 2:
	case 3:
	{
		float p1_x, p1_y;
		float p2_x, p2_y;
		ss >> p1_x;
		ss >> p1_y;
		ss >> p2_x;
		ss >> p2_y;
		Vector3f p1 = Vector3f(p1_x, 0, p1_y);
		Vector3f p2 = Vector3f(p2_x, 0, p2_y);
		result.control_points.push_back(p1);
		result.control_points.push_back(p2);
		ss >> result.segments;
	}
	break;
	default:
		break;
	}

	// crease index should start from here
	std::cout << "finished reading crease line " << line << std::endl;
	return result;
}

