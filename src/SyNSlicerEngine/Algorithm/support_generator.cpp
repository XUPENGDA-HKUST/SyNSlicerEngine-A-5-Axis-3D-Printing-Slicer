#include "support_generator.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

using namespace Clipper2Lib;
using SyNSlicerEngine::Algorithm::SupportGenerator;

SupportGenerator::SupportGenerator(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions)
	: side_step(0.4)
	, m_paritions(input_paritions)
{
	spdlog::get("basic_logger")->info("Construct SupportGenerator");
}

SupportGenerator::~SupportGenerator()
{
	spdlog::get("basic_logger")->info("Destroy SupportGenerator");
}

static int count = 0;

void SupportGenerator::generateSupportStructure()
{
	spdlog::info("Generate support structure!");
	m_paritions.revert();
	for (int partition_index = 0; partition_index < m_paritions.numberOfPartitions(); partition_index++) // from 0 means from top to down
	{
		SO::PrintingLayerCollection working_printing_layers = m_paritions[partition_index].getPrintingLayers();

		for (int layer_number = working_printing_layers.getNumberOfLayers() - 1; layer_number >= 0; --layer_number)
		{
			working_printing_layers.getLayer(layer_number).getSupportStructureContours().reset();
		}

		for (int layer_number = working_printing_layers.getNumberOfLayers() - 1; layer_number >= 1; --layer_number)
		{
			//spdlog::info("{}", layer_number);

			if (working_printing_layers.getLayer(layer_number - 1).getNumberOfContours() <= 0)
			{
				continue;
			};

			SO::PrintingLayer p_layer_i = working_printing_layers.getLayer(layer_number);
			SO::PrintingLayer p_layer_i_minus_1 = working_printing_layers.getLayer(layer_number - 1);
			SO::Plane xy_plane(p_layer_i_minus_1.getSlicingPlane().getOrigin(), Eigen::Vector3d::UnitZ());

			SO::PolygonCollection contours = p_layer_i.getContours();
			SO::PolygonCollection projected_contours = contours.projectToOtherPlane(p_layer_i_minus_1.getSlicingPlane()); // Subject_0

			SO::PolygonCollection support_contours = p_layer_i.getSupportStructureContours();
			SO::PolygonCollection support_projected_contours = support_contours.projectToOtherPlane(p_layer_i_minus_1.getSlicingPlane()); // Subject_1

			SO::PolygonCollection contours_from_layer_i_minus_1 = p_layer_i_minus_1.getContours();
			contours_from_layer_i_minus_1 = contours_from_layer_i_minus_1.getOffset(0.5 * side_step); // clip


			count += 1;

			SO::PolygonCollection support_contours_from_layer_i_minus_1 = p_layer_i_minus_1.getSupportStructureContours();
			support_contours_from_layer_i_minus_1 = support_contours_from_layer_i_minus_1.getTransformedPolygons(xy_plane);

			SO::PolygonCollection all_contours_from_layer_i_minus_1 = contours_from_layer_i_minus_1; 
			all_contours_from_layer_i_minus_1.addPolygons(support_contours_from_layer_i_minus_1); // clip_1

			SO::PolygonCollection support_contours_at_layer_i_minus_1 = projected_contours.getDifference(all_contours_from_layer_i_minus_1);
			support_contours_at_layer_i_minus_1.addPolygons(support_projected_contours.getDifference(all_contours_from_layer_i_minus_1));
			support_contours_at_layer_i_minus_1.addPolygons(support_contours_from_layer_i_minus_1);

			// group solution_0 and solution_1 together;
			std::vector<SO::PolygonCollection> splited_contours;

			this->splitAndGroupContours(support_contours_at_layer_i_minus_1, splited_contours, 3 * side_step);

			SO::PolygonCollection convexhulls_of_contours;
			SO::Polygon temp_polyline;

			for (int i = 0; i < splited_contours.size(); i++)
			{
				convexhulls_of_contours.addPolygon(splited_contours[i].getConvexHullPolygon()); // subject_2
			}	

			support_contours_at_layer_i_minus_1.reset();
			support_contours_at_layer_i_minus_1 = convexhulls_of_contours.getDifference(contours_from_layer_i_minus_1);

			p_layer_i_minus_1.setSupportStructureContours(support_contours_at_layer_i_minus_1);
			working_printing_layers.setPrintingLayer(layer_number - 1, p_layer_i_minus_1);
		}

		std::vector<SO::Polyhedron<CgalMesh_EPICK>> temp_polyhedrons;

		if (m_paritions[partition_index].getBasePlane() != SO::Plane())
		{
			SO::PrintingLayer p_layer_i = working_printing_layers.getLayer(0);
			if (p_layer_i.getSupportStructureContours().numberOfPolygons() > 0)
			{
				SO::Plane slicing_plane = m_paritions[partition_index].getBasePlane();
				SO::PolygonCollection projected_contours = p_layer_i.getSupportStructureContours().projectToOtherPlane(slicing_plane);
				generateSupportStructureForSupportStructure(projected_contours, m_paritions[partition_index].getBasePlane(), temp_polyhedrons);
			}
		}

		// clip temp_polyhedron and then store into the corresponding partition
		for (int i = partition_index + 1; i < m_paritions.numberOfPartitions() - 1; i++)
		{
			CgalMesh_EPICK up_mesh;
			CgalMesh_EPICK low_mesh;
			for (int j = 0; j < temp_polyhedrons.size(); j++)
			{
				this->clipSupportStructure(temp_polyhedrons[j].getMesh(), up_mesh, low_mesh, m_paritions[i].getBasePlane());
				this->addSupportStructureofSupportStructureToParition(m_paritions[i], up_mesh);
				temp_polyhedrons[j] = low_mesh;
			}
		}

		for (int j = 0; j < temp_polyhedrons.size(); j++)
		{
			this->addSupportStructureofSupportStructureToParition(m_paritions.back(), temp_polyhedrons[j]);
		}

		m_paritions[partition_index].setPrintingLayers(working_printing_layers);
	}
	m_paritions.revert();
}

void SupportGenerator::splitAndGroupContours(SO::PolygonCollection &contours, std::vector<SO::PolygonCollection> &splited_contours, double epsilon)
{
	SO::PolygonCollection working_contours = contours;
	SO::PolygonCollection temp_contours;

	while (extractContoursFromcontours(working_contours, temp_contours, epsilon))
	{
		splited_contours.push_back(temp_contours);
	}
	splited_contours.push_back(temp_contours);
}

bool SupportGenerator::extractContoursFromcontours(SO::PolygonCollection &input_contours,
	SO::PolygonCollection &output_contours, double epsilon)
{
	// Check if input_contours is empty.
	if (input_contours.numberOfPolygons() <= 0)
	{
		return false;
	}

	// Find first non empty polygon in input_contours.
	int index = 0;
	while (input_contours[index].numberOfPoints() == 0)
	{
		++index;
		if (index >= input_contours.numberOfPolygons())
		{
			return false;
		}
	}

	std::vector<bool> access_table_of_contours;
	access_table_of_contours.resize(input_contours.numberOfPolygons());

	std::vector<SO::Polygon> contours_for_start_searching;
	contours_for_start_searching.push_back(input_contours[index]);
	access_table_of_contours[index] = true;

	while (findNeighourContours(contours_for_start_searching, input_contours, access_table_of_contours, epsilon))
	{

	}

	SO::PolygonCollection temp_contours;
	output_contours.reset();

	for (int i = 0; i < access_table_of_contours.size(); i++)
	{
		if (access_table_of_contours[i] == true)
		{
			output_contours.addPolygon(input_contours[i]);
		}
		else
		{
			temp_contours.addPolygon(input_contours[i]);
		}
	}

	input_contours = temp_contours;
	if (input_contours.numberOfPolygons() > 0)
	{
		// Some contours are waiting for grouping.
		return true;
	}
	else
	{
		// No contours are waiting for grouping.
		return false;
	}
}

bool SupportGenerator::findNeighourContours(std::vector<SO::Polygon> &contours, SO::PolygonCollection &contours_to_be_search, std::vector<bool> &access_table, double epsilon)
{
	std::vector<SO::Polygon> contours_used_in_the_next_searching;
	double distance = 0.0;
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours_to_be_search.numberOfPolygons(); j++)
		{	
			distance = contours[i].getMinimumDistanceFromPolygon(contours_to_be_search[j]);
			if (distance <= epsilon && access_table[j] == false)
			{
				access_table[j] = true;
				contours_used_in_the_next_searching.emplace_back(contours_to_be_search[j]);
			}
		}
	}
	if (contours_used_in_the_next_searching.size() != 0)
	{
		contours = contours_used_in_the_next_searching;
		return true;
	}
	return false;
}

void SupportGenerator::generateSupportStructureForSupportStructure(SO::PolygonCollection &contours, 
	const SO::Plane &plane, std::vector<SO::Polyhedron<CgalMesh_EPICK>> &support_structures)
{
	// Step 1: check plane normal.

	std::vector<SO::PolylineCollection> contours_of_support_structure_at_different_plane;

	SO::PolylineCollection polylines;
	for (size_t i = 0; i < contours.numberOfPolygons(); i++)
	{
		polylines.addPolyline(SO::Polyline(contours[i].get()));
	}

	contours_of_support_structure_at_different_plane.push_back(polylines);

	if (plane.getNormal().dot(Eigen::Vector3d(0, 0, 1)) < cos(M_PI_4))
	{
		int p = 0;
		int q = 0;

		for (int i = 0; i < contours.numberOfPolygons(); i++)
		{
			for (int j = 0; j < contours[i].numberOfPoints(); j++)
			{
				if (contours[i][j][2] < contours[p][q][2])
				{
					p = i;
					q = j;
				}
			}
		}

		Eigen::Vector3d lowest_point = contours[p][q];
		
		Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
		SO::Plane built_plate;
		transformation_matrix = Eigen::AngleAxis<double>(
			M_PI_4,
			built_plate.getAxisOfRotation(plane));

		Eigen::Vector3d new_plane_normal = transformation_matrix.linear() * Eigen::Vector3d(0, 0, 1);

		SO::Plane forty_five_degree_plane(lowest_point, new_plane_normal);

		SO::PolylineCollection contours_1;
		for (int i = 0; i < contours.numberOfPolygons(); i++)
		{
			SO::Polyline temp_contour;
			for (int j = 0; j < contours[i].numberOfPoints(); j++)
			{
				Eigen::Vector3d projected_point = forty_five_degree_plane.getProjectionOfPointOntoPlane(contours[i][j]);
				temp_contour.push_back(projected_point);
			}
			contours_1.addPolyline(temp_contour);
		}
		contours_of_support_structure_at_different_plane.push_back(contours_1);
	}

	SO::PolylineCollection contours_2;
	SO::Plane built_plate;
	SO::PolylineCollection &temp_contours = contours_of_support_structure_at_different_plane.back();
	for (int i = 0; i < temp_contours.size(); i++)
	{
		SO::Polyline temp_contour;
		for (int j = 0; j < temp_contours[i].numberOfPoints(); j++)
		{
			Eigen::Vector3d projected_point = built_plate.getProjectionOfPointOntoPlane(temp_contours[i][j]);
			temp_contour.push_back(projected_point);
		}
		contours_2.addPolyline(temp_contour);
	}
	contours_of_support_structure_at_different_plane.emplace_back(contours_2);

	std::vector<SO::PolylineCollection> contours_of_support_structure;
	for (int i = 0; i < contours_of_support_structure_at_different_plane.front().size(); i++)
	{
		contours_of_support_structure.emplace_back(SO::PolylineCollection());
	}

	for (int i = 0; i < contours_of_support_structure_at_different_plane.size(); i++)
	{
		for (int j = 0; j < contours_of_support_structure_at_different_plane[i].size(); j++)
		{
			contours_of_support_structure[j].addPolyline(contours_of_support_structure_at_different_plane[i][j]);
		}
	}

	for (int i = 0; i < contours_of_support_structure.size(); i++)
	{
		SO::Polyhedron<CgalMesh_EPICK> temp_polyhedron;
		generatePolyhedronFromContours(contours_of_support_structure[i], temp_polyhedron);
		support_structures.push_back(temp_polyhedron);
	}
}

bool SupportGenerator::generatePolyhedronFromContours(SO::PolylineCollection &contours,
	SO::Polyhedron<CgalMesh_EPICK> &polyhedron)
{
	if (contours.size() < 2)
	{
		return false;
	}

	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].numberOfPoints()!= contours[0].numberOfPoints())
		{
			return false;
		}
	}

	CgalMesh_EPICK support_structure;
	int number_of_contours = contours.size();
	int contour_size = contours[0].numberOfPoints();

	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].numberOfPoints(); j++)
		{
			support_structure.add_vertex(CgalPoint_EPICK(contours[i][j][0], contours[i][j][1], contours[i][j][2]));
		}
	}

	int a = 0, b = 0;

	for (int i = 0; i < number_of_contours - 1; i++)
	{
		for (int j = 0; j < contour_size; j++)
		{
			a = i * contour_size + j;
			b = (i + 1) * contour_size + j;
			if (j == contour_size - 1)
			{
				support_structure.add_face(CgalMesh_EPICK::Vertex_index(a), CgalMesh_EPICK::Vertex_index(b), CgalMesh_EPICK::Vertex_index(i * contour_size));
				support_structure.add_face(CgalMesh_EPICK::Vertex_index(i * contour_size), CgalMesh_EPICK::Vertex_index(b), CgalMesh_EPICK::Vertex_index((i + 1) * contour_size));
			}
			else
			{
				support_structure.add_face(CgalMesh_EPICK::Vertex_index(a), CgalMesh_EPICK::Vertex_index(b), CgalMesh_EPICK::Vertex_index(a + 1));
				support_structure.add_face(CgalMesh_EPICK::Vertex_index(a + 1), CgalMesh_EPICK::Vertex_index(b), CgalMesh_EPICK::Vertex_index(b + 1));
			}
		}
	}

	SO::Polyhedron<CgalMesh_EPICK> partition(support_structure);
	partition.makeAsCleanAsPossible();

	if (partition.fillHoles())
	{
		polyhedron = partition;
		spdlog::info("generatePolyhedronFromContours() return true!");
		return true;
	}
	else
	{
		spdlog::info("generatePolyhedronFromContours() return false!");
		return false;
	}
}

void SupportGenerator::clipSupportStructure(CgalMesh_EPICK sm, CgalMesh_EPICK &sm_U, 
	CgalMesh_EPICK &sm_L, const SO::Plane &clip_plane)
{
	if (clip_plane != SO::Plane())
	{
		sm_U = sm;
		sm_L = sm;
		CgalPlane_EPICK cgal_plane(clip_plane.a(), clip_plane.b(), clip_plane.c(), clip_plane.d());
		CGAL::Polygon_mesh_processing::clip(sm_L, cgal_plane, CGAL::parameters::clip_volume(true));
		CGAL::Polygon_mesh_processing::clip(sm_U, cgal_plane.opposite(), CGAL::parameters::clip_volume(true));
		sm_L.collect_garbage();
		sm_U.collect_garbage();
	}
}

void SupportGenerator::addSupportStructureofSupportStructureToParition(
	SO::Partition<CgalMesh_EPICK> &parition, const SO::Polyhedron<CgalMesh_EPICK> &support_structure)
{
	// slice the support structure here
	SO::PrintingLayerCollection printing_layers = parition.getPrintingLayers();
	CgalMesh_EPICK mesh = support_structure.getMesh();
	CGAL::Polygon_mesh_slicer<CgalMesh_EPICK, EPICK> slicer(mesh);
	SO::Plane slicing_plane;

	for (int i = 0; i < printing_layers.size(); i++)
	{
		SO::PrintingLayer working_printing_layer = printing_layers[i];
		slicing_plane = printing_layers[i].getSlicingPlane();
		std::vector<CgalPolyline_EPICK> polylines;
		slicer(EPICK::Plane_3(slicing_plane.a(), slicing_plane.b(), slicing_plane.c(), slicing_plane.d()), std::back_inserter(polylines));

		if (polylines.size() != 0)
		{
			SO::PolygonCollection contours;
			contours.setPlane(slicing_plane);
			for (int j = 0; j < polylines.size(); j++)
			{
				SO::Polygon contour;
				contour.setPlane(slicing_plane);
				for (size_t i = 0; i < polylines[j].size(); i++)
				{
					contour.addPointToBack(Eigen::Vector3d(polylines[j][i].x(), polylines[j][i].y(), polylines[j][i].z()));
				}
				contours.addPolygon(contour);
			}
			this->addContoursToPrintingLayer(contours, working_printing_layer);
			printing_layers.setPrintingLayer(i, working_printing_layer);
		}
	}

	slicing_plane.offset(0.3);
	std::vector<CgalPolyline_EPICK> polylines;
	slicer(EPICK::Plane_3(slicing_plane.a(), slicing_plane.b(), slicing_plane.c(), slicing_plane.d()), std::back_inserter(polylines));

	while (polylines.size() != 0)
	{
		SO::PolygonCollection temp_contours(polylines, slicing_plane);
		SO::PrintingLayer new_printing_layer(slicing_plane, printing_layers[printing_layers.size() - 1].getSlicingPlane());
		this->addContoursToPrintingLayer(temp_contours, new_printing_layer);
		printing_layers.addPrintingLayer(new_printing_layer);

		slicing_plane.offset(0.3);
		polylines.clear();
		slicer(CgalPlane_EPICK(slicing_plane.a(), slicing_plane.b(), slicing_plane.c(), slicing_plane.d()), std::back_inserter(polylines));
	};
}

void SupportGenerator::addContoursToPrintingLayer(
	SO::PolygonCollection &contours, SO::PrintingLayer &printing_layer)
{
	if (printing_layer.getNumberOfContours() <= 0)
	{
		printing_layer.addSupportStructureContours(contours);
		return;
	}

	SO::PolygonCollection contours_to_be_added = printing_layer.getContours().getDifference(contours);

	printing_layer.addSupportStructureContours(contours_to_be_added);
}
