#include "support_generator_debug.h"

using SyNSlicerEngine::GUI::SupportGeneratorDebug;

SupportGeneratorDebug::SupportGeneratorDebug(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions, vtkRenderer *p_renderer)
	: SupportGenerator(input_paritions)
	, m_drawer(p_renderer)
{

}

SupportGeneratorDebug::~SupportGeneratorDebug()
{

}

static int count = 0;

void SupportGeneratorDebug::generateSupportStructure()
{
	spdlog::info("Generate support structure!");
	m_paritions.revert();
	for (int partition_index = 0; partition_index < m_paritions.numberOfPartitions(); partition_index++) // from 0 means from top to down
	{
		SO::PrintingLayerCollection working_printing_layers = m_paritions[partition_index].getPrintingLayers();

		for (int layer_number = working_printing_layers.getNumberOfLayers() - 1; layer_number >= 1; --layer_number)
		{
			if (working_printing_layers.getLayer(layer_number - 1).getNumberOfContours() <= 0)
			{
				continue;
			};

			SO::PrintingLayer p_layer_i = working_printing_layers.getLayer(layer_number);
			SO::PrintingLayer p_layer_i_minus_1 = working_printing_layers.getLayer(layer_number - 1);

			SO::PolygonCollection contours = p_layer_i.getContours();
			SO::PolygonCollection projected_contours = contours.projectToOtherPlane(p_layer_i_minus_1.getSlicingPlane());

			SO::PolygonCollection support_contours = p_layer_i.getSupportStructureContours();
			SO::PolygonCollection support_projected_contours = support_contours.projectToOtherPlane(p_layer_i_minus_1.getSlicingPlane());

			SO::PolygonCollection contours_from_layer_i_minus_1 = p_layer_i_minus_1.getContours();
			SO::PolygonCollection offsetted_contours_from_layer_i_minus_1 = contours_from_layer_i_minus_1.getOffset(0.5 * side_step); 

			SO::PolygonCollection support_contours_from_layer_i_minus_1 = p_layer_i_minus_1.getSupportStructureContours();
			SO::PolygonCollection offsetted_support_contours_from_layer_i_minus_1 = support_contours_from_layer_i_minus_1.getOffset(0.5 * side_step);

			SO::PolygonCollection all_contours_from_layer_i_minus_1 = offsetted_contours_from_layer_i_minus_1;
			all_contours_from_layer_i_minus_1.addPolygons(offsetted_support_contours_from_layer_i_minus_1);

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
			support_contours_at_layer_i_minus_1 = convexhulls_of_contours.getDifference(offsetted_contours_from_layer_i_minus_1);

			if (partition_index == 3)
			{
				//m_drawer.drawPolygons(support_contours_at_layer_i_minus_1, "support_contours_at_layer_i_minus_1" + std::to_string(layer_number));
				//m_drawer.setColor("support_contours_at_layer_i_minus_1" + std::to_string(layer_number), 1, 0, 0);
			}

			p_layer_i_minus_1.setSupportStructureContours(support_contours_at_layer_i_minus_1);
			working_printing_layers.setPrintingLayer(layer_number - 1, p_layer_i_minus_1);
		}

		m_paritions[partition_index].setPrintingLayers(working_printing_layers);

		std::vector<SO::Polyhedron<CgalMesh_EPICK>> temp_polyhedrons;

		int index = partition_index + 1;
		if (m_paritions[partition_index].getBasePlane() != SO::Plane())
		{
			SO::PrintingLayer p_layer_i = working_printing_layers.getLayer(0);
			if (p_layer_i.getSupportStructureContours().numberOfPolygons() > 0)
			{
				SO::Plane slicing_plane = m_paritions[partition_index].getBasePlane();
				SO::PolygonCollection projected_contours = p_layer_i.getSupportStructureContours().projectToOtherPlane(slicing_plane);

				SO::PolygonCollection pos, neg;
				for (int i = partition_index - 1; i >= 0; i--)
				{
					if (projected_contours.clipWithPlane(m_paritions[i].getBasePlane(), pos, neg))
					{
						projected_contours = neg;
					}
				}

				while (projected_contours.clipWithPlane(m_paritions[index].getBasePlane(), pos, neg))
				{
					index += 1;
				}
				while (neg.numberOfPolygons() > 0)
				{
					index += 1;
					projected_contours.clipWithPlane(m_paritions[index].getBasePlane(), pos, neg);
				}
				generateSupportStructureForSupportStructure(projected_contours, m_paritions[index].getBasePlane(), temp_polyhedrons);	
			}
		}

		// clip temp_polyhedron and then store into the corresponding partition
		for (int i = partition_index + 1; i < index; i++)
		{
			CgalMesh_EPICK up_mesh;
			CgalMesh_EPICK low_mesh;
			for (int j = 0; j < temp_polyhedrons.size(); j++)
			{
				if (this->clipSupportStructure(temp_polyhedrons[j].getMesh(), up_mesh, low_mesh, m_paritions[i].getBasePlane()))
				{
					this->addSupportStructureofSupportStructureToParition(m_paritions[i], up_mesh);	
					temp_polyhedrons[j] = low_mesh;
				}
			}
		}

		for (int j = 0; j < temp_polyhedrons.size(); j++)
		{
			this->addSupportStructureofSupportStructureToParition(m_paritions[index], temp_polyhedrons[j]);
		}
	}
	m_paritions.revert();
}

void SupportGeneratorDebug::generateSupportStructureForSupportStructure(SO::PolygonCollection &contours, const SO::Plane &plane, std::vector<SO::Polyhedron<CgalMesh_EPICK>> &support_structures)
{
	SO::Plane plane_above = contours.getPlane();
	SO::Plane plane_below = plane;
	SO::PolygonCollection &support_contours = contours;

	std::vector<SO::PolylineCollection> contours_of_support_structure_at_different_plane;

	SO::PolylineCollection polylines;
	for (size_t i = 0; i < contours.numberOfPolygons(); i++)
	{
		polylines.addPolyline(SO::Polyline(contours[i].get()));
	}

	contours_of_support_structure_at_different_plane.push_back(polylines);

	if (plane_above.getNormal().dot(plane_below.getNormal()) < cos(M_PI_4))
	{

		//! Find point in support_contours closest to plane_below;
		int p = 0;
		int q = 0;

		for (int i = 0; i < contours.numberOfPolygons(); i++)
		{
			for (int j = 0; j < contours[i].numberOfPoints(); j++)
			{
				if (plane_below.getDistanceFromPointToPlane(contours[i][j]) < plane_below.getDistanceFromPointToPlane(contours[p][q]))
				{
					p = i;
					q = j;
				}
			}
		}

		Eigen::Vector3d point_closest_to_plane_below = contours[p][q];
		// 

		Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
		transformation_matrix = Eigen::AngleAxis<double>(M_PI_4, plane_below.getAxisOfRotation(plane_above));

		Eigen::Vector3d new_plane_normal = transformation_matrix.linear() * plane_below.getNormal();

		SO::Plane middle_plane(point_closest_to_plane_below, new_plane_normal);

		SO::PolylineCollection contours_1;
		for (int i = 0; i < contours.numberOfPolygons(); i++)
		{
			SO::Polyline temp_contour;
			for (int j = 0; j < contours[i].numberOfPoints(); j++)
			{
				Eigen::Vector3d projected_point = middle_plane.getProjectionOfPointOntoPlane(contours[i][j]);
				temp_contour.push_back(projected_point);
			}
			contours_1.addPolyline(temp_contour);
		}
		contours_of_support_structure_at_different_plane.push_back(contours_1);
	}

	SO::PolylineCollection contours_2;
	SO::PolylineCollection &temp_contours = contours_of_support_structure_at_different_plane.back();
	for (int i = 0; i < temp_contours.size(); i++)
	{
		SO::Polyline temp_contour;
		for (int j = 0; j < temp_contours[i].numberOfPoints(); j++)
		{
			Eigen::Vector3d projected_point = plane_below.getProjectionOfPointOntoPlane(temp_contours[i][j]);
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
		if (generatePolyhedronFromContours(contours_of_support_structure[i], temp_polyhedron) == true)
		{
			//m_drawer.drawMesh(temp_polyhedron.getMesh(), "Mesh" + std::to_string(m_debug));
			//m_drawer.setColor("Mesh" + std::to_string(m_debug), 1, 0, 0);
			//m_debug += 1;
		}
		//! Check if the support_contours intersects with plane_below.
		
		support_structures.push_back(temp_polyhedron);
	}
}

void SupportGeneratorDebug::addSupportStructureofSupportStructureToParition(
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

	parition.setPrintingLayers(printing_layers);
}

void SupportGeneratorDebug::addContoursToPrintingLayer(
	SO::PolygonCollection &contours, SO::PrintingLayer &printing_layer)
{
	if (printing_layer.getNumberOfContours() <= 0)
	{
		printing_layer.addSupportStructureContours(contours);
		return;
	}

	SO::PolygonCollection contours_to_be_added = contours.getDifference(printing_layer.getContours());
	printing_layer.addSupportStructureContours(contours_to_be_added);
}