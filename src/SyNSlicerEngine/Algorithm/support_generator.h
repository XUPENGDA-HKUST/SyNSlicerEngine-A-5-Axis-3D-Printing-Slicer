#ifndef SYNSLICERENGINE_ALGORITHM_SUPPORTGENERATOR_H_
#define SYNSLICERENGINE_ALGORITHM_SUPPORTGENERATOR_H_

#include <clipper2/clipper.h>

#include <CGAL_CORE_CLASS>

#include "Object/partition_collection.h"
#include "Object/polygon_collection.h"
#include "Object/polyline_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm {

	//! This class is used to generate support structure.
	class SupportGenerator
	{
	public:
		SupportGenerator(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions);
		~SupportGenerator();

		virtual void generateSupportStructure();

	protected:
		
		virtual void splitAndGroupContours(SO::PolygonCollection &contours, std::vector<SO::PolygonCollection> &splited_contours, double epsilon);
		
		virtual bool findNeighourContours(std::vector<SO::Polygon> &contours,
			SO::PolygonCollection &contours_to_be_search, 
			std::vector<bool> &access_table, double epsilon);

		virtual bool extractContoursFromcontours(SO::PolygonCollection &input_contours, SO::PolygonCollection &output_contours, double epsilon);

		virtual void generateSupportStructureForSupportStructure(SO::PolygonCollection &contours,
			const SO::Plane &plane, std::vector<SO::Polyhedron<CgalMesh_EPICK>> &support_structures);

		virtual bool generatePolyhedronFromContours(SO::PolylineCollection &contours, SO::Polyhedron<CgalMesh_EPICK> &polyhedron);

		virtual void clipSupportStructure(CgalMesh_EPICK sm, CgalMesh_EPICK &sm_U, CgalMesh_EPICK &sm_L, const SO::Plane &clip_plane);
		virtual void addSupportStructureofSupportStructureToParition(SO::Partition<CgalMesh_EPICK> &parition, const SO::Polyhedron<CgalMesh_EPICK> &support_structure);
		virtual void addContoursToPrintingLayer(SO::PolygonCollection &contours, SO::PrintingLayer &printing_layer);

		double side_step;

		SO::PartitionCollection<CgalMesh_EPICK> &m_paritions;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_SUPPORTGENERATOR_H_