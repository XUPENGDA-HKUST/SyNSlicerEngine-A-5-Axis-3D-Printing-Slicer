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
		//! Default constructor not allowed.
		SupportGenerator() = delete;

		//! Constructor.
		/*!
			\param input_paritions	All the partitions.
		*/
		SupportGenerator(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions);

		//! Destructor.
		~SupportGenerator();

		//! Call to generate support structure.
		virtual void generateSupportStructure();

	protected:
		//! Group contours that are close to each others into SO::PolygonCollection.
		/*!
			\param[in]	contours			The contours waiting to group.
			\param[out]	splited_contours	The contours grouped.
			\param[in]	epsilon				The distance defines close.
		*/
		virtual void splitAndGroupContours(SO::PolygonCollection &contours, std::vector<SO::PolygonCollection> &splited_contours, double epsilon);

		//! Extract contours from contours one by one.
		/*!
			\param[in,out]	input_contours		The contours waiting to group in this round.
			\param[out]		output_contours		The contours removed from input_contours in this round.
			\param[in]		epsilon				The distance defines close.
		*/
		virtual bool extractContoursFromcontours(SO::PolygonCollection &input_contours, SO::PolygonCollection &output_contours, double epsilon);

		//! Find contours that are close to input_contours.
		/*!
			\param[in]		contours				The seed contours used to search for close contours.
			\param[in]		contours_to_be_search	The contours waiting to group in this round.
			\param[in,out]	access_table			The container record whether the contours are grouped.
			\param[in]		epsilon					The distance defines close.
		*/
		virtual bool findNeighourContours(std::vector<SO::Polygon> &contours,
			SO::PolygonCollection &contours_to_be_search, 
			std::vector<bool> &access_table, double epsilon);

		//! Generate support structure for support strcuture
		/*!
			\param[in]		contours				The seed contours used to search for close contours.
			\param[in]		plane					The contours waiting to group in this round.
			\param[in,out]	support_structures		The container record whether the contours are grouped.
		*/
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