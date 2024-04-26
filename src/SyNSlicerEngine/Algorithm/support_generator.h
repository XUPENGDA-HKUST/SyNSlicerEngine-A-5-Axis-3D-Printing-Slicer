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

		//! Extract contours that are close to each other from contours one by one.
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

		//! Generate 3D model given some contours.
		/*!
			\param[in]	contours	The contours.
			\param[out]	polyhedron	The 3D model.
			\return \b True if 3D model is generated. \b False if 3D model is not generated.
		*/
		virtual bool generatePolyhedronFromContours(SO::PolylineCollection &contours, SO::Polyhedron<CgalMesh_EPICK> &polyhedron);

		//! Clip a 3d model with a plane.
		/*!
			\param[in]	sm			The 3D model.
			\param[out]	sm_U		The part on postive side of clip_plane.
			\param[out]	sm_L		The part on negative side of clip_plane.
			\param[in]	clip_plane	The clipping plane.
			\return \b True if sm_U and sm_L are not empty. \b False if sm_U or sm_L is empty.
		*/
		virtual bool clipSupportStructure(CgalMesh_EPICK sm, CgalMesh_EPICK &sm_U, CgalMesh_EPICK &sm_L, const SO::Plane &clip_plane);
		
		//! Slice the 3d model of support structure and added the contours to partition.
		/*!
			\param[in,out]	parition			The partition.
			\param[in]		support_structure	The 3D model of the support structure.
		*/
		virtual void addSupportStructureofSupportStructureToParition(SO::Partition<CgalMesh_EPICK> &parition, const SO::Polyhedron<CgalMesh_EPICK> &support_structure);

		//! Slice the 3d model of support structure and added the contours to partition.
		/*!
			\param[in]		contours		The contours of the support structure.
			\param[in,out]	printing_layer	The printing layer that the contours add to.
		*/
		virtual void addContoursToPrintingLayer(SO::PolygonCollection &contours, SO::PrintingLayer &printing_layer);

		//! Distance between two consecutive contours.
		double side_step;

		//! Container store all partitions.
		SO::PartitionCollection<CgalMesh_EPICK> &m_paritions;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_SUPPORTGENERATOR_H_