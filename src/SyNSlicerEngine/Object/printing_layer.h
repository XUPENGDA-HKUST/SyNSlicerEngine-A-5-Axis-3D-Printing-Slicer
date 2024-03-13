#ifndef SYNSLICERENGINE_OBJECT_PRINTINGLAYER_H_
#define SYNSLICERENGINE_OBJECT_PRINTINGLAYER_H_

#include <CGAL_CORE_CLASS>
#include <Eigen/Core>

#include "Object/line.h"
#include "Object/plane.h"
#include "Object/polyline.h"
#include "Object/polyline_collection.h"
#include "Object/polygon.h"
#include "Object/polygon_collection.h"
#include "Object/printing_path_collection.h"
#include "Object/toolpath_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//!  This class is used to store the toolpaths associated with the printing layers.
	/*!

	*/
	class PrintingLayer
	{
	public:
		PrintingLayer();
		PrintingLayer(const PrintingLayer &other);
		PrintingLayer(const SO::Plane &slicing_plane, const SO::Plane &prev_slicing_plane);
		PrintingLayer(const SO::PolygonCollection &contours, const SO::Plane &slicing_plane, const SO::Plane &prev_slicing_plane);
		~PrintingLayer();

		int getNumberOfContours();
		const Polyline &getContour(int index) const;
		PolylineCollection &getContours();

		void setSupportStructureContours(const PolylineCollection &input_support_structure_contours);
		void addSupportStructureContours(const PolylineCollection &input_support_structure_contours);
		PolylineCollection &getSupportStructureContours();

		Eigen::Vector3d &getOrigin();
		SO::Line &getDirection1();
		SO::Line &getDirection2();
		PrintingPathCollection &getPrintingPaths();
		PrintingPathCollection &getPrintingPathsForSupport();

		void setToolpaths(const ToolpathCollection &toolpaths);

		const ToolpathCollection &getToolpaths() const;
		void clearToolpath();

		SO::Plane getSlicingPlane();
		SO::Plane getPrevSlicingPlane();

		PrintingLayer &operator=(const PrintingLayer &other);

	private:

		//! m_contours store the outer contours
		PolylineCollection m_contours;
		PolylineCollection m_support_structure_contours;

		//! All the points in Contours should lie onto mp_slicing_plane;
		Plane m_slicing_plane;
		Plane m_prev_slicing_plane;

		//!
		Eigen::Vector3d m_origin_for_infill_alignment;
		SO::Line m_direction_1_for_infill_alignment;
		SO::Line m_direction_2_for_infill_alignment;

		PrintingPathCollection m_printing_paths;
		PrintingPathCollection m_printing_paths_support;

		//! m_toolpaths store the toolpath to printing this layer
		ToolpathCollection m_toolpaths;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_PRINTINGLAYER_H_