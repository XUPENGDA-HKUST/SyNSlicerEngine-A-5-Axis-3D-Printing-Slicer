#ifndef SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_

#include <vector>

#include "clipper2/clipper.h"
#include "Object/polygon.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//! Container used to store multiple polygons.
	class PolygonCollection
	{		
	public:
		//! Default constructor.
		PolygonCollection();

		//! Copy constructor.
		PolygonCollection(const PolygonCollection &other);

		//! Construct from Clipper2Lib::PathsD
		/*!
			\param[in]	polygons	Polygons from Clipper2Lib.
			\param[in]	plane		Plane.
		*/
		PolygonCollection(const Clipper2Lib::PathsD &polygons, const SO::Plane &plane);

		//! Construct from std::vector<CgalPolyline_EPICK>
		/*!
			\param[in]	polygons	Polygons in std::vector<CgalPolyline_EPICK>.
			\param[in]	plane		Plane.
		*/
		PolygonCollection(const std::vector<CgalPolyline_EPICK> &polygons, const SO::Plane &plane);

		//! Load from .txt file.
		/*!
			\param[in]	file_name	Path of the .txt file.
		*/
		PolygonCollection(std::string file_name);

		//! Destructor.
		~PolygonCollection();

		//! Save to a .txt file.
		/*!
			\param[in]	file_name	Path of the .txt file.
		*/
		bool save(std::string file_name);

		//! Load to a .txt file.
		/*!
			\param[in]	file_name	Path of the .txt file.
		*/
		bool load(std::string file_name);

		//! Get number of polygons.
		/*!
			\return \b in Number of polygons.
		*/
		int numberOfPolygons() const;

		//! Get all polygons.
		/*!
			\return \b std::vector<SO::Polygon> All polygons.
		*/
		const std::vector<SO::Polygon> &get() const;

		//! Close all the polygon if the first point do not equal the last point.
		void closePolygons();

		//! Get centroid of multiple polgyons.
		/*!
			\return \b Eigen::Vector3d Centroid.
		*/
		Eigen::Vector3d centroid() const;

		//! Get centroid of multiple polgyons.
		/*!
			\return \b Eigen::Vector3d Centroid.
		*/
		Eigen::Vector3d getCentroid() const;

		//! Get bounding box of the polygons.
		/*!
			\param bound Bounding box.
		*/
		void getBoundingBox(double bound[6]);

		//! Find closest point on the polygons to a line.
		/*!
			\param[in] line Line.
			\param[out] point Closest point.
			\return \b double Distance between point to line.
		*/
		double getClosestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;

		//! Find furthest point on the polygons to a line.
		/*!
			\param[in] line Line.
			\param[out] point Furthest point.
			\return \b double Distance between point to line.
		*/
		double getFurthestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;

		//! Check if the polygons intersect with a plane.
		/*!
			\param[in] plane Plane.
			\return \b True if yes \n \b False in no.
		*/
		bool isIntersectedWithPlane(const SO::Plane &plane) const;

		//! Check if the polygons intersect with a plane.
		/*!
			\param[in] plane Plane.
			\return \b std::vector<Eigen::Vector3d> Intersecting points.
		*/
		std::vector<Eigen::Vector3d> getIntersectionWithPlane(const SO::Plane &plane) const;

		//! Clip the polygons with a plane.
		/*!
			\param[in]	plane			Plane.
			\param[out] positive_side	Polygons on postive side.
			\param[out] negative_side	Polygons on negative side.
			\return \b True if the plane clip the polygons into two polygons. \n \b False if plane does not clip the polygons.
		*/
		bool clipWithPlane(const SO::Plane &plane, SO::PolygonCollection &positive_side, SO::PolygonCollection &negative_side) const;

		//! Get furthest point from polygon to a plane.
		/*!
			\param[in]	plane			Plane.
			\return \b double Distance from that point to the given plane.
		*/
		double getMaximumDistanceFromPlane(const SO::Plane &plane) const;

		//! Get furthest point from polygon to a plane.
		/*!
			\param[in]	plane	Plane.
			\param[out] point	Furthest point.
			\return \b double Distance from that point to the given plane.
		*/
		double getMaximumDistanceFromPlane(const SO::Plane &plane, Eigen::Vector3d &point) const;

		//! Get closest point from polygon to a plane.
		/*!
			\param[in]	plane			Plane.
			\return \b double Distance from that point to the given plane.
		*/
		double getMinimumDistanceFromPlane(const SO::Plane &plane) const;

		//! Get closest point from polygon to a plane.
		/*!
			\param[in]	plane	Plane.
			\param[out] point	Closest point.
			\return \b double Distance from that point to the given plane.
		*/
		double getMinimumDistanceFromPlane(const SO::Plane &plane, Eigen::Vector3d &point) const;

		//! Transform polygons to another plane.
		/*!
			\param[in]	plane	Target plane.
			\return \b PolygonCollection Transformed polygons.
		*/
		PolygonCollection getTransformedPolygons(const SO::Plane &plane) const;

		//! Transform polygons from one plane to another plane.
		/*!
			\param[in]	source_plane	Source plane.
			\param[in]	target_plane	Target plane.
			\return \b PolygonCollection Transformed polygons.
		*/
		PolygonCollection getTransformedPolygons(const SO::Plane &source_plane, const SO::Plane &target_plane) const;

		//! Translate polygons to other origin.
		/*!
			\param[in]	new_origin		Target origin.
			\return \b PolygonCollection Translated polygons.
		*/
		PolygonCollection getTranslatedPolygons(const Eigen::Vector3d &new_origin) const;

		//! Project this polygon to other plane.
		/*!
			\param[in]	plane				Target plane.
			\return \b PolygonCollection	Projected polygons.
		*/
		PolygonCollection projectToOtherPlane(const SO::Plane &plane) const;

		//! Project other polygon to this plane.
		/*!
			\param[in]	other				Other polygons.
			\return \b PolygonCollection	Projected polygons.
		*/
		PolygonCollection getProjection(const PolygonCollection &other);

		//! Get offsetted polygons.
		/*!
			\param[in]	distance			Offset distance.
			\return \b PolygonCollection	Offsetted polygons.
		*/
		PolygonCollection getOffset(double distance);

		//! Subtract this polygons with other polygons.
		/*!
			\brief		result = this - other
			\param[in]	other	Other polygons.
			\return \b PolygonCollection Remained polygons.
		*/
		PolygonCollection getDifference(const PolygonCollection &other);

		//! Get intersection of two polygons.
		/*!
			\param[in]	other	Other polygons.
			\return \b PolygonCollection Intersecting region.
		*/
		PolygonCollection getIntersection(const PolygonCollection &other);

		//! Get Union of two polygons.
		/*!
			\param[in]	other	Other polygons.
			\return \b PolygonCollection Union.
		*/
		PolygonCollection getUnion(const PolygonCollection &other);

		//! Get largest polygon in this polygons.
		/*!
			\return \b Polygon Largest polygon.
		*/
		Polygon getLargestPolygon() const;

		//! Get convex hull of in this polygons.
		/*!
			\return \b Polygon Convex hull.
		*/
		Polygon getConvexHullPolygon() const;

		//! Get closest polygon in this polygons to other polygon.
		/*!
			\param[in]	polygon	Other polygons.
			\return \b Polygon Closest polygon.
		*/
		Polygon getPolygonCloestToPolygon(const Polygon &polygon) const;

		//! Add polygon into this polygons.
		/*!
			\param[in]	polygon	Other polygon.
		*/
		void addPolygon(const Polygon &polygon);

		//! Add polygons into this polygons.
		/*!
			\param[in]	other	Other polygons.
		*/
		void addPolygons(const PolygonCollection &other);
		
		//! Add polygon into this polygons.
		/*!
			\param[in]	polygon	Other polygon.
		*/
		void push_back(const Polygon &polygon);

		//! Add polygons into this polygons.
		/*!
			\param[in]	other	Other polygons.
		*/
		void push_back(const PolygonCollection &other);

		//! Add polygon into this polygons.
		/*!
			\param[in]	polygon	Other polygon.
		*/
		void emplace_back(const Polygon &polygon);

		//! Add polygons into this polygons.
		/*!
			\param[in]	other	Other polygons.
		*/
		void emplace_back(const PolygonCollection &other);

		//! Remove the last polygon from this polygons.
		void pop_back();

		//! Get number of polygon in this polygons.
		/*!
			\return \b int	Number of polygon.
		*/
		int size();

		//! Restore to default value.
		void clear();

		//! Remove polygons below a given plane.
		/*!
			\param[in] plane Plane.
			\return \b int	Number of polygons removed.
		*/
		int removePolygonsBelowPlane(const SO::Plane &plane);

		//! Set plane of this polygons.
		/*!
			\param[in] plane Plane.
		*/
		void setPlane(const Plane &plane);

		//! Get plane of this polygons.
		/*!
			\return \b Plane	Plane.
		*/
		const Plane &getPlane() const;

		//! Restore to default value.
		void reset();

		//! Copy assignment operator.
		PolygonCollection &operator=(const PolygonCollection &other);

		//! Get the n-th polygon in this polygons.
		/*!
			\param[in] index Index.
			\return \b Polygon	Polygon.
		*/
		Polygon &operator[](unsigned int index);

		//! Get the n-th polygon in this polygons.
		/*!
			\param[in] index Index.
			\return \b Polygon	Polygon.
		*/
		const Polygon &operator[](unsigned int index) const;

	protected:

		//! Convert this to clipper2 polygons.
		/*!
			\return \b Clipper2Lib::PathsD	Clipper2 polygons.
		*/
		Clipper2Lib::PathsD getClipper2Polygons();

		//! Store all the polygons.
		std::vector<Polygon> m_polygons;

		//! Store the plane.
		Plane m_plane;

		//! Store whether the bounding box is calculated.
		bool m_boudning_box_calculated = false;

		//! Store the bounding box.
		double m_bounding_box[6] = { 0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0 };
	};
}

#endif  // SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_