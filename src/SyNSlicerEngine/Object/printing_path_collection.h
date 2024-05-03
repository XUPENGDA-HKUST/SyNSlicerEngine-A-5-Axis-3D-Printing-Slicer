#ifndef SYNSLICERENGINE_OBJECT_PRINTINGPATHCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_PRINTINGPATHCOLLECTION_H_

#include "Object/polygon_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object
{
	//! Container to store all printing path in a printing layer.
	class PrintingPathCollection
	{
	public:
		//! Default constructor.
		PrintingPathCollection();

		//! Destructor.
		~PrintingPathCollection();

		//! Restore to default value.
		void reset();

		//! Get surface contours.
		/*!
			\return \b PolygonCollection Contours for surface.
		*/
		PolygonCollection &getSurface();

		//! Get wall contours.
		/*!
			\return \b PolygonCollection Contours for walls.
		*/
		std::vector<PolygonCollection> &getWall();

		//! Get bottom contours.
		/*!
			\return \b PolygonCollection Contours for bottom.
		*/
		std::vector<PolygonCollection> &getBottom();

		//! Get top contours.
		/*!
			\return \b PolygonCollection Contours for top.
		*/
		std::vector<PolygonCollection> &getTop();

		//! Get union of top and bottom contours.
		/*!
			\return \b PolygonCollection Contours for union of top and bottom.
		*/
		PolygonCollection &getBottomTopUnion();

		//! Get infill contours.
		/*!
			\return \b PolygonCollection Contours for infill.
		*/
		PolygonCollection &getInfill();

	protected:
		//! Store contours for surface.
		PolygonCollection m_surface;
		
		//! Store contours for wall.
		/*!
			m_wall.back() will be used to calculate m_bottom, m_top, m_infill.
		*/ 
		std::vector<PolygonCollection> m_wall;

		//! Store contours for bottom.
		std::vector<PolygonCollection> m_bottom;

		//! Store contours for top.
		std::vector<PolygonCollection> m_top;

		//! Store contours for union of top and bottom.
		PolygonCollection m_bottom_top_union;

		//! Store contours for infill.
		PolygonCollection m_infill;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_PRINTINGPATHCOLLECTION_H_