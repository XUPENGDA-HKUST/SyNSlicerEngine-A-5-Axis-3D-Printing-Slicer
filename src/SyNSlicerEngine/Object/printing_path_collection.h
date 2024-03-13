#ifndef SYNSLICERENGINE_OBJECT_PRINTINGPATHCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_PRINTINGPATHCOLLECTION_H_

#include "Object/polygon_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object
{
	class PrintingPathCollection
	{
	public:
		PrintingPathCollection();
		~PrintingPathCollection();

		void reset();

		PolygonCollection &getSurface();
		std::vector<PolygonCollection> &getWall();
		std::vector<PolygonCollection> &getBottom();
		std::vector<PolygonCollection> &getTop();
		PolygonCollection &getBottomTopUnion();
		PolygonCollection &getInfill();

	private:
		PolygonCollection m_surface;
		
		// m_wall.back() will be used to calculate m_bottom, m_top, m_infill.
		std::vector<PolygonCollection> m_wall;
		std::vector<PolygonCollection> m_bottom;
		std::vector<PolygonCollection> m_top;
		PolygonCollection m_bottom_top_union;
		PolygonCollection m_infill;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_PRINTINGPATHCOLLECTION_H_