#ifndef SYNSLICERGUI_OBJECTDRAWER_H_
#define SYNSLICERGUI_OBJECTDRAWER_H_

#include <string>
#include <map>
#include <vector>

#include <vtkActor.h>

#include "Object/line.h"
#include "Object/plane.h"
#include "Object/partition.h"

#include "object_for_visualization.h"

namespace SO = SyNSlicerEngine::Object;

namespace GUI {

	class ObjectDrawer
	{
	public:
		ObjectDrawer() = delete;
		ObjectDrawer(vtkRenderer *p_renderer);
		~ObjectDrawer();

		void drawPoint(const Eigen::Vector3d &point, std::string name);
		void drawLine(const SO::Line &line, std::string name);
		void drawPolygon(const SO::Polygon &polygon, std::string name);
		void drawPolygons(const SO::PolygonCollection &polygons, std::string name);
		void drawPlane(const SO::Plane &plane, std::string name);
		void drawMesh(const CgalMesh_EPICK &mesh, std::string name);
		void drawMesh(const CgalMesh_EPECK &mesh, std::string name);

		void setColor(std::string name, double r, double g, double b);
		void setOpacity(std::string name, double opacity);

		GUI::ObjectForVisualization *getObjectDrawn(std::string name);
		void removeObjectDrawn(std::string name);

	private:
		void addObjectToRenderer(vtkPolyData *p_polydata, std::string name);

		vtkRenderer *mp_renderer;
		std::map<std::string, GUI::ObjectForVisualization *> m_object_in_renderer;

	};

}

#endif // SYNSLICERGUI_OBJECTDRAWER_H_