#ifndef SYNSLICERGUI_OBJECTDRAWER_H_
#define SYNSLICERGUI_OBJECTDRAWER_H_

#include <string>
#include <map>
#include <vector>

#include <vtkActor.h>

#include "Object/line.h"
#include "Object/plane.h"
#include "Object/partition.h"
#include "Object/polyline_collection.h"

#include "object_for_visualization.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerGUI {

	class ObjectDrawer
	{
	public:
		ObjectDrawer() = delete;
		ObjectDrawer(vtkRenderer *p_renderer);
		~ObjectDrawer();

		vtkRenderer *getRenderer();

		std::string drawPoint(const Eigen::Vector3d &point, std::string &name);
		std::string drawPoints(const std::vector<Eigen::Vector3d> &points, std::string &name);
		std::string drawLine(const SO::Line &line, std::string &name);
		std::string drawPolyline(const SO::Polyline &polygon, std::string &name);
		std::string drawPolylines(const SO::PolylineCollection &polygons, std::string &name);
		std::string drawPolygon(const SO::Polygon &polygon, std::string &name);
		std::string drawPolygons(SO::PolygonCollection &polygons, std::string &name);
		std::string drawPlane(const SO::Plane &plane, std::string &name);
		bool removePlane(std::string name);
		std::string drawTriangles(std::vector<int> faces, const CgalMesh_EPICK &mesh, std::string &name);
		std::string drawTriangles(std::vector<CgalMesh_EPICK::Face_index> faces, const CgalMesh_EPICK &mesh, std::string &name);
		std::string drawMesh(const CgalMesh_EPICK &mesh, std::string &name);
		std::string drawMesh(const CgalMesh_EPECK &mesh, std::string &name);

		void setColor(std::string name, double r, double g, double b);
		void setOpacity(std::string name, double opacity);
		void setVisible(std::string name, bool visible);

		int numberOfObjectsDrawn();
		ObjectForVisualization *getObjectDrawn(std::string name);
		std::map<std::string, SyNSlicerGUI::ObjectForVisualization *> getAllObjectDrawn();

		bool removeObjectDrawn(std::string name);
		int removeAllObjectsDrawn();

	private:
		std::string addObjectToRenderer(vtkPolyData *p_polydata, std::string &name);

		vtkRenderer *mp_renderer;
		std::map<std::string, SyNSlicerGUI::ObjectForVisualization *> m_object_in_renderer;
	};
}

#endif // SYNSLICERGUI_OBJECTDRAWER_H_