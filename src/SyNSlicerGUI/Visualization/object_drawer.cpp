#include "object_drawer.h"

using SyNSlicerGUI::ObjectDrawer;
using SyNSlicerGUI::ObjectForVisualization;

ObjectDrawer::ObjectDrawer(vtkRenderer *p_renderer)
	: mp_renderer(p_renderer)
{

}

ObjectDrawer::~ObjectDrawer()
{
	this->removeAllObjectsDrawn();
}

vtkRenderer *ObjectDrawer::getRenderer()
{
	return mp_renderer;
}

std::string ObjectDrawer::drawPoint(const Eigen::Vector3d &point, std::string &name)
{
	vtkNew<vtkPoints> points;
	// Create the topology of the point (a vertex)
	vtkNew<vtkCellArray> vertices;
	// We need an an array of point id's for InsertNextCell.
	vtkIdType pid[1];

	pid[0] = points->InsertNextPoint(point[0], point[1], point[2]);
	vertices->InsertNextCell(1, pid);

	// Create a polydata object
	vtkNew<vtkPolyData> pointcloud;

	// Set the points and vertices we created as the geometry and topology of the polydata
	pointcloud->SetPoints(points);
	pointcloud->SetVerts(vertices);

	this->addObjectToRenderer(pointcloud, name);
	m_object_in_renderer[name]->getProperty()->SetPointSize(10);
	m_object_in_renderer[name]->setColor(1, 0, 0);

	return name;
}

std::string ObjectDrawer::drawPoints(const std::vector<Eigen::Vector3d> &points, std::string &name)
{
	vtkNew<vtkPoints> vtk_points;
	// Create the topology of the point (a vertex)
	vtkNew<vtkCellArray> vtk_vertices;
	// We need an an array of point id's for InsertNextCell.
	vtkIdType pid[1];

	for (int i = 0; i < points.size(); i++)
	{
		pid[0] = vtk_points->InsertNextPoint(points[i][0], points[i][1], points[i][2]);
		vtk_vertices->InsertNextCell(1, pid);
	}

	// Create a polydata object
	vtkNew<vtkPolyData> polydata;

	// Set the points and vertices we created as the geometry and topology of the polydata
	polydata->SetPoints(vtk_points);
	polydata->SetVerts(vtk_vertices);

	this->addObjectToRenderer(polydata, name);
	m_object_in_renderer[name]->getProperty()->SetPointSize(10);
	m_object_in_renderer[name]->setColor(1, 0, 0);

	return name;
}

std::string ObjectDrawer::drawLine(const SO::Line &line, std::string &name)
{
	int pointID = 0;
	vtkNew<vtkPoints> points;
	vtkNew<vtkCellArray> cells;

	points->InsertNextPoint(line.getSource()[0], line.getSource()[1], line.getSource()[2]);
	points->InsertNextPoint(line.getTarget()[0], line.getTarget()[1], line.getTarget()[2]);

	vtkNew<vtkLine> vtk_line;
	vtk_line->GetPointIds()->SetId(0, 0);
	vtk_line->GetPointIds()->SetId(1, 1);
	cells->InsertNextCell(vtk_line);

	vtkNew<vtkPolyData> line_poly_data;
	line_poly_data->SetPoints(points);
	line_poly_data->SetLines(cells);

	this->addObjectToRenderer(line_poly_data, name);
	m_object_in_renderer[name]->setColor(1, 0, 0);

	return name;
}

std::string ObjectDrawer::drawPolyline(const SO::Polyline &polyline, std::string &name)
{
	int pointID = 0;
	vtkNew<vtkPoints> points;
	vtkNew<vtkCellArray> cells;

	for (int j = 0; j < polyline.numberOfPoints(); j++)
	{
		points->InsertNextPoint(polyline[j][0], polyline[j][1], polyline[j][2]);
	}
	vtkNew<vtkPolyLine> polyLine;
	polyLine->GetPointIds()->SetNumberOfIds(polyline.numberOfPoints());
	for (unsigned int k = 0; k < polyline.numberOfPoints(); k++)
	{
		polyLine->GetPointIds()->SetId(k, pointID);
		++pointID;
	}
	cells->InsertNextCell(polyLine);

	vtkNew<vtkPolyData> polyline_poly_data;
	polyline_poly_data->SetPoints(points);
	polyline_poly_data->SetLines(cells);

	this->addObjectToRenderer(polyline_poly_data, name);

	return name;
}

std::string ObjectDrawer::drawPolylines(const SO::PolylineCollection &polylines, std::string &name)
{
	int pointID = 0;
	vtkNew<vtkPoints> points;
	vtkNew<vtkCellArray> cells;

	for (int i = 0; i < polylines.numberOfPolylines(); i++)
	{
		for (int j = 0; j < polylines[i].numberOfPoints(); j++)
		{
			points->InsertNextPoint(polylines[i][j][0], polylines[i][j][1], polylines[i][j][2]);
		}
		vtkNew<vtkPolyLine> polyLine;
		polyLine->GetPointIds()->SetNumberOfIds(polylines[i].numberOfPoints());
		for (unsigned int k = 0; k < polylines[i].numberOfPoints(); k++)
		{
			polyLine->GetPointIds()->SetId(k, pointID);
			++pointID;
		}
		cells->InsertNextCell(polyLine);
	}

	vtkNew<vtkPolyData> polyline_poly_data;
	polyline_poly_data->SetPoints(points);
	polyline_poly_data->SetLines(cells);

	this->addObjectToRenderer(polyline_poly_data, name);

	return name;
}

std::string ObjectDrawer::drawPolygon(const SO::Polygon &polygon, std::string &name)
{
	int pointID = 0;
	vtkNew<vtkPoints> points;
	vtkNew<vtkCellArray> cells;

	for (int j = 0; j < polygon.numberOfPoints(); j++)
	{
		points->InsertNextPoint(polygon[j][0], polygon[j][1], polygon[j][2]);
	}
	vtkNew<vtkPolyLine> polyLine;
	polyLine->GetPointIds()->SetNumberOfIds(polygon.numberOfPoints());
	for (unsigned int k = 0; k < polygon.numberOfPoints(); k++)
	{
		polyLine->GetPointIds()->SetId(k, pointID);
		++pointID;
	}
	cells->InsertNextCell(polyLine);
	
	vtkNew<vtkPolyData> polyline_poly_data;
	polyline_poly_data->SetPoints(points);
	polyline_poly_data->SetLines(cells);

	this->addObjectToRenderer(polyline_poly_data, name);

	return name;
}

std::string ObjectDrawer::drawPolygons(SO::PolygonCollection &polygons, std::string &name)
{
	int pointID = 0;
	vtkNew<vtkPoints> points;
	vtkNew<vtkCellArray> cells;

	for (int i = 0; i < polygons.numberOfPolygons(); i++)
	{
		for (int j = 0; j < polygons[i].numberOfPoints(); j++)
		{
			points->InsertNextPoint(polygons[i][j][0], polygons[i][j][1], polygons[i][j][2]);
		}
		vtkNew<vtkPolyLine> polyLine;
		polyLine->GetPointIds()->SetNumberOfIds(polygons[i].numberOfPoints());
		for (unsigned int k = 0; k < polygons[i].numberOfPoints(); k++)
		{
			polyLine->GetPointIds()->SetId(k, pointID);
			++pointID;
		}
		cells->InsertNextCell(polyLine);
	}

	vtkNew<vtkPolyData> polyline_poly_data;
	polyline_poly_data->SetPoints(points);
	polyline_poly_data->SetLines(cells);

	this->addObjectToRenderer(polyline_poly_data, name);

	return name;
}

std::string ObjectDrawer::drawPlane(const SO::Plane &plane, std::string &name)
{
	vtkNew<vtkPlaneSource> p_plane_source;
	p_plane_source->SetOrigin(0, 0, 0);
	p_plane_source->SetPoint1(20, 0, 0);
	p_plane_source->SetPoint2(0, 20, 0);
	p_plane_source->SetCenter(plane.getOrigin()[0], plane.getOrigin()[1], plane.getOrigin()[2]);
	p_plane_source->SetNormal(plane.getNormal()[0], plane.getNormal()[1], plane.getNormal()[2]);
	p_plane_source->Update();
	this->addObjectToRenderer(p_plane_source->GetOutput(), name);

	std::string origin_name = std::string("Origin") + name;
	this->drawPoint(plane.getOrigin(), origin_name);

	std::string normal_name = std::string("Normal") + name;
	this->drawLine(SO::Line(plane.getOrigin(), plane.getNormal(), 10), normal_name);
	this->setColor(normal_name, 1, 0, 0);

	return name;
}

bool ObjectDrawer::removePlane(std::string name)
{
	bool result = true;
	result = result && this->removeObjectDrawn(name);
	result = result && this->removeObjectDrawn(std::string("Origin") + name);
	result = result && this->removeObjectDrawn(std::string("Normal") + name);
	return result;
}

std::string ObjectDrawer::drawTriangles(std::vector<int> faces, const CgalMesh_EPICK &mesh, std::string &name)
{
	vtkNew<vtkPoints> points;
	vtkNew<vtkCellArray> triangles;

	int v0 = 0;
	int v1 = 1;
	int v2 = 2;

	for (auto f : faces)
	{
		for (auto v : mesh.vertices_around_face(mesh.halfedge(CgalMesh_EPICK::Face_index(f))))
		{
			points->InsertNextPoint(mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z());
		}

		vtkNew<vtkTriangle> triangle;
		triangle->GetPointIds()->SetId(0, v0);
		triangle->GetPointIds()->SetId(1, v1);
		triangle->GetPointIds()->SetId(2, v2);

		triangles->InsertNextCell(triangle);
		v0 += 3;
		v1 += 3;
		v2 += 3;
	}

	vtkNew<vtkPolyData> trianglePolyData;
	trianglePolyData->SetPoints(points);
	trianglePolyData->SetPolys(triangles);

	this->addObjectToRenderer(trianglePolyData, name);
	this->setColor(name, 1, 0, 0);

	return name;
}

std::string ObjectDrawer::drawTriangles(std::vector<CgalMesh_EPICK::Face_index> faces,
	const CgalMesh_EPICK &mesh, std::string &name)
{
	vtkNew<vtkPoints> points;
	vtkNew<vtkCellArray> triangles;

	int v0 = 0;
	int v1 = 1;
	int v2 = 2;

	for (auto &f : faces)
	{
		for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
		{
			points->InsertNextPoint(mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z());
		}

		vtkNew<vtkTriangle> triangle;
		triangle->GetPointIds()->SetId(0, v0);
		triangle->GetPointIds()->SetId(1, v1);
		triangle->GetPointIds()->SetId(2, v2);

		triangles->InsertNextCell(triangle);
		v0 += 3;
		v1 += 3;
		v2 += 3;
	}

	vtkNew<vtkPolyData> trianglePolyData;
	trianglePolyData->SetPoints(points);
	trianglePolyData->SetPolys(triangles);

	this->addObjectToRenderer(trianglePolyData, name);
	this->setColor(name, 1, 0, 0);

	return name;
}

std::string ObjectDrawer::drawMesh(const CgalMesh_EPICK &mesh, std::string &name)
{
	if (CGAL::IO::write_polygon_mesh("temp.stl", mesh))
	{
		std::cout << "CGAL Write STL successfully!" << std::endl;
	}

	vtkNew<vtkSTLReader> reader;
	reader->SetFileName("temp.stl");
	reader->Update();

	this->addObjectToRenderer(reader->GetOutput(), name);

	return name;
}

std::string ObjectDrawer::drawMesh(const CgalMesh_EPECK &mesh, std::string &name)
{
	if (CGAL::IO::write_polygon_mesh("temp.stl", mesh))
	{
		std::cout << "CGAL Write STL successfully!" << std::endl;
	}

	vtkNew<vtkSTLReader> reader;
	reader->SetFileName("temp.stl");
	reader->Update();

	this->addObjectToRenderer(reader->GetOutput(), name);

	return name;
}

void ObjectDrawer::setColor(std::string name, double r, double g, double b)
{
	m_object_in_renderer[name]->setColor(r, g, b);
}

void ObjectDrawer::setOpacity(std::string name, double opacity)
{
	m_object_in_renderer[name]->setOpacity(opacity);
}

void ObjectDrawer::setVisible(std::string name, bool visible)
{
	m_object_in_renderer[name]->setVisible(visible);
}

void ObjectDrawer::setProperty(std::string name, vtkProperty *vtk_property)
{
	m_object_in_renderer[name]->getProperty()->DeepCopy(vtk_property);
}

int ObjectDrawer::numberOfObjectsDrawn()
{
	return m_object_in_renderer.size();
}

ObjectForVisualization *ObjectDrawer::getObjectDrawn(std::string name)
{
	return m_object_in_renderer[name];
}

std::map<std::string, SyNSlicerGUI::ObjectForVisualization *> ObjectDrawer::getAllObjectDrawn()
{
	return m_object_in_renderer;
}

bool ObjectDrawer::removeObjectDrawn(std::string name)
{
	auto it = m_object_in_renderer.find(name);
	if (it == m_object_in_renderer.end())
	{
		return false;
	}
	mp_renderer->RemoveActor(m_object_in_renderer[name]->getActor());
	delete m_object_in_renderer[name];
	m_object_in_renderer.erase(name);
	return true;
}

int ObjectDrawer::removeAllObjectsDrawn()
{
	int number_of_objects_removed = m_object_in_renderer.size();
	for (auto &[name, actor] : m_object_in_renderer)
	{
		mp_renderer->RemoveActor(m_object_in_renderer[name]->getActor());
		delete m_object_in_renderer[name];
	}
	m_object_in_renderer.clear();
	assert(m_object_in_renderer.size() == 0);
	return number_of_objects_removed;
}

std::string ObjectDrawer::addObjectToRenderer(vtkPolyData *p_polydata, std::string &name)
{
	ObjectForVisualization *object = new SyNSlicerGUI::ObjectForVisualization();
	object->setPolyData(p_polydata);
	object->addToRenderer(mp_renderer);
	
	const auto [it, success] =  m_object_in_renderer.insert(std::make_pair(name, object));
	bool status = success;
	int index = this->numberOfObjectsDrawn();
	while (status == false)
	{
		name = std::to_string(index);
		const auto [it, success] = m_object_in_renderer.insert(std::make_pair(name, object));
		status = success;
		index += 1;
	}
	return name;
}